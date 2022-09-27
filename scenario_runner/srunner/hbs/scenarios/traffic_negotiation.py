#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Object crash without prior vehicle action scenario:
The scenario realizes the user controlled ego vehicle
moving along the road and encountering a cyclist ahead.
"""

from __future__ import print_function

import math
import py_trees
import carla
import random

from srunner.tools.scenario_helper import generate_target_waypoint_list_multilane
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.hbs.scenarioatomics.atomic_behaviors import (HBSChangeAutoPilot)

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      AccelerateToVelocity,
                                                                      ChangeAutoPilot,
                                                                      HandBrakeVehicle,
                                                                      KeepVelocity,
                                                                      SyncArrival,
                                                                      StopVehicle,
                                                                      WaypointFollower)

from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocationAlongRoute,
                                                                               InTimeToArrivalToVehicle,
                                                                               InTimeToArrivalToLocation,
                                                                               InTriggerDistanceToLocation,
                                                                               InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               InTriggerRegion,
                                                                               DriveDistance,
                                                                               StandStill)

from srunner.hbs.scenarioatomics.atomic_behaviors import (ChangeEgoSpeed, DecelerateToStop)

from srunner.hbs.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocationOneShot,
                                                                   TargetsInProximityViewOfActor,
                                                                   TargetsNotInProximityViewOfActor,
                                                                   InProximityViewOfActor,
                                                                   InProximityViewOfActorOneShot,
                                                                   NotInProximityOfActor,
                                                                   TargetProximityInfo,
                                                                   OutsideProximityViewOfStoppedActor)




from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_location_in_distance_from_wp
from srunner.tools.scenario_helper import get_waypoint_in_distance


'''
Intersection
- other
  - start
  - end
  - sync
- ego
  - trigger
'''
class TrafficNegotiation(BasicScenario):

    """
    Implementation class for
    'Non-signalized junctions: crossing negotiation' scenario,
    (Traffic Scenario 10).
    This is a single ego vehicle scenario
    """

    # ego vehicle parameters
    _ego_vehicle_max_velocity = 20
    _ego_vehicle_driven_distance = 105

    # other vehicle
    _other_actor_max_brake = 1.0
    _other_actor_target_velocity = 15

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._wmap.get_waypoint(self._trigger_location)
        self._other_actor_transform = None
        self.timeout = timeout
        super(TrafficNegotiation, self).__init__("TrafficNegotiation",
                                                ego_vehicles,
                                                config,
                                                world,
                                                debug_mode,
                                                criteria_enable=criteria_enable)


    def _initialize_actors(self, config):

        for actor_config in config.other_actors:
            if actor_config.destination_transform and (actor_config.rolename == 'pedestrian' or actor_config.rolename == 'cyclist'):
                # Orient the actor towards its destination
                other_actor_destination_dir = actor_config.destination_transform.location - actor_config.transform.location
                other_actor_destination_dir = other_actor_destination_dir.make_unit_vector()
                actor_config.transform.rotation.yaw = math.degrees(math.atan2(other_actor_destination_dir.y, other_actor_destination_dir.x)) 
            
            # import pdb; pdb.set_trace()
            actor = CarlaDataProvider.request_new_actor(actor_config.type, actor_config.transform, actor_category=actor_config.rolename)
            if actor:
                # actor.set_simulate_physics(enabled=False)
                actor.speed = actor_config.speed
                actor.transform = actor_config.transform
                actor.sync_point = actor_config.sync_point
                actor.destination_transform = actor_config.destination_transform
                # actor.destination_radius = actor_config.safety_radius
                # actor.destination_is_waypoint = actor_config.destination_is_waypoint
                actor.rolename = actor_config.rolename
                actor.awareness_distance = actor_config.awareness_distance
                actor.trigger_distance = actor_config.trigger_distance
                actor.path = actor_config.path
                actor.path_is_terminal = actor_config.path_is_terminal
                self.other_actors.append( actor )

                if actor.rolename == 'vehicle' or actor.rolename == 'cyclist':
                    actor.set_simulate_physics(enabled=True)

    def _find_first_vehicle(self):
        vehicle = None
        for actor in self.other_actors:
            if actor.rolename == 'vehicle':
                vehicle = actor
                break
        return vehicle

    def _find_first_vehicle_with_syncpoint(self):
        vehicle = None
        for actor in self.other_actors:
            if actor.rolename == 'vehicle' and actor.sync_point:
                vehicle = actor
                break
        return vehicle        

    def _create_other_vehicle_behavior(self, parent_behavior):

        for i, vehicle in enumerate(self.other_actors):
            if vehicle.rolename == 'vehicle' or vehicle.rolename == 'cyclist':
                behavior = py_trees.composites.Sequence(name="Vehicle " + str(i) + " behavior")

                path_end_condition = InTriggerDistanceToLocation(vehicle, vehicle.path[-1].location, 5)

                drive_plan = []
                for p in vehicle.path:
                    drive_plan.append( p.location )

                drive_to_sync_point = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                drive_to_sync_point.add_child(WaypointFollower(vehicle, target_speed=vehicle.speed, plan=drive_plan))
                drive_to_sync_point.add_child(path_end_condition)

                behavior.add_child(drive_to_sync_point)

                if vehicle.path_is_terminal:
                    behavior.add_child(StopVehicle( vehicle, 1, name="Stop vehicle"))
                else:
                    behavior.add_child(WaypointFollower(vehicle, target_speed=vehicle.speed, name="Follow waypoints"))
                    
                parent_behavior.add_child(behavior)

    def _create_ego_vehicle_behavior(self, parent_behavior):
        behavior = py_trees.composites.Sequence(name="Ego vehicle behavior")
        drive_conditions = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        stop_conditions = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        for i, vehicle in enumerate(self.other_actors):
            # if not vehicle.sync_point: continue
            if not (vehicle.rolename == 'vehicle' or vehicle.rolename == 'cyclist'): continue
            if vehicle.awareness_distance==0: continue

            drive_conditions.add_child(InTriggerDistanceToLocationOneShot( vehicle, vehicle.path[-1].location, 5,name='egodrive'))
            # stop_conditions.add_child(InTriggerDistanceToLocationOneShot(
            #     self.ego_vehicles[0],
            #     vehicle.sync_point.transform.location,
            #     vehicle.sync_point.radius,
            #     name='egostop'))

            #reference_actor, actor, distance
            # stop_conditions.add_child(InTriggerDistanceToVehicle(
            #     vehicle, 
            #     self.ego_vehicles[0],
            #     vehicle.awareness_distance,
            #     name='egostop'))

            stop_conditions.add_child(InProximityViewOfActorOneShot(
                actor=self.ego_vehicles[0],
                target=vehicle,
                distance=vehicle.awareness_distance,
                view_angle=0.0,
                name="egostop"
            ))                
            

        speed_params = {}
        speed_params["max_speed"] = self.ego_vehicles[0].get_speed_limit()
        speed_params["ignore_lights_percentage"] = 100
        speed_params["ignore_signs_percentage"] = 100            

        start_behavior = py_trees.composites.Sequence()
        # start_behavior = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        start_behavior.add_child(HBSChangeAutoPilot(actor=self.ego_vehicles[0], activate=True, parameters=speed_params, name="Start"))
        start_behavior.add_child(stop_conditions)

        stop_behavior = py_trees.composites.Sequence()
        # stop_behavior = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        stop_params = {}
        stop_params["max_speed"] = 0
        stop_params["ignore_lights_percentage"] = 100
        stop_params["ignore_signs_percentage"] = 100
        stop = HBSChangeAutoPilot(actor=self.ego_vehicles[0], activate=True, parameters=stop_params, name="Stop")
        stop_behavior.add_child(stop)
        stop_behavior.add_child(drive_conditions)

        drive_behavior = py_trees.composites.Sequence()
        drive = HBSChangeAutoPilot(actor=self.ego_vehicles[0], activate=True, parameters=speed_params, name="Drive")
        drive_behavior.add_child(drive)

        # behavior.add_child(HBSChangeAutoPilot(actor=self.ego_vehicles[0], activate=True, parameters=start_params, name="Start"))
        behavior.add_child(start_behavior)
        behavior.add_child(stop_behavior)
        behavior.add_child(drive_behavior)
        parent_behavior.add_child(behavior)

    def _create_behavior(self):

        behaviors = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        self._create_other_vehicle_behavior( behaviors )
        self._create_ego_vehicle_behavior( behaviors )

        root = py_trees.composites.Sequence(name="No Signal Junction Behavior")
        root.add_child(behaviors)
        root.add_child(DriveDistance(self.ego_vehicles[0], TrafficNegotiation._ego_vehicle_driven_distance, name="EndCondition"))
        root.add_child(StandStill(self.ego_vehicles[0], name="wait speed", duration=1))

        for i, actor in enumerate(self.other_actors):
            root.add_child(ActorDestroy(actor, name="DestroyActor" + str(i)))

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collison_criteria = CollisionTest(self.ego_vehicles[0])
        criteria.append(collison_criteria)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
