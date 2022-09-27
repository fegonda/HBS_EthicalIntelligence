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
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      AccelerateToVelocity,
                                                                      ChangeAutoPilot,
                                                                      ChangeEgoSpeed,
                                                                      DecelerateToStop,
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
                                                                               InTriggerDistanceToLocationOneShot,
                                                                               InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               InTriggerRegion,
                                                                               InProximityViewOfActor,
                                                                               InProximityViewOfActorOneShot,
                                                                               TargetProximityInfo,
                                                                               TargetsInProximityViewOfActor,
                                                                               TargetsNotInProximityViewOfActor,
                                                                               NotInProximityOfActor,
                                                                               OutsideProximityViewOfStoppedActor,
                                                                               DriveDistance,
                                                                               WaitUntilClear,
                                                                               StandStill)




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

    def _create_other_vehicle_behaviordd(self, parent_behavior):

        for i, vehicle in enumerate(self.other_actors):
            if vehicle.rolename == 'vehicle' or vehicle.rolename == 'cyclist':
                behavior = py_trees.composites.Sequence(name="Vehicle " + str(i) + " behavior")

                in_sync_proximity = InTriggerDistanceToLocation(
                                vehicle,
                                vehicle.sync_point.transform.location,
                                vehicle.sync_point.radius,
                                name='Arrived sync point condition')

                drive_to_sync_plan = []
                for p in vehicle.path:
                    drive_to_sync_plan.append( p.location )

                drive_to_sync_point = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                drive_to_sync_point.add_child(WaypointFollower(vehicle, target_speed=vehicle.speed, plan=drive_to_sync_plan))
                drive_to_sync_point.add_child(in_sync_proximity)

                left_sync_point_condition = InTriggerDistanceToLocation(
                                vehicle,
                                vehicle.destination_transform.location,
                                vehicle.destination_radius)

                leave_sync_point_plan = []
                leave_sync_point_plan.append(vehicle.sync_point.transform.location)
                leave_sync_point_plan.append(vehicle.destination_transform.location)
                leave_sync_point = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                leave_sync_point.add_child(WaypointFollower(vehicle, target_speed=vehicle.speed, plan=leave_sync_point_plan))
                leave_sync_point.add_child(left_sync_point_condition)

                behavior.add_child(drive_to_sync_point)
                behavior.add_child(leave_sync_point)
                behavior.add_child(WaypointFollower(vehicle, target_speed=vehicle.speed))

                parent_behavior.add_child(behavior)


    def _create_other_vehicle_behaviordd(self, parent_behavior):
        vehicle = self._find_first_vehicle()
        if vehicle == None:
            return

        behavior = py_trees.composites.Sequence(name="Other vehicle behavior")

        in_sync_proximity = InTriggerDistanceToLocation(
                        vehicle,
                        vehicle.sync_point.transform.location,
                        vehicle.sync_point.radius,
                        name='Other vehicle reached sync point')

        # drive_to_sync_point = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        # drive_to_sync_point.add_child(KeepVelocity(vehicle, vehicle.speed))
        # drive_to_sync_point.add_child(other_in_sync_proximity)

        drive_to_sync_plan = []
        drive_to_sync_plan.append(vehicle.transform.location)
        drive_to_sync_plan.append(vehicle.sync_point.transform.location)
        drive_to_sync_point = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        drive_to_sync_point.add_child(WaypointFollower(vehicle, target_speed=vehicle.speed, plan=drive_to_sync_plan))
        drive_to_sync_point.add_child(in_sync_proximity)


        end_scenario_condition = InTriggerDistanceToLocation(
                        vehicle,
                        vehicle.destination_transform.location,
                        vehicle.destination_radius,
                        name='Other vehicle reached destination')


        leave_sync_point_plan = []
        leave_sync_point_plan.append(vehicle.sync_point.transform.location)
        leave_sync_point_plan.append(vehicle.destination_transform.location)
        leave_sync_point = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        leave_sync_point.add_child(WaypointFollower(vehicle, target_speed=vehicle.speed, plan=leave_sync_point_plan))
        leave_sync_point.add_child(end_scenario_condition)
        # leave_sync_point.add_child(WaypointFollower(vehicle, target_speed=vehicle.speed))

        behavior.add_child(drive_to_sync_point)
        behavior.add_child(leave_sync_point)
        behavior.add_child(WaypointFollower(vehicle, target_speed=vehicle.speed))
        # behavior.add_child(StopVehicle( vehicle, 1))
        parent_behavior.add_child(behavior)

    def _create_other_vehicle_behaviora(self, parent_behavior):
        vehicle = self._find_first_vehicle()
        if vehicle == None:
            return

        behavior = py_trees.composites.Sequence(name="Other vehicle behavior")

        sync_arrival = SyncArrival(
            vehicle, 
            self.ego_vehicles[0],
            carla.Location(x=vehicle.sync_point.transform.location.x, y=vehicle.sync_point.transform.location.y))

        ego_pass_through_trigger = InTriggerDistanceToLocation(
                        self.ego_vehicles[0],
                        vehicle.sync_point.transform.location,
                        vehicle.sync_point.radius,
                        name='Ego vehicle reached sync point')

        sync_arrival_parallel = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        sync_arrival_parallel.add_child(sync_arrival)
        sync_arrival_parallel.add_child(ego_pass_through_trigger)

        stop_vehicle_trigger = InTriggerDistanceToLocation(
                        vehicle,
                        vehicle.destination_transform.location,
                        vehicle.destination_radius,
                        name='Other vehicle reached destination')

        keep_velocity_parallel = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        # keep_velocity_other = KeepVelocity(vehicle, vehicle.speed)

        plan = []
        plan.append(vehicle.sync_point.transform.location)
        plan.append(vehicle.destination_transform.location)
        follow_waypoints = WaypointFollower(vehicle, target_speed=vehicle.speed, plan=plan)

        keep_velocity_parallel.add_child(follow_waypoints)
        # keep_velocity_parallel.add_child(keep_velocity_other)
        keep_velocity_parallel.add_child(stop_vehicle_trigger)

        behavior.add_child(sync_arrival_parallel)
        behavior.add_child(keep_velocity_parallel)
        # behavior.add_child(StopVehicle( vehicle, 1))
        parent_behavior.add_child(behavior)

    def _create_ego_vehicle_behavioraaa(self, parent_behavior):

        # drive_conditions = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        drive_conditions = py_trees.composites.Sequence()
        stop_conditions = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        for i, vehicle in enumerate(self.other_actors):
            if vehicle.rolename == 'vehicle' or vehicle.rolename == 'cyclist':
                drive_conditions.add_child(InTriggerDistanceToLocation(
                    vehicle,
                    vehicle.path[-1].location,
                    5, name='egodrive' + str(i)))        

                stop_conditions.add_child(InTriggerDistanceToLocation(
                    self.ego_vehicles[0],
                    vehicle.sync_point.transform.location,
                    vehicle.sync_point.radius))

        stop_behavior = py_trees.composites.Sequence(name="Ego stop behavior")
        stop_params = {}
        stop_params["max_speed"] = 0
        stop_params["ignore_lights_percentage"] = 100
        stop = ChangeAutoPilot(actor=self.ego_vehicles[0], activate=True, parameters=stop_params, name="Stop")
        stop_behavior.add_child(stop_conditions)
        stop_behavior.add_child(stop)

        drive_behavior = py_trees.composites.Sequence(name="Ego drive behavior")
        drive_params = {}
        drive_params["max_speed"] = self.ego_vehicles[0].get_speed_limit()
        drive_params["ignore_lights_percentage"] = 100
        drive = ChangeAutoPilot(actor=self.ego_vehicles[0], activate=True, parameters=drive_params, name="Drive")
        drive_behavior.add_child(drive_conditions)
        drive_behavior.add_child(drive)

        behavior = py_trees.composites.Sequence(name="Ego vehicle behavior")
        behavior.add_child(stop_behavior)
        behavior.add_child(drive_behavior)
        parent_behavior.add_child(behavior)


    def _create_ego_vehicle_behaviorpp(self, parent_behavior):
        # vehicle = self._find_first_vehicle()
        # if not vehicle:
        #     return    
        # 
        vehicle = self._find_first_vehicle_with_syncpoint()  
        
        behavior = py_trees.composites.Sequence(name="Ego vehicle behavior")

        drive_conditions = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        
        # drive_condition1 = TargetsNotInProximityViewOfActor(
        #     actor=self.ego_vehicles[0],
        #     targets=[TargetProximityInfo(vehicle,vehicle.awareness_distance)],
        #     success_on_all=True,
        #     view_angle=0.0,
        #     name="egodrive")
        # drive_conditions.add_child(drive_condition1)
        
        # ego_drive_condition = TargetsInProximityViewOfActor(
        #     actor=self.ego_vehicles[0],
        #     targets=[TargetProximityInfo(vehicle,vehicle.sync_point.radius*0.5)],
        #     success_on_all=True,
        #     name="Drive condition")
        drive_condition = InTriggerDistanceToLocationOneShot(
            vehicle,
            vehicle.path[-1].location,
            5,
            #vehicle.destination_radius,
            name='egodrive')        
        # drive_conditions.add_child(drive_condition2)

        # ego_stop_condition = TargetsInProximityViewOfActor(
        #     actor=self.ego_vehicles[0],
        #     targets=[TargetProximityInfo(vehicle,vehicle.awareness_distance)],
        #     name="Stop condition")
        stop_condition = InTriggerDistanceToLocationOneShot(
            self.ego_vehicles[0],
            vehicle.sync_point.transform.location,
            vehicle.sync_point.radius,
            name='ego Stop condition')
        
        # behavior_parallel = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        stop_condition_seq = py_trees.composites.Sequence(name="Ego stop condition")
        stop_condition_seq.add_child(stop_condition)

        stop_behavior = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        stop_params = {}
        stop_params["max_speed"] = 0
        stop_params["ignore_lights_percentage"] = 100
        stop_params["ignore_signs_percentage"] = 100
        stop = ChangeAutoPilot(actor=self.ego_vehicles[0], activate=True, parameters=stop_params, name="Stop")
        stop_behavior.add_child(stop)
        stop_behavior.add_child(drive_condition)

        drive_behavior = py_trees.composites.Sequence(name="Ego drive behavior")
        drive_params = {}
        drive_params["max_speed"] = self.ego_vehicles[0].get_speed_limit()
        drive_params["ignore_lights_percentage"] = 100
        drive_params["ignore_signs_percentage"] = 100
        drive = ChangeAutoPilot(actor=self.ego_vehicles[0], activate=True, parameters=drive_params, name="Drive")
        drive_behavior.add_child(drive)

        behavior.add_child(stop_condition)
        behavior.add_child(stop_behavior)
        behavior.add_child(drive_behavior)
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
        start_behavior.add_child(ChangeAutoPilot(actor=self.ego_vehicles[0], activate=True, parameters=speed_params, name="Start"))
        start_behavior.add_child(stop_conditions)

        stop_behavior = py_trees.composites.Sequence()
        # stop_behavior = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        stop_params = {}
        stop_params["max_speed"] = 0
        stop_params["ignore_lights_percentage"] = 100
        stop_params["ignore_signs_percentage"] = 100
        stop = ChangeAutoPilot(actor=self.ego_vehicles[0], activate=True, parameters=stop_params, name="Stop")
        stop_behavior.add_child(stop)
        stop_behavior.add_child(drive_conditions)

        drive_behavior = py_trees.composites.Sequence()
        drive = ChangeAutoPilot(actor=self.ego_vehicles[0], activate=True, parameters=speed_params, name="Drive")
        drive_behavior.add_child(drive)

        # behavior.add_child(ChangeAutoPilot(actor=self.ego_vehicles[0], activate=True, parameters=start_params, name="Start"))
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

    def _create_behaviorOLD(self):
        """
        After invoking this scenario, it will wait for the user
        controlled vehicle to enter the start region,
        then make a traffic participant to accelerate
        until it is going fast enough to reach an intersection point.
        at the same time as the user controlled vehicle at the junction.
        Once the user controlled vehicle comes close to the junction,
        the traffic participant accelerates and passes through the junction.
        After 60 seconds, a timeout stops the scenario.
        """

        # Creating leaf nodes
        start_other_trigger = InTriggerRegion(
            self.ego_vehicles[0],
            -80, -70,
            -75, -60)

        sync_arrival = SyncArrival(
            self.other_actors[0], self.ego_vehicles[0],
            carla.Location(x=-74.63, y=-136.34))

        pass_through_trigger = InTriggerRegion(
            self.ego_vehicles[0],
            -90, -70,
            -124, -119)

        keep_velocity_other = KeepVelocity(
            self.other_actors[0],
            self._other_actor_target_velocity)

        stop_other_trigger = InTriggerRegion(
            self.other_actors[0],
            -45, -35,
            -140, -130)

        stop_other = StopVehicle(
            self.other_actors[0],
            self._other_actor_max_brake)

        end_condition = InTriggerRegion(
            self.ego_vehicles[0],
            -90, -70,
            -170, -156
        )

        # Creating non-leaf nodes
        root = py_trees.composites.Sequence()
        scenario_sequence = py_trees.composites.Sequence()
        sync_arrival_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_other_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # Building tree
        root.add_child(scenario_sequence)
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))
        scenario_sequence.add_child(start_other_trigger)
        scenario_sequence.add_child(sync_arrival_parallel)
        scenario_sequence.add_child(keep_velocity_other_parallel)
        scenario_sequence.add_child(stop_other)
        scenario_sequence.add_child(end_condition)
        scenario_sequence.add_child(ActorDestroy(self.other_actors[0]))

        sync_arrival_parallel.add_child(sync_arrival)
        sync_arrival_parallel.add_child(pass_through_trigger)
        keep_velocity_other_parallel.add_child(keep_velocity_other)
        keep_velocity_other_parallel.add_child(stop_other_trigger)

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
