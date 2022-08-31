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

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      AccelerateToVelocity,
                                                                      HandBrakeVehicle,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocationAlongRoute,
                                                                               InTimeToArrivalToVehicle,
                                                                               InTimeToArrivalToLocation,
                                                                               InTriggerDistanceToLocation,
                                                                               InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill)




from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_location_in_distance_from_wp
from srunner.tools.scenario_helper import get_waypoint_in_distance


def is_lane_a_parking(waypoint):
    """
    This function filters false negative Shoulder which are in reality Parking lanes.
    These are differentiated from the others because, similar to the driving lanes,
    they have, on the right, a small Shoulder followed by a Sidewalk.
    """

    # Parking are wide lanes
    if waypoint.lane_width > 2:
        wp_next = waypoint.get_right_lane()

        # That are next to a mini-Shoulder
        if wp_next is not None and wp_next.lane_type == carla.LaneType.Shoulder:
            wp_next_next = wp_next.get_right_lane()

            # Followed by a Sidewalk
            if wp_next_next is not None and wp_next_next.lane_type == carla.LaneType.Sidewalk:
                return True

    return False


def get_transform(waypoint, offset_yaw=-90, offset_z=0.2, offset_right=0.0, offset_forward=0.0):
    """
    Processes the waypoint transform to find a suitable spawning one at the sidewalk.
    It first rotates the transform so that it is pointing towards the road and then moves a
    bit to the side waypoint that aren't part of sidewalks, as they might be invading the road
    """
    new_rotation = waypoint.transform.rotation
    new_rotation.yaw += offset_yaw

    forward_vector = waypoint.transform.get_forward_vector()
    forward_offset = carla.Location(offset_forward * forward_vector.x, offset_forward * forward_vector.y)

    right_vector = waypoint.transform.get_right_vector()
    offset_location = carla.Location(offset_right * right_vector.x, offset_right * right_vector.y)

    new_location = waypoint.transform.location + offset_location + forward_offset
    new_location.z += offset_z

    print('offset_location:', offset_location)
    print('forward_offset:',forward_offset)
    print('old location:',waypoint.transform.location)
    print('new location:',new_location)

    return carla.Transform(new_location, new_rotation)


def get_sidewalk_transform(waypoint, offset_yaw=-90, offset_z=0.2, offset_dist=1.5):
    """
    Processes the waypoint transform to find a suitable spawning one at the sidewalk.
    It first rotates the transform so that it is pointing towards the road and then moves a
    bit to the side waypoint that aren't part of sidewalks, as they might be invading the road
    """
    new_rotation = waypoint.transform.rotation
    new_rotation.yaw += offset_yaw

    if waypoint.lane_type == carla.LaneType.Sidewalk:
        new_location = waypoint.transform.location
    else:
        right_vector = waypoint.transform.get_right_vector()
        offset_location = carla.Location(offset_dist * right_vector.x, offset_dist * right_vector.y)
        new_location = waypoint.transform.location + offset_location
    new_location.z += offset_z

    return carla.Transform(new_location, new_rotation)

def get_opposite_sidewalk_transform(waypoint, offset_yaw=90, offset_z=0.2, offset_dist=2.5):
    """
    Processes the waypoint transform to find a suitable spawning one at the sidewalk.
    It first rotates the transform so that it is pointing towards the road and then moves a
    bit to the side waypoint that aren't part of sidewalks, as they might be invading the road
    """
    new_rotation = waypoint.transform.rotation
    new_rotation.yaw += offset_yaw

    # if waypoint.lane_type == carla.LaneType.Sidewalk:
    #     new_location = waypoint.transform.location
    # else:
    right_vector = waypoint.transform.get_right_vector()
    offset_location = carla.Location(offset_dist * right_vector.x, offset_dist * right_vector.y)
    new_location = waypoint.transform.location + offset_location
    new_location.z += offset_z

    return carla.Transform(new_location, new_rotation)

def get_number_of_lanes(waypoint):
    """
    Returns the number of driving lanes
    Gets the driving / parking lane that is most to the right of the waypoint
    as well as the number of lane changes done
    """
    lane_changes = 0

    wp = waypoint

    # count lane changes to the right of the waypoint
    while True:
        wp_next = wp.get_right_lane()
        lane_changes += 1

        if wp_next is None or wp_next.lane_type == carla.LaneType.Sidewalk:
            break
        elif wp_next.lane_type == carla.LaneType.Shoulder:
            # Filter Parkings considered as Shoulders
            if is_lane_a_parking(wp_next):
                lane_changes += 1
            break
        wp = wp_next

    wp = waypoint
    # count lane changes to the left of the waypoint
    while True:
        wp_next = wp.get_left_lane()
        if wp_next == wp:
            break
        lane_changes += 1

        if wp_next is None or wp_next.lane_type == carla.LaneType.Sidewalk:
            break
        elif wp_next.lane_type == carla.LaneType.Shoulder:
            # Filter Parkings considered as Shoulders
            if is_lane_a_parking(wp_next):
                lane_changes += 1
            break
        print('lane_changes:', lane_changes, wp_next, wp_next.lane_type)
        wp = wp_next
    return lane_changes



class CyclistsCrossing(BasicScenario):
    """
    This class holds everything required for a cyclist crash
    without prior vehicle action involving a vehicle and a cyclist/pedestrian,
    The ego vehicle is passing through a road,
    And encounters a cyclist/pedestrian crossing the road.

    This is a single ego vehicle scenario
    """
    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()

        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40

        self.num_cyclists = 2
        self.cyclists = []

        for i in range(self.num_cyclists):
            cyclist = {}
            cyclist.velocity = 5
            cyclist.brake = 1.0
            cyclist.transform = None
            self.cyclists.append( cyclist )


        # other vehicle parameters
        self._other_actor_target_velocity = 5
        self._other_actor_max_brake = 1.0
        self._time_to_reach = 10
        self._walker_yaw = 0
        self._num_lane_changes = 1
        self.transform = None
        self.transform2 = None
        self.timeout = timeout
        self._trigger_location = config.trigger_points[0].location
        # Total Number of attempts to relocate a vehicle before spawning
        self._number_of_attempts = 20
        # Number of attempts made so far
        self._spawn_attempted = 0

        self._ego_route = CarlaDataProvider.get_ego_vehicle_route()

        super(CyclistsCrossing, self).__init__("CyclistsCrossing",
                                                    ego_vehicles,
                                                    config,
                                                    world,
                                                    debug_mode,
                                                    criteria_enable=criteria_enable)

    def _calculate_base_transform(self, _start_distance, waypoint):

        lane_width = waypoint.lane_width

        # Patches false junctions
        if self._reference_waypoint.is_junction:
            stop_at_junction = False
        else:
            stop_at_junction = True

        print('stop_at_junction: ', stop_at_junction, '_start_distance:',_start_distance)
        location, _ = get_location_in_distance_from_wp(waypoint, _start_distance, stop_at_junction)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 270, "position": 90, "z": 0.6, "k": 1.0}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        location.z = self._trigger_location.z + offset['z']
        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)), orientation_yaw

    def _spawn_adversary(self, transform, orientation_yaw):
        self._time_to_reach *= self._num_lane_changes
        self._other_actor_target_velocity = self._other_actor_target_velocity * self._num_lane_changes
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.diamondback.century', transform)
        first_vehicle.set_simulate_physics(enabled=False)
        if isinstance(first_vehicle, carla.Vehicle):
            first_vehicle.set_simulate_physics(enabled=True)
            #first_vehicle.apply_control(carla.VehicleControl(hand_brake=True))
        adversary = first_vehicle
        return adversary

    def _spawn_blocker(self, transform, orientation_yaw):
        """
        Spawn the blocker prop that blocks the vision from the egovehicle of the jaywalker
        :return:
        """
        # static object transform
        shift = 0.9
        x_ego = self._reference_waypoint.transform.location.x
        y_ego = self._reference_waypoint.transform.location.y
        x_cycle = transform.location.x
        y_cycle = transform.location.y
        x_static = x_ego + shift * (x_cycle - x_ego)
        y_static = y_ego + shift * (y_cycle - y_ego)

        spawn_point_wp = self.ego_vehicles[0].get_world().get_map().get_waypoint(transform.location)

        self.transform2 = carla.Transform(carla.Location(x_static, y_static,
                                                         spawn_point_wp.transform.location.z + 0.3),
                                          carla.Rotation(yaw=orientation_yaw + 180))

        static = CarlaDataProvider.request_new_actor('static.prop.vendingmachine', self.transform2)
        static.set_simulate_physics(enabled=False)

        return static

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # cyclist transform
        _start_distance = 25
        # We start by getting and waypoint in the closest sidewalk.
        waypoint = self._reference_waypoint
        while True:
            wp_next = waypoint.get_right_lane()
            self._num_lane_changes += 1
            if wp_next is None or wp_next.lane_type == carla.LaneType.Sidewalk:
                print('<----fg> Sidewalk')
                break
            elif wp_next.lane_type == carla.LaneType.Shoulder:
                print('<----fg> Shoulder')
                # Filter Parkings considered as Shoulders
                if wp_next.lane_width > 2:
                    _start_distance += 1.5
                    waypoint = wp_next
                break
            else:
                _start_distance += 1.5
                waypoint = wp_next

        print('_start_distance:', _start_distance)
        while True:  # We keep trying to spawn avoiding props

            try:
                self.transform, orientation_yaw = self._calculate_base_transform(_start_distance, waypoint)
                first_vehicle = self._spawn_adversary(self.transform, orientation_yaw)
                blocker = self._spawn_blocker(self.transform, orientation_yaw)
                break
            except RuntimeError as r:
                # We keep retrying until we spawn
                print("Base transform is blocking objects ", self.transform)
                _start_distance += 0.4
                self._spawn_attempted += 1
                if self._spawn_attempted >= self._number_of_attempts:
                    raise r

        # Now that we found a possible position we just put the vehicle to the underground
        disp_transform = carla.Transform(
            carla.Location(self.transform.location.x,
                           self.transform.location.y,
                           self.transform.location.z),
            self.transform.rotation)

        prop_disp_transform = carla.Transform(
            carla.Location(self.transform2.location.x,
                           self.transform2.location.y,
                           self.transform2.location.z),
            self.transform2.rotation)

        first_vehicle.set_transform(disp_transform)
        blocker.set_transform(prop_disp_transform)
        # if not isinstance(first_vehicle, carla.Vehicle):
        #     first_vehicle.set_simulate_physics(enabled=False)
        blocker.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)
        self.other_actors.append(blocker)

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="OccludedObjectCrossing")
        lane_width = self._reference_waypoint.lane_width
        lane_width = lane_width + (1.25 * lane_width * self._num_lane_changes)

        dist_to_trigger = 12 + self._num_lane_changes

        actor_velocity = KeepVelocity(self.other_actors[0],
                                      self._other_actor_target_velocity,
                                      name="walker velocity")
        actor_drive = DriveDistance(self.other_actors[0],
                                    0.5 * lane_width,
                                    name="walker drive distance")
        actor_start_cross_lane = AccelerateToVelocity(self.other_actors[0],
                                                      1.0,
                                                      self._other_actor_target_velocity,
                                                      name="walker crossing lane accelerate velocity")
        actor_cross_lane = DriveDistance(self.other_actors[0],
                                         lane_width,
                                         name="walker drive distance for lane crossing ")
        actor_stop_crossed_lane = StopVehicle(self.other_actors[0],
                                              self._other_actor_max_brake,
                                              name="walker stop")
        ego_pass_machine = DriveDistance(self.ego_vehicles[0],
                                         5,
                                         name="ego vehicle passed prop")
        actor_remove = ActorDestroy(self.other_actors[0],
                                    name="Destroying walker")
        static_remove = ActorDestroy(self.other_actors[1],
                                     name="Destroying Prop")
        end_condition = DriveDistance(self.ego_vehicles[0],
                                      self._ego_vehicle_distance_driven,
                                      name="End condition ego drive distance")

        # non leaf nodes

        scenario_sequence = py_trees.composites.Sequence()
        keep_velocity_other = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity other")
        keep_velocity = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity")

        # building tree

        root.add_child(scenario_sequence)
        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[0], False))
        scenario_sequence.add_child(keep_velocity)
        scenario_sequence.add_child(keep_velocity_other)
        scenario_sequence.add_child(actor_stop_crossed_lane)
        scenario_sequence.add_child(actor_remove)
        scenario_sequence.add_child(static_remove)
        scenario_sequence.add_child(end_condition)

        keep_velocity.add_child(actor_velocity)
        keep_velocity.add_child(actor_drive)
        keep_velocity_other.add_child(actor_start_cross_lane)
        keep_velocity_other.add_child(actor_cross_lane)
        keep_velocity_other.add_child(ego_pass_machine)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class CyclistCrossing(BasicScenario):
    """
    This class holds everything required for a cyclist crash
    without prior vehicle action involving a vehicle and a cyclist/pedestrian,
    The ego vehicle is passing through a road,
    And encounters a cyclist/pedestrian crossing the road.

    This is a single ego vehicle scenario
    """
    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()

        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40
        # other vehicle parameters
        self._other_actor_target_velocity = 5
        self._other_actor_max_brake = 1.0
        self._time_to_reach = 10
        self._walker_yaw = 0
        self._num_lane_changes = 1
        self.transform = None
        self.transform2 = None
        self.timeout = timeout
        self._trigger_location = config.trigger_points[0].location
        # Total Number of attempts to relocate a vehicle before spawning
        self._number_of_attempts = 20
        # Number of attempts made so far
        self._spawn_attempted = 0

        self._ego_route = CarlaDataProvider.get_ego_vehicle_route()

        super(CyclistCrossing, self).__init__("CyclistCrossing",
                                                    ego_vehicles,
                                                    config,
                                                    world,
                                                    debug_mode,
                                                    criteria_enable=criteria_enable)

    def _calculate_base_transform(self, _start_distance, waypoint):

        lane_width = waypoint.lane_width

        # Patches false junctions
        if self._reference_waypoint.is_junction:
            stop_at_junction = False
        else:
            stop_at_junction = True

        print('stop_at_junction: ', stop_at_junction, '_start_distance:',_start_distance)
        location, _ = get_location_in_distance_from_wp(waypoint, _start_distance, stop_at_junction)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 270, "position": 90, "z": 0.6, "k": 1.0}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        location.z = self._trigger_location.z + offset['z']
        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)), orientation_yaw

    def _spawn_adversary(self, transform, orientation_yaw):
        self._time_to_reach *= self._num_lane_changes
        self._other_actor_target_velocity = self._other_actor_target_velocity * self._num_lane_changes
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.diamondback.century', transform)
        first_vehicle.set_simulate_physics(enabled=False)
        if isinstance(first_vehicle, carla.Vehicle):
            first_vehicle.set_simulate_physics(enabled=True)
            #first_vehicle.apply_control(carla.VehicleControl(hand_brake=True))
        adversary = first_vehicle
        return adversary

    def _spawn_blocker(self, transform, orientation_yaw):
        """
        Spawn the blocker prop that blocks the vision from the egovehicle of the jaywalker
        :return:
        """
        # static object transform
        shift = 0.9
        x_ego = self._reference_waypoint.transform.location.x
        y_ego = self._reference_waypoint.transform.location.y
        x_cycle = transform.location.x
        y_cycle = transform.location.y
        x_static = x_ego + shift * (x_cycle - x_ego)
        y_static = y_ego + shift * (y_cycle - y_ego)

        spawn_point_wp = self.ego_vehicles[0].get_world().get_map().get_waypoint(transform.location)

        self.transform2 = carla.Transform(carla.Location(x_static, y_static,
                                                         spawn_point_wp.transform.location.z + 0.3),
                                          carla.Rotation(yaw=orientation_yaw + 180))

        static = CarlaDataProvider.request_new_actor('static.prop.vendingmachine', self.transform2)
        static.set_simulate_physics(enabled=False)

        return static

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # cyclist transform
        _start_distance = 25
        # We start by getting and waypoint in the closest sidewalk.
        waypoint = self._reference_waypoint
        while True:
            wp_next = waypoint.get_right_lane()
            self._num_lane_changes += 1
            if wp_next is None or wp_next.lane_type == carla.LaneType.Sidewalk:
                print('<----fg> Sidewalk')
                break
            elif wp_next.lane_type == carla.LaneType.Shoulder:
                print('<----fg> Shoulder')
                # Filter Parkings considered as Shoulders
                if wp_next.lane_width > 2:
                    _start_distance += 1.5
                    waypoint = wp_next
                break
            else:
                _start_distance += 1.5
                waypoint = wp_next

        print('_start_distance:', _start_distance)
        while True:  # We keep trying to spawn avoiding props

            try:
                self.transform, orientation_yaw = self._calculate_base_transform(_start_distance, waypoint)
                first_vehicle = self._spawn_adversary(self.transform, orientation_yaw)
                blocker = self._spawn_blocker(self.transform, orientation_yaw)
                break
            except RuntimeError as r:
                # We keep retrying until we spawn
                print("Base transform is blocking objects ", self.transform)
                _start_distance += 0.4
                self._spawn_attempted += 1
                if self._spawn_attempted >= self._number_of_attempts:
                    raise r

        # Now that we found a possible position we just put the vehicle to the underground
        disp_transform = carla.Transform(
            carla.Location(self.transform.location.x,
                           self.transform.location.y,
                           self.transform.location.z),
            self.transform.rotation)

        prop_disp_transform = carla.Transform(
            carla.Location(self.transform2.location.x,
                           self.transform2.location.y,
                           self.transform2.location.z),
            self.transform2.rotation)

        first_vehicle.set_transform(disp_transform)
        blocker.set_transform(prop_disp_transform)
        if not isinstance(first_vehicle, carla.Vehicle):
            first_vehicle.set_simulate_physics(enabled=False)
        
        blocker.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)
        self.other_actors.append(blocker)

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="OccludedObjectCrossing")
        lane_width = self._reference_waypoint.lane_width
        lane_width = lane_width + (1.25 * lane_width * self._num_lane_changes)

        dist_to_trigger = 12 + self._num_lane_changes

        actor_velocity = KeepVelocity(self.other_actors[0],
                                      self._other_actor_target_velocity,
                                      name="walker velocity")
        actor_drive = DriveDistance(self.other_actors[0],
                                    0.5 * lane_width,
                                    name="walker drive distance")
        actor_start_cross_lane = AccelerateToVelocity(self.other_actors[0],
                                                      1.0,
                                                      self._other_actor_target_velocity,
                                                      name="walker crossing lane accelerate velocity")
        actor_cross_lane = DriveDistance(self.other_actors[0],
                                         lane_width,
                                         name="walker drive distance for lane crossing ")
        actor_stop_crossed_lane = StopVehicle(self.other_actors[0],
                                              self._other_actor_max_brake,
                                              name="walker stop")
        ego_pass_machine = DriveDistance(self.ego_vehicles[0],
                                         5,
                                         name="ego vehicle passed prop")
        actor_remove = ActorDestroy(self.other_actors[0],
                                    name="Destroying walker")
        static_remove = ActorDestroy(self.other_actors[1],
                                     name="Destroying Prop")
        end_condition = DriveDistance(self.ego_vehicles[0],
                                      self._ego_vehicle_distance_driven,
                                      name="End condition ego drive distance")

        # non leaf nodes

        scenario_sequence = py_trees.composites.Sequence()
        keep_velocity_other = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity other")
        keep_velocity = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity")

        # building tree

        root.add_child(scenario_sequence)
        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[0], False))
        scenario_sequence.add_child(keep_velocity)
        scenario_sequence.add_child(keep_velocity_other)
        scenario_sequence.add_child(actor_stop_crossed_lane)
        scenario_sequence.add_child(actor_remove)
        scenario_sequence.add_child(static_remove)
        scenario_sequence.add_child(end_condition)

        keep_velocity.add_child(actor_velocity)
        keep_velocity.add_child(actor_drive)
        keep_velocity_other.add_child(actor_start_cross_lane)
        keep_velocity_other.add_child(actor_cross_lane)
        keep_velocity_other.add_child(ego_pass_machine)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class PedestrianCrossingOLD(BasicScenario):

    """
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a pedestrian,
    The ego vehicle is passing through a road,
    And encounters a pedestrian crossing the road.
    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config,
                 randomize=False, debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._wmap.get_waypoint(self._trigger_location)
        self._num_lane_changes = 0

        self._start_distance = 12  # how far from the trigger the adversary and blocker is spawn
        self._blocker_shift = 0.9
        self._retry_dist = 0.4

        self._adversary_type = 'walker.*'  # blueprint filter of the adversary
        self._blocker_type = 'static.prop.vendingmachine'  # blueprint filter of the blocker
        self._adversary_transform = None
        self._blocker_transform = None
        self._collision_wp = None

        self._adversary_speed = 4.0  # Speed of the adversary [m/s]
        self._reaction_time = 0.8  # Time the agent has to react to avoid the collision [s]
        self._reaction_ratio = 0.12  # The higehr the number of lane changes, the smaller the reaction time
        self._min_trigger_dist = 6.0  # Min distance to the collision location that triggers the adversary [m]
        self._ego_end_distance = 40
        self.timeout = timeout

        self._number_of_attempts = 6

        super(PedestrianCrossingOLD, self).__init__("PedestrianCrossingOLD",
                                                    ego_vehicles,
                                                    config,
                                                    world,
                                                    debug_mode,
                                                    criteria_enable=criteria_enable)

    def _get_sidewalk_transform(self, waypoint, offset):
        """
        Processes the waypoint transform to find a suitable spawning one at the sidewalk.
        It first rotates the transform so that it is pointing towards the road and then moves a
        bit to the side waypoint that aren't part of sidewalks, as they might be invading the road
        """

        new_rotation = waypoint.transform.rotation
        new_rotation.yaw += offset['yaw']

        if waypoint.lane_type == carla.LaneType.Sidewalk:
            new_location = waypoint.transform.location
        else:
            right_vector = waypoint.transform.get_right_vector()
            offset_dist = waypoint.lane_width * offset["k"]
            offset_location = carla.Location(offset_dist * right_vector.x, offset_dist * right_vector.y)
            new_location = waypoint.transform.location + offset_location
        new_location.z += offset['z']

        return carla.Transform(new_location, new_rotation)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # Get the waypoint in front of the ego.
        move_dist = self._start_distance
        waypoint = self._reference_waypoint
        while self._number_of_attempts > 0:
            self._num_lane_changes = 0

            # Move to the front
            location, _ = get_location_in_distance_from_wp(waypoint, move_dist, False)
            waypoint = self._wmap.get_waypoint(location)
            self._collision_wp = waypoint

            # Move to the right
            sidewalk_waypoint = waypoint
            while sidewalk_waypoint.lane_type != carla.LaneType.Sidewalk:
                right_wp = sidewalk_waypoint.get_right_lane()
                if right_wp is None:
                    break  # No more right lanes
                sidewalk_waypoint = right_wp
                self._num_lane_changes += 1

            # Get the adversary transform and spawn it
            offset = {"yaw": 270, "z": 0.5, "k": 1.0}
            self._adversary_transform = self._get_sidewalk_transform(sidewalk_waypoint, offset)
            adversary = CarlaDataProvider.request_new_actor(self._adversary_type, self._adversary_transform)
            if adversary is None:
                self._number_of_attempts -= 1
                move_dist = self._retry_dist
                continue

            # Get the blocker transform and spawn it
            blocker_wp = sidewalk_waypoint.previous(self._blocker_shift)[0]
            offset = {"yaw": 90, "z": 0.0, "k": 1.0}
            self._blocker_transform = self._get_sidewalk_transform(blocker_wp, offset)
            blocker = CarlaDataProvider.request_new_actor(self._blocker_type, self._blocker_transform)
            if not blocker:
                adversary.destroy()
                self._number_of_attempts -= 1
                move_dist = self._retry_dist
                continue

            # Both actors where summoned, end
            break

        if self._number_of_attempts == 0:
            raise Exception("Couldn't find viable position for the adversary and blocker actors")

        if isinstance(adversary, carla.Vehicle):
            adversary.apply_control(carla.VehicleControl(hand_brake=True))
        blocker.set_simulate_physics(enabled=False)
        self.other_actors.append(adversary)
        self.other_actors.append(blocker)

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """
        sequence = py_trees.composites.Sequence()
        collision_location = self._collision_wp.transform.location
        collision_distance = collision_location.distance(self._adversary_transform.location)
        collision_duration = collision_distance / self._adversary_speed
        reaction_time = self._reaction_time - self._reaction_ratio * self._num_lane_changes
        collision_time_trigger = collision_duration + reaction_time

        # Wait until ego is close to the adversary
        trigger_adversary = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="TriggerAdversaryStart")
        trigger_adversary.add_child(InTimeToArrivalToLocation(
            self.ego_vehicles[0], collision_time_trigger, collision_location))
        trigger_adversary.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], collision_location, self._min_trigger_dist))
        sequence.add_child(trigger_adversary)

        # Move the adversary
        speed_duration = 2.0 * collision_duration * self._num_lane_changes
        speed_distance = 2.0 * collision_distance * self._num_lane_changes
        sequence.add_child(KeepVelocity(
            self.other_actors[0], self._adversary_speed,
            duration=speed_duration, distance=speed_distance, name="AdversaryCrossing"))

        # Remove everything
        sequence.add_child(ActorDestroy(self.other_actors[0], name="DestroyAdversary"))
        sequence.add_child(ActorDestroy(self.other_actors[1], name="DestroyBlocker"))
        sequence.add_child(DriveDistance(self.ego_vehicles[0], self._ego_end_distance, name="EndCondition"))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()



class CrowdCrossing(BasicScenario):

    """
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a pedestrian,
    The ego vehicle is passing through a road,
    And encounters a pedestrian crossing the road.
    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config,
                 randomize=False, debug_mode=False, criteria_enable=True, timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._wmap.get_waypoint(self._trigger_location)
        self._num_lane_changes = 0


        # num_lanes = get_number_of_lanes(self._reference_waypoint)
        # print('num_lanes:', num_lanes)

        self._pedestrian_type = 'walker.*'  # blueprint filter of the pedestrian
        self._blocker_type = 'static.prop.vendingmachine'  # blueprint filter of the blocker
        # self._adversary_transform = None
        # self._blocker_transform = None
        self._collision_wp = None
        self.timeout = timeout
        self._number_of_attempts = 6
        self._pedestrians = []

        self._init_settings()


        super(CrowdCrossing, self).__init__("CrowdCrossing",
                                                    ego_vehicles,
                                                    config,
                                                    world,
                                                    debug_mode,
                                                    criteria_enable=criteria_enable)

    def _init_settings(self):
        self._start_distance = 15
        self._blocker_shift = 0.9
        self._actor_position_shift = 0.9
        self._retry_dist = 0.4

        self._pedestrian_speed = 4.0  # Speed of the pedestrian [m/s]
        self._reaction_time = 0.8     #0.8  # Time the agent has to react to avoid the collision [s]
        self._reaction_ratio = 0.12   # The higehr the number of lane changes, the smaller the reaction time
        self._min_trigger_dist = 16.0  # Min distance to the collision location that triggers the pedestrian [m]
        self._ego_end_distance = 40

        # pedestrian configurations
        self._pedestrian_configs = []
        self._pedestrian_configs.append( { "speed": self._pedestrian_speed*0.75 } )
        self._pedestrian_configs.append( { "speed": self._pedestrian_speed*0.45 } )
        self._pedestrian_configs.append( { "speed": self._pedestrian_speed*0.5 } )
        self._pedestrian_configs.append( { "speed": self._pedestrian_speed*1.0 } )


    def _get_spawn_waypoint( self, distance):

        # Move to the front
        location, _ = get_location_in_distance_from_wp(self._reference_waypoint, distance, False)
        waypoint = self._wmap.get_waypoint(location)

        # Move to the right
        sidewalk_waypoint = waypoint
        while sidewalk_waypoint.lane_type != carla.LaneType.Sidewalk:
            right_wp = sidewalk_waypoint.get_right_lane()
            if right_wp is None:
                break  # No more right lanes
            sidewalk_waypoint = right_wp
            self._num_lane_changes += 1

        return waypoint, sidewalk_waypoint

    def _initialize_actors(self, config):

        move_dist = self._start_distance

        while self._number_of_attempts > 0:
            self._collision_wp, spawn_waypoint = self._get_spawn_waypoint(move_dist)
            if self._initialize_pedestrians( spawn_waypoint, config ):
                break
            else:
                self._number_of_attempts -= 1
                move_dist = self._retry_dist

        if self._number_of_attempts == 0:
            raise Exception("Couldn't find viable position for the adversary and blocker actors")


    def _initialize_pedestrians(self, spawn_waypoint, config):
        # Get the pedestrian transform and spawn it
        #import pdb; pdb.set_trace()
        pedestrian_shift = self._actor_position_shift
        pedestrian_wp = spawn_waypoint

        for config in self._pedestrian_configs:
            waypoint = spawn_waypoint.next(pedestrian_shift)[0]
            transform = get_sidewalk_transform( waypoint, offset_yaw=270, offset_z=0.5, offset_dist=1.50 )
            actor = CarlaDataProvider.request_new_actor(self._pedestrian_type, transform)
            if actor:
                pedestrian = {}
                pedestrian[ 'transform' ] = transform
                pedestrian[ 'speed' ] = config['speed']
                pedestrian[ 'delay' ] = 0.0
                pedestrian[ 'actor' ] = actor
                self._pedestrians.append( pedestrian )        
                self.other_actors.append( actor )
                pedestrian_shift += self._actor_position_shift

        return True

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """        
        sequence = py_trees.composites.Sequence()


        # Trigger the crowsing based on the furthest pedestrian from the ego vehicle
        collision_location = self._collision_wp.transform.location
       
        # Wait until ego is close to the pedestrian
        trigger_crowd = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="TriggeCrowdCrossingStart")
        
        if len(self._pedestrians) > 0:
            collision_distance = collision_location.distance(self._pedestrians[0]['transform'].location)
            collision_duration = collision_distance / self._pedestrian_speed
            reaction_time = self._reaction_time - self._reaction_ratio * self._num_lane_changes
            collision_time_trigger = collision_duration + reaction_time
            trigger_crowd.add_child(InTimeToArrivalToLocation(
                self.ego_vehicles[0], collision_time_trigger, collision_location))

        trigger_crowd.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], collision_location, self._min_trigger_dist))
        sequence.add_child(trigger_crowd)



        crowd_crossing_street = py_trees.composites.Parallel(
        "CrowdCrossing",
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        for i, pedestrian in enumerate(self._pedestrians):

            # Move the pedestrian
            collision_distance = collision_location.distance(pedestrian['transform'].location)
            collision_duration = collision_distance / pedestrian['speed']
            speed_duration = 2.0 * collision_duration * self._num_lane_changes
            speed_distance = 2.0 * collision_distance * self._num_lane_changes

            print('speed_duration:', speed_duration, 'speed_distance:', speed_distance, 'pseed:',pedestrian['speed'], 'delay:',pedestrian['delay'])
            pedestrian_crossing = py_trees.composites.Sequence("Pedestrian" + str(i))
            pedestrian_crossing.add_child(TimeOut( pedestrian['delay'] ))
            pedestrian_crossing.add_child(KeepVelocity(
                pedestrian['actor'], 
                pedestrian['speed'],
                duration=speed_duration + pedestrian['delay'], 
                distance=speed_distance, 
                name="Pedestrian" + str(i) + "Crossing"))

            crowd_crossing_street.add_child( pedestrian_crossing )

        sequence.add_child(crowd_crossing_street)

        # Remove everything
        sequence.add_child(DriveDistance(self.ego_vehicles[0], self._ego_end_distance, name="EndCondition"))

        for i, actor in enumerate(self.other_actors):
            sequence.add_child(ActorDestroy(actor, name="DestroyActor" + str(i)))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class CrowdCrossingOppositeSidewalk(CrowdCrossing):

    """
    This class holds everything required for a construction scenario
    The ego vehicle is passing through a road and encounters
    a stationary rectangular construction cones setup and traffic warning.

    This is a single ego vehicle scenario
    """

    def __init__(
            self, 
            world, 
            ego_vehicles, 
            config,
            randomize=False, 
            debug_mode=False, 
            criteria_enable=True, 
            timeout=60):
        """
        Setup all relevant parameters and create scenario
        """

        # # self._pedestrian_speed = 4.0  # Speed of the pedestrian [m/s]
        # self._reaction_time = 2.8     #0.8  # Time the agent has to react to avoid the collision [s]
        # self._reaction_ratio = 0.12   # The higehr the number of lane changes, the smaller the reaction time
        # self._min_trigger_dist = 16.0  # Min distance to the collision location that triggers the pedestrian [m]


        super(
            CrowdCrossingOppositeSidewalk,
            self).__init__(
            world,
            ego_vehicles=ego_vehicles,
            config=config,
            debug_mode=debug_mode,
            criteria_enable=criteria_enable,
            timeout=timeout)

    def _init_settings(self):
        CrowdCrossing._init_settings( self )
        self._start_distance = 25
        self._reaction_time = 1.8     #0.8  # Time the agent has to react to avoid the collision [s]



    def _initialize_pedestrians(self, spawn_waypoint, config):
        # Get the pedestrian transform and spawn it
        #import pdb; pdb.set_trace()
        pedestrian_shift = self._actor_position_shift
        pedestrian_wp = spawn_waypoint

        self._pedestrians = []

        for config in self._pedestrian_configs:
            waypoint = spawn_waypoint.next(pedestrian_shift)[0]
            transform = get_opposite_sidewalk_transform( waypoint, offset_yaw=90, offset_z=0.5, offset_dist=-13 + 2*random.random() )
            actor = CarlaDataProvider.request_new_actor(self._pedestrian_type, transform)
            if actor:
                pedestrian = {}
                pedestrian[ 'transform' ] = transform
                pedestrian[ 'speed' ] = config['speed']
                pedestrian[ 'delay' ] = 0.0
                pedestrian[ 'actor' ] = actor
                self._pedestrians.append( pedestrian )        
                self.other_actors.append( actor )
                pedestrian_shift += self._actor_position_shift

        return True

class CrowdCrossingNoBlocker(CrowdCrossing):

    """
    This class holds everything required for a construction scenario
    The ego vehicle is passing through a road and encounters
    a stationary rectangular construction cones setup and traffic warning.

    This is a single ego vehicle scenario
    """

    def __init__(
            self, 
            world, 
            ego_vehicles, 
            config,
            randomize=False, 
            debug_mode=False, 
            criteria_enable=True, 
            timeout=60):
        """
        Setup all relevant parameters and create scenario
        """

        super(
            CrowdCrossingNoBlocker,
            self).__init__(
            world,
            ego_vehicles=ego_vehicles,
            config=config,
            debug_mode=debug_mode,
            criteria_enable=criteria_enable,
            timeout=timeout)

    def _initialize_blocker(self, spawn_waypoint, config):
        pass


class CrowdCrossingOppositeSidewalkNoBlocker(CrowdCrossingOppositeSidewalk):

    """
    This class holds everything required for a construction scenario
    The ego vehicle is passing through a road and encounters
    a stationary rectangular construction cones setup and traffic warning.

    This is a single ego vehicle scenario
    """

    def __init__(
            self, 
            world, 
            ego_vehicles, 
            config,
            randomize=False, 
            debug_mode=False, 
            criteria_enable=True, 
            timeout=60):
        """
        Setup all relevant parameters and create scenario
        """

        super(
            CrowdCrossingOppositeSidewalkNoBlocker,
            self).__init__(
            world,
            ego_vehicles=ego_vehicles,
            config=config,
            debug_mode=debug_mode,
            criteria_enable=criteria_enable,
            timeout=timeout)

    def _initialize_blocker(self, spawn_waypoint, config):
        pass


class PedestrianCrossing(CrowdCrossing):

    """
    This class holds everything required for a construction scenario
    The ego vehicle is passing through a road and encounters
    a stationary rectangular construction cones setup and traffic warning.

    This is a single ego vehicle scenario
    """


    def _init_settings(self):
        CrowdCrossing._init_settings( self )
        
        self._start_distance = 12

        self._pedestrian_speed = 4.0  # Speed of the pedestrian [m/s]
        self._reaction_time = 0.8     #0.8  # Time the agent has to react to avoid the collision [s]
        self._reaction_ratio = 0.12   # The higehr the number of lane changes, the smaller the reaction time
        self._min_trigger_dist = 16.0  # Min distance to the collision location that triggers the pedestrian [m]
        self._ego_end_distance = 40

        # pedestrian configurations
        self._pedestrian_configs = []
        self._pedestrian_configs.append( { "speed": self._pedestrian_speed*1.0 } )


class PedestrianCrossingOppositeSidewalk(CrowdCrossingOppositeSidewalk):

    """
    This class holds everything required for a construction scenario
    The ego vehicle is passing through a road and encounters
    a stationary rectangular construction cones setup and traffic warning.

    This is a single ego vehicle scenario
    """

    def _init_settings(self):
        CrowdCrossing._init_settings( self )
        
        self._start_distance = 25

        # self._pedestrian_speed = 4.0  # Speed of the pedestrian [m/s]
        # self._reaction_time = 0.8     #0.8  # Time the agent has to react to avoid the collision [s]
        # self._reaction_ratio = 0.12   # The higehr the number of lane changes, the smaller the reaction time
        # self._min_trigger_dist = 16.0  # Min distance to the collision location that triggers the pedestrian [m]
        # self._ego_end_distance = 40

        # pedestrian configurations
        self._pedestrian_configs = []
        self._pedestrian_configs.append( { "speed": self._pedestrian_speed*1.0 } )



class PedestrianCrossingNoBlocker(PedestrianCrossing):

    """
    This class holds everything required for a construction scenario
    The ego vehicle is passing through a road and encounters
    a stationary rectangular construction cones setup and traffic warning.

    This is a single ego vehicle scenario
    """

    def __init__(
            self, 
            world, 
            ego_vehicles, 
            config,
            randomize=False, 
            debug_mode=False, 
            criteria_enable=True, 
            timeout=60):
        """
        Setup all relevant parameters and create scenario
        """

        super(
            PedestrianCrossingNoBlocker,
            self).__init__(
            world,
            ego_vehicles=ego_vehicles,
            config=config,
            debug_mode=debug_mode,
            criteria_enable=criteria_enable,
            timeout=timeout)

    def _initialize_blocker(self, spawn_waypoint, config):
        pass


"""
   Setup Requirements:
   - Atleast 94 cm between the start and end of the route
   - Atleast 66 cm clearance infront of the scenario trigger - no junctions etc
   - Leading vehicle starts 25 cm infront of scenario trigger
   - Cyclist starts 41 cm infront of the leading vehicle
   - Rould shoulders should be clear for cyclist to drive off the road
   
"""
class FollowLeadingVehicleWithObstruction(BasicScenario):

    """
    This class holds a scenario similar to FollowLeadingVehicle
    but there is an obstacle in front of the leading vehicle

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=360):
        """
        Setup all relevant parameters and create scenario
        """
        self._map = CarlaDataProvider.get_map()


        self._init_settings()

        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._first_actor_transform = None
        self._second_actor_transform = None

        super(FollowLeadingVehicleWithObstruction, self).__init__("FollowLeadingVehicleWithObstruction",
                                                               ego_vehicles,
                                                               config,
                                                               world,
                                                               debug_mode,
                                                               criteria_enable=criteria_enable)
        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

    def _init_settings(self):
        self._first_actor_location = 35 #25
        self._second_actor_location = self._first_actor_location + 41
        self._first_actor_speed = 10
        self._second_actor_speed = 1.5

        
    def _get_sidewalk_transform(self, waypoint, offset):
        """
        Processes the waypoint transform to find a suitable spawning one at the sidewalk.
        It first rotates the transform so that it is pointing towards the road and then moves a
        bit to the side waypoint that aren't part of sidewalks, as they might be invading the road
        """

        new_rotation = waypoint.transform.rotation
        new_rotation.yaw += offset['yaw']

        if waypoint.lane_type == carla.LaneType.Sidewalk:
            new_location = waypoint.transform.location
        else:
            right_vector = waypoint.transform.get_right_vector()
            offset_dist = waypoint.lane_width * offset["k"]
            offset_location = carla.Location(offset_dist * right_vector.x, offset_dist * right_vector.y)
            new_location = waypoint.transform.location + offset_location
        new_location.z += offset['z']

        return carla.Transform(new_location, new_rotation)

    def _spawn_actor(self, type, transform, enable_physics=True):
        actor = CarlaDataProvider.request_new_actor(type, transform)
        actor.set_simulate_physics(enabled=enable_physics)
        self.other_actors.append(actor)
        return actor

    def _initialize_leading_vehicle(self, config):
        location, _ = get_location_in_distance_from_wp(self._reference_waypoint, self._first_actor_location, False)
        waypoint = self._map.get_waypoint(location)
        actor_waypoint, _t1 = get_waypoint_in_distance(self._reference_waypoint, self._first_actor_location)
        actor_transform = carla.Transform(
            carla.Location(waypoint.transform.location.x,
                           waypoint.transform.location.y,
                           waypoint.transform.location.z),
            actor_waypoint.transform.rotation)
        self._spawn_actor('vehicle.nissan.patrol', actor_transform)

    def _initialize_obstructors(self, config):
        location, _ = get_location_in_distance_from_wp(self._reference_waypoint, self._second_actor_location, False)
        waypoint = self._map.get_waypoint(location)
        actor_waypoint, _t2 = get_waypoint_in_distance(self._reference_waypoint, self._second_actor_location)
        offset = {"yaw": 90, "z": 0.5, "k": -2}
        actor_transform = self._get_sidewalk_transform(actor_waypoint, offset)
        self._spawn_actor('vehicle.diamondback.century', actor_transform)

    def _initialize_actors(self, config):
        self._initialize_leading_vehicle( config )
        self._initialize_obstructors( config )

    def _initialize_actorsOLD(self, config):
        """
        Custom initialization
        """

        location, _ = get_location_in_distance_from_wp(self._reference_waypoint, self._first_actor_location, False)
        f1_waypoint = self._map.get_waypoint(location)

        
        location, _ = get_location_in_distance_from_wp(self._reference_waypoint, self._second_actor_location, False)
        f2_waypoint = self._map.get_waypoint(location)

        # import pdb; pdb.set_trace()

        first_actor_waypoint, _t1 = get_waypoint_in_distance(self._reference_waypoint, self._first_actor_location)
        second_actor_waypoint, _t2 = get_waypoint_in_distance(self._reference_waypoint, self._second_actor_location)

        # import pdb; pdb.set_trace()
        first_actor_transform = carla.Transform(
            carla.Location(first_actor_waypoint.transform.location.x,
                           first_actor_waypoint.transform.location.y,
                           first_actor_waypoint.transform.location.z),
            first_actor_waypoint.transform.rotation)



        offset = {"yaw": 90, "z": 0.5, "k": -2}
        second_actor_transform = self._get_sidewalk_transform(second_actor_waypoint, offset)
        
        # yaw_1 = second_actor_waypoint.transform.rotation.yaw + 90
        # second_actor_transform = carla.Transform(
        #     carla.Location(second_actor_waypoint.transform.location.x,
        #                    second_actor_waypoint.transform.location.y,
        #                    second_actor_waypoint.transform.location.z),
        #     carla.Rotation(second_actor_waypoint.transform.rotation.pitch, yaw_1,
        #                    second_actor_waypoint.transform.rotation.roll))

        first_actor = CarlaDataProvider.request_new_actor(
            'vehicle.nissan.patrol', first_actor_transform)

        second_actor = CarlaDataProvider.request_new_actor(
            'vehicle.diamondback.century', second_actor_transform)

        first_actor.set_simulate_physics(enabled=True)
        second_actor.set_simulate_physics(enabled=True)
        self.other_actors.append(first_actor)
        self.other_actors.append(second_actor)

        fd1 = self._reference_waypoint.transform.location.distance(first_actor_waypoint.transform.location)
        fd2 = first_actor_waypoint.transform.location.distance(second_actor_waypoint.transform.location)


    def _create_behavior(self):
        # on trigger
        # - parallel:
        #   - obstacle begin entering road
        #   - leading vehicle drives toward next intersection
        #   - ego follow leading vehicle
        # -
        # parallel events
        # - 

        # obstacle behavior
        obstacle_sequence = py_trees.composites.Sequence("Obstacle Behavior")
        obstacle_enter_road = py_trees.composites.Parallel("Obstalce entering road",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        obstacle_enter_road.add_child(DriveDistance(self.other_actors[1], self._reference_waypoint.lane_width*2.0))
        obstacle_enter_road.add_child(KeepVelocity(self.other_actors[1], self._second_actor_speed))
        obstacle_sequence.add_child(obstacle_enter_road)
        obstacle_sequence.add_child(TimeOut(5))
        
        obstacle_clear_road = py_trees.composites.Parallel("Obstalce clearing road",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        obstacle_clear_road.add_child(DriveDistance(self.other_actors[1], self._reference_waypoint.lane_width*3))
        obstacle_clear_road.add_child(KeepVelocity(self.other_actors[1], self._second_actor_speed))
        obstacle_sequence.add_child(obstacle_clear_road)


        # Leading vehicle behavior - drive until next intersection
        leader_sequence = py_trees.composites.Sequence("Leading Vehicle Behavior")
        driving_to_next_intersection = py_trees.composites.Parallel("Driving towards Intersection", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._first_actor_speed))
        driving_to_next_intersection.add_child(InTriggerDistanceToVehicle(self.other_actors[1],self.other_actors[0], 15))
        leader_sequence.add_child(driving_to_next_intersection)
        leader_sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))
        leader_sequence.add_child(DriveDistance(self.other_actors[1], self._reference_waypoint.lane_width*1.5))

        # leader_sequence.add_child(TimeOut(3))

        stop_near_intersection = py_trees.composites.Parallel("Waiting for end position near Intersection",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        stop_near_intersection.add_child(WaypointFollower(self.other_actors[0], 10))
        stop_near_intersection.add_child(InTriggerDistanceToNextIntersection(self.other_actors[0], 20))
        leader_sequence.add_child(stop_near_intersection)
        leader_sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))


        events = py_trees.composites.Parallel("Follow Leading Vehicle",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        events.add_child(obstacle_sequence)
        events.add_child(leader_sequence)

        # end condition
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=20,
                                                        name="FinalDistance")
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        #        traffic_manager.distance_to_leading_vehicle(ego_vehicle, 5.0)


        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(events)
        sequence.add_child(endcondition)
        sequence.add_child(TimeOut(10))
        sequence.add_child(ActorDestroy(self.other_actors[0]))
        sequence.add_child(ActorDestroy(self.other_actors[1]))

        return sequence
        
    def _create_behaviorOLD(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive towards obstacle.
        Once obstacle clears the road, make the other actor to drive towards the
        next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """

        # let the other actor drive until next intersection
        driving_to_next_intersection = py_trees.composites.Parallel(
            "Driving towards Intersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)


        obstacle_enter_road = py_trees.composites.Parallel("Obstalce enter road",
                                                           policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        obstacle_enter_road.add_child(DriveDistance(self.other_actors[1], self._reference_waypoint.lane_width*1.5))
        obstacle_enter_road.add_child(KeepVelocity(self.other_actors[1], self._second_actor_speed))



        obstacle_clear_road = py_trees.composites.Parallel("Obstalce clearing road",
                                                           policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        obstacle_clear_road.add_child(DriveDistance(self.other_actors[1], self._reference_waypoint.lane_width*2))
        obstacle_clear_road.add_child(KeepVelocity(self.other_actors[1], self._second_actor_speed))

        stop_near_intersection = py_trees.composites.Parallel(
            "Waiting for end position near Intersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        stop_near_intersection.add_child(WaypointFollower(self.other_actors[0], 10))
        stop_near_intersection.add_child(InTriggerDistanceToNextIntersection(self.other_actors[0], 20))

        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._first_actor_speed))
        driving_to_next_intersection.add_child(InTriggerDistanceToVehicle(self.other_actors[1],
                                                                          self.other_actors[0], 15))

        # end condition
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=20,
                                                        name="FinalDistance")
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(obstacle_enter_road)
        sequence.add_child(driving_to_next_intersection)
        sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))
        sequence.add_child(TimeOut(3))
        sequence.add_child(obstacle_clear_road)
        sequence.add_child(stop_near_intersection)
        sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))
        sequence.add_child(endcondition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))
        sequence.add_child(ActorDestroy(self.other_actors[1]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class FollowLeadingVehiclePedestriansCrossing(FollowLeadingVehicleWithObstruction):

    """
    This class holds everything required for a construction scenario
    The ego vehicle is passing through a road and encounters
    a stationary rectangular construction cones setup and traffic warning.

    This is a single ego vehicle scenario
    """
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=360):
        self._actor_position_shift = 0.9
        self._pedestrian_type = 'walker.*'  # blueprint filter of the pedestrian
        self._pedestrians = []      
        self._num_lane_changes = 2

        super(FollowLeadingVehiclePedestriansCrossing, self).__init__(world,
                                                               ego_vehicles,
                                                               config,
                                                               randomize=randomize, 
                                                               debug_mode=debug_mode, 
                                                               criteria_enable=criteria_enable, 
                                                               timeout=timeout)          

    def _init_settings(self):
        FollowLeadingVehicleWithObstruction._init_settings( self )

        pedestrian_speed = 4.0  # Speed of the pedestrian [m/s]

        self._pedestrians_location = 139.50
        self._offset_to_sidewalk = 4

        # pedestrian configurations
        self._pedestrian_configs = []
        self._pedestrian_configs.append( { "speed": pedestrian_speed*0.75, "offset":self._offset_to_sidewalk*0.75 } )
        self._pedestrian_configs.append( { "speed": pedestrian_speed*0.45, "offset":self._offset_to_sidewalk*0.95 } )
        self._pedestrian_configs.append( { "speed": pedestrian_speed*0.5, "offset":self._offset_to_sidewalk*0.60 } )
        self._pedestrian_configs.append( { "speed": pedestrian_speed*1.0, "offset":self._offset_to_sidewalk*0.7 } )
        self._pedestrian_configs.append( { "speed": pedestrian_speed*0.30, "offset":self._offset_to_sidewalk*0.65 } )


    def _initialize_obstructors(self, config):
        FollowLeadingVehicleWithObstruction._initialize_obstructors(self, config)

        location, _ = get_location_in_distance_from_wp(self._reference_waypoint, self._pedestrians_location, False)
        waypoint = self._map.get_waypoint(location)
        # actor_waypoint, _t2 = get_waypoint_in_distance(self._reference_waypoint, self._second_actor_location)
        # offset = {"yaw": 90, "z": 0.5, "k": -2}
        # actor_transform = self._get_sidewalk_transform(actor_waypoint, offset)
        # self._spawn_actor('vehicle.diamondback.century', actor_transform)

        # Get the pedestrian transform and spawn it
        #import pdb; pdb.set_trace()
        pedestrian_shift = self._actor_position_shift

        for config in self._pedestrian_configs:
            #waypoint = waypoint.next(pedestrian_shift)[0]
            transform = get_transform(waypoint, offset_yaw=270, offset_z=0.5, offset_right=config[ 'offset' ], offset_forward=pedestrian_shift)

            print('transform:', transform, 'shift:', pedestrian_shift)

            # transform = get_sidewalk_transform( waypoint, offset_yaw=270, offset_z=0.5, offset_dist=1.50 )
            actor = CarlaDataProvider.request_new_actor(self._pedestrian_type, transform)
            if actor:
                pedestrian = {}
                pedestrian[ 'transform' ] = transform
                pedestrian[ 'speed' ] = config['speed']
                pedestrian[ 'actor' ] = actor
                self._pedestrians.append( pedestrian )        
                self.other_actors.append( actor )
                pedestrian_shift += self._actor_position_shift

    def _create_behavior(self):
        
        # Build behavior tree
        location, _ = get_location_in_distance_from_wp(self._reference_waypoint, self._pedestrians_location, False)

        pedestrians_crossing_street = py_trees.composites.Parallel("Pedestrians Crossing",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        for i, pedestrian in enumerate(self._pedestrians):

            # Move the pedestrian
            collision_distance = self._offset_to_sidewalk * self._num_lane_changes
            collision_duration = collision_distance / pedestrian['speed']
            speed_duration = 1.5*collision_duration
            speed_distance = 1.5*collision_distance

            pedestrian_crossing = py_trees.composites.Sequence("Pedestrian" + str(i))
            pedestrian_crossing.add_child(InTriggerDistanceToVehicle(pedestrian['actor'],self.other_actors[0], 30))

            pedestrian_crossing.add_child(KeepVelocity(
                pedestrian['actor'], 
                pedestrian['speed'],
                duration=speed_duration, 
                distance=speed_distance, 
                name="Pedestrian" + str(i) + "Crossing"))

            pedestrians_crossing_street.add_child( pedestrian_crossing )



        # cyclist behavior
        obstacle_sequence = py_trees.composites.Sequence("Obstacle Behavior")
        obstacle_enter_road = py_trees.composites.Parallel("Obstalce entering road",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        obstacle_enter_road.add_child(DriveDistance(self.other_actors[1], self._reference_waypoint.lane_width*2.0))
        obstacle_enter_road.add_child(KeepVelocity(self.other_actors[1], self._second_actor_speed))
        obstacle_sequence.add_child(obstacle_enter_road)
        obstacle_sequence.add_child(TimeOut(1))
        
        obstacle_clear_road = py_trees.composites.Parallel("Obstalce clearing road",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        obstacle_clear_road.add_child(DriveDistance(self.other_actors[1], self._reference_waypoint.lane_width*3))
        obstacle_clear_road.add_child(KeepVelocity(self.other_actors[1], self._second_actor_speed))
        obstacle_sequence.add_child(obstacle_clear_road)


        # Leading vehicle behavior - drive until next intersection
        leader_sequence = py_trees.composites.Sequence("Leading Vehicle Behavior")
        driving_to_cyclist = py_trees.composites.Parallel("Driving towards Cyclist", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        driving_to_cyclist.add_child(WaypointFollower(self.other_actors[0], self._first_actor_speed))
        driving_to_cyclist.add_child(InTriggerDistanceToVehicle(self.other_actors[1],self.other_actors[0], 15))
        leader_sequence.add_child(driving_to_cyclist)
        leader_sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))
        leader_sequence.add_child(DriveDistance(self.other_actors[1], self._reference_waypoint.lane_width*1.5))

        if len(self._pedestrians) > 0:
            driving_to_pedestrians = py_trees.composites.Parallel("Driving towards Pedestrians", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
            driving_to_pedestrians.add_child(WaypointFollower(self.other_actors[0], self._first_actor_speed))
            driving_to_pedestrians.add_child(InTriggerDistanceToVehicle(self._pedestrians[0]['actor'],self.other_actors[0], 15))
            leader_sequence.add_child(driving_to_pedestrians)
            leader_sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))
            leader_sequence.add_child(pedestrians_crossing_street)

        stop_near_intersection = py_trees.composites.Parallel("Waiting for end position near Intersection",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        stop_near_intersection.add_child(WaypointFollower(self.other_actors[0], 10))
        stop_near_intersection.add_child(InTriggerDistanceToNextIntersection(self.other_actors[0], 20))
        leader_sequence.add_child(stop_near_intersection)
        leader_sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))


        events = py_trees.composites.Parallel("Follow Leading Vehicle",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        events.add_child(obstacle_sequence)
        events.add_child(leader_sequence)
        # events.add_child(pedestrians_crossing_street)

        # end condition
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=20,
                                                        name="FinalDistance")
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)


        sequence = py_trees.composites.Sequence("Sequence Behavior")
        # sequence.add_child(pedestrians_crossing_street)
        sequence.add_child(events)
        sequence.add_child(endcondition)
        sequence.add_child(TimeOut(10))

        for actor in self.other_actors:
            sequence.add_child(ActorDestroy(actor))

        return sequence