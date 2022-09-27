#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides all atomic scenario behaviors that reflect
trigger conditions to either activate another behavior, or to stop
another behavior.

For example, such a condition could be "InTriggerRegion", which checks
that a given actor reached a certain region on the map, and then starts/stops
a behavior of this actor.

The atomics are implemented with py_trees and make use of the AtomicCondition
base class
"""

from __future__ import print_function

import operator
import datetime
import math
import py_trees
import carla

from agents.navigation.global_route_planner import GlobalRoutePlanner

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import calculate_distance
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from srunner.tools.scenario_helper import get_distance_along_route

from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (AtomicCondition,InTriggerDistanceToLocation)

import srunner.tools as sr_tools

EPSILON = 0.001


class NotInProximityOfActor(AtomicCondition):

    def __init__(self,
                 actor,
                 target,
                 distance,
                 name="OutsideProximityOfActor"):
        """
        Setup trigger distance
        """
        super(NotInProximityOfActor, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._target = target
        self._actor = actor
        self._distance = distance
        self._comparison_operator = operator.gt
        self._map = CarlaDataProvider.get_map()

    def update(self):
        """
        Check if the actor is within trigger distance to the target location
        """
        new_status = py_trees.common.Status.RUNNING

         # keep running if actor position is not available
        location = CarlaDataProvider.get_location(self._actor)
        if location is None:
            return py_trees.common.Status.RUNNING

        # keep running if target position is not available
        actor_transform = CarlaDataProvider.get_transform(self._actor)
        if actor_transform is None:
            return py_trees.common.Status.RUNNING

        # keep running if target position is not available
        target_transform = CarlaDataProvider.get_transform(self._target)
        if target_transform is None:
            return py_trees.common.Status.RUNNING

        distance_between_actors = calculate_distance(actor_transform.location, target_transform.location)

        if operator.ge(distance_between_actors, self._distance):
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status


class OutsideProximityViewOfStoppedActor(AtomicCondition):

    def __init__(self,
                 actor,
                 target,
                 distance,
                 view_limits_angle=0.45,
                 name="OutsideProximityViewOfStoppedActor"):
        """
        Setup trigger distance
        """
        super(OutsideProximityViewOfStoppedActor, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._target = target
        self._actor = actor
        self._distance = distance
        self._view_limits_angle = view_limits_angle
        self._comparison_operator = operator.gt
        self._map = CarlaDataProvider.get_map()

    def update(self):
        """
        Check if the actor is within trigger distance to the target location
        """
        new_status = py_trees.common.Status.RUNNING

        # Keep running if the actor is moving
        actor_speed = CarlaDataProvider.get_velocity(self._actor)
        # if actor_speed > EPSILON:
        #     print('<-Outside-> actor_speed:', actor_speed)
        #     return py_trees.common.Status.RUNNING

        # keep running if actor position is not available
        location = CarlaDataProvider.get_location(self._actor)
        if location is None:
            print('<-Outside-> location')
            return py_trees.common.Status.RUNNING

        # keep running if target position is not available
        target_transform = CarlaDataProvider.get_transform(self._target)
        if target_transform is None:
            print('<-Outside-> target_transform')
            return py_trees.common.Status.RUNNING

        # Keep running if waypoints not available
        actor_waypoint = self._map.get_waypoint(location)
        if actor_waypoint is None:
            print('<-Outside-> actor_waypoint')
            return py_trees.common.Status.RUNNING

        # Determine if target is in actor's view
        # success if target is outside of actor's view
        actor_dir = actor_waypoint.transform.get_forward_vector()
        actor_target_dir = target_transform.location - location
        actor_target_dir = actor_target_dir.make_unit_vector()
        actor_dot = actor_dir.x * actor_target_dir.x + actor_dir.y * actor_target_dir.y + actor_dir.z * actor_target_dir.z

        distance_between_actors = calculate_distance(location, target_transform.location)

        # if actor_dot < self._view_limits_angle:
        #     import pdb; pdb.set_trace()

        if actor_dot <= self._view_limits_angle:
        
            new_status = py_trees.common.Status.SUCCESS

        else:

            target_speed = CarlaDataProvider.get_velocity(self._target)
            if target_speed <= EPSILON:
                if actor_dot <= 0.9:
                    new_status = py_trees.common.Status.SUCCESS

            if operator.ge(distance_between_actors, self._distance):
                new_status = py_trees.common.Status.SUCCESS


            # # Determine if target is moving away from actor
            # target_dir = target_transform.get_forward_vector()
            # target_actor_dir = location - target_transform.location
            # target_dot = actor_dir.x * target_actor_dir.x + actor_dir.y * target_actor_dir.y + actor_dir.z * target_actor_dir.z

            # if actor_dot < 45 and target_speed <= EPSILON:
            #     new_status = py_trees.common.Status.SUCCESS
                
            # if actor_dot < 45:
            #     if target_speed <= EPSILON:
            #         new_status = py_trees.common.Status.SUCCESS
            #     else:
            #         if target_dot <= 0.0 and operator.ge(distance_between_actors, self._distance):
            #             new_status = py_trees.common.Status.SUCCESS
            # else:
            #         if operator.ge(distance_between_actors, self._distance):
            #             new_status = py_trees.common.Status.SUCCESS

            # if target_speed > EPSILON:
                
            #     # import pdb; pdb.set_trace()
            #     print('-----> 2.target dot: ', target_dot, 'speed:', target_speed)
            #     # if target_speed > EPSILON and dot_ve_wp > 0.0:
            #     #if target_dot <= 0.0 and operator.ge(distance_between_actors, self._distance):
            #     if actor_dot < 45:
            #         if operator.ge(distance_between_actors, self._distance):
            #             if actor_dot > 45 and target_dot <= 0.0:
            #                 new_status = py_trees.common.Status.SUCCESS
 
            # else:

            #     # Ok to move if there's clearance infront of the actor
            #     #if operator.le(distance_between_actors, self._distance) and actor_dot < 45:
            #     if actor_dot < 45:
            #         new_status = py_trees.common.Status.SUCCESS
                        


                # else:
                #     import pdb; pdb.set_trace()

        # print('<-Outside-> actor dot: ', actor_dot, 'limit:', self._view_limits_angle, 'speed:',actor_speed, 'distance:',distance_between_actors, 'status:',new_status, self.name)

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status

class TargetProximityInfo:
    def __init__(self, actor, distance, success_on_move=True, success_on_stopped=True):
        self.actor = actor
        self.distance = distance
        self.success_on_move = success_on_move
        self.success_on_stopped = success_on_stopped

class TargetsNotInProximityViewOfActor(AtomicCondition):
    def __init__(self,
                 actor,
                 targets,
                 view_angle=0,
                 success_on_all=False,
                 name="TargetsNotInProximityViewOfActor"):
        """
        Setup trigger distance
        """
        super(TargetsNotInProximityViewOfActor, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._targets = targets
        self._actor = actor
        self._view_angle = view_angle
        self._success_on_all = success_on_all
        self._map = CarlaDataProvider.get_map()

    def update(self):
        """
        Check if the targets are within view distance of actor
        """
        new_status = py_trees.common.Status.RUNNING

        location = CarlaDataProvider.get_location(self._actor)
        if location is None:
            return new_status

        actor_waypoint = self._map.get_waypoint(location)
        if actor_waypoint is None:
            return new_status

        success_count = 0
        for target in self._targets:

            target_location = CarlaDataProvider.get_location(target.actor)
            if target_location is None:
                continue

            target_speed = CarlaDataProvider.get_velocity(target.actor)
            if target_speed is None:
                continue

            # if target.success_on_move and target_speed <= EPSILON:
            #     continue

            # if target.success_on_stopped and target_speed > EPSILON:
            #     continue

            # check if target is not in proximity of actor
            proximity_distance = calculate_distance(location, target_location)
            # check if target is actor's view
            actor_dir = actor_waypoint.transform.get_forward_vector()
            actor_target_dir = target_location - location
            actor_target_dir = actor_target_dir.make_unit_vector()
            dot_ve_wp = actor_dir.x * actor_target_dir.x + actor_dir.y * actor_target_dir.y + actor_dir.z * actor_target_dir.z

            # if dot_ve_wp < self._view_angle:
            target_waypoint = self._map.get_waypoint(target_location)
            if target_waypoint:
                if target_waypoint.lane_id != actor_waypoint.lane_id:
                    success_count = success_count + 1
                    continue


            if operator.gt(proximity_distance, target.distance) or (dot_ve_wp < self._view_angle):
                 success_count = success_count + 1

            # print('<-NotInProximity->', 'dot_ve_wp:',dot_ve_wp, 'proximity_distance:', proximity_distance, 'distance:', target.distance, self.name)

        if (self._success_on_all and success_count == len(self._targets)) or (self._success_on_all==False and success_count>0):
            new_status = py_trees.common.Status.SUCCESS

        # print('<-NotInProximity->', 'status:', self.status, 'new_status:',new_status, self.name, 'success_count:', success_count)

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status



class TargetsInProximityViewOfActor(AtomicCondition):
    def __init__(self,
                 actor,
                 targets,
                 view_angle=0,
                 success_on_all=False,
                 name="TargetsInProximityViewOfActor"):
        """
        Setup trigger distance
        """
        super(TargetsInProximityViewOfActor, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._targets = targets
        self._actor = actor
        self._view_angle = view_angle
        self._success_on_all = success_on_all
        self._map = CarlaDataProvider.get_map()

    def update(self):
        """
        Check if the targets are within view distance of actor
        """
        new_status = py_trees.common.Status.RUNNING

        location = CarlaDataProvider.get_location(self._actor)
        if location is None:
            print('<-InProximity->', 'bad location:', self.name)
            return new_status

        actor_waypoint = self._map.get_waypoint(location)
        if actor_waypoint is None:
            print('<-InProximity->', 'bad waypoint:', self.name)
            return new_status

        success_count = 0
        for target in self._targets:

            target_location = CarlaDataProvider.get_location(target.actor)
            if target_location is None:
                continue

            target_speed = CarlaDataProvider.get_velocity(target.actor)
            if target_speed is None:
                continue

            # if target.success_on_move and target_speed <= EPSILON:
            #     continue

            # if target.success_on_stopped and target_speed > EPSILON:
            #     continue

            # check if target is in proximity of actor
            proximity_distance = calculate_distance(location, target_location)
            if operator.le(proximity_distance, target.distance):

                # check if target is actor's view
                actor_dir = actor_waypoint.transform.get_forward_vector()
                actor_target_dir = target_location - location
                actor_target_dir = actor_target_dir.make_unit_vector()
                dot_ve_wp = actor_dir.x * actor_target_dir.x + actor_dir.y * actor_target_dir.y + actor_dir.z * actor_target_dir.z

                if dot_ve_wp >= self._view_angle:
                    success_count = success_count + 1

                # print('<-InProximity->', 'dot_ve_wp:',dot_ve_wp)

            # print('<-InProximity->', 'proximity_distance:', proximity_distance, 'distance:', target.distance)
        if (self._success_on_all and success_count == len(self._targets)) or (self._success_on_all==False and success_count>0):
            new_status = py_trees.common.Status.SUCCESS

        # print('<-InProximity->', 'status:', self.status,new_status, self.name, 'success_count:', success_count, '#targets:', len(self._targets))

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status


class InProximityViewOfActor(AtomicCondition):

    def __init__(self,
                 actor,
                 target,
                 distance,
                 view_angle=0,
                 name="InProximityViewOfActor"):
        """
        Setup trigger distance
        """
        super(InProximityViewOfActor, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._target = target
        self._actor = actor
        self._distance = distance
        self._view_angle = view_angle
        self._map = CarlaDataProvider.get_map()

    def update(self):
        """
        Check if the actor is within trigger distance to the target location
        """
        new_status = py_trees.common.Status.RUNNING

        location = CarlaDataProvider.get_location(self._actor)
        if location is None:
            return new_status


        target_location = CarlaDataProvider.get_location(self._target)
        if target_location is None:
            return new_status

        actor_waypoint = self._map.get_waypoint(location)
        if actor_waypoint is None:
            return new_status

        in_proximity_distance = False
        proximity_distance = calculate_distance(location, target_location)
        if operator.le(proximity_distance, self._distance):
            in_proximity_distance = True


        # Wait for the vehicle to be in front
        actor_dir = actor_waypoint.transform.get_forward_vector()
        actor_target_dir = target_location - location
        actor_target_dir = actor_target_dir.make_unit_vector()
        dot_ve_wp = actor_dir.x * actor_target_dir.x + actor_dir.y * actor_target_dir.y + actor_dir.z * actor_target_dir.z

        # print('InProximityViewOfActor - dot:', dot_ve_wp, 'proximity:', proximity_distance,  in_proximity_distance)
        in_view = False
        if dot_ve_wp >= self._view_angle:
            in_view = True

        if in_view and in_proximity_distance:
            new_status = py_trees.common.Status.SUCCESS

        if self.name.startswith('egostop'):
            print('<-InProximity-> dot: ', dot_ve_wp, 'limit:', self._view_angle, 'distance:',proximity_distance, 'prox', self._distance, 'status:',new_status, self.name)

        # import traceback
        # traceback.print_stack()

        # import pdb; pdb.set_trace()

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status


class InProximityViewOfActorOneShot(InProximityViewOfActor):
    def __init__(self,
                 actor,
                 target,
                 distance,
                 view_angle=0,
                 name="InProximityViewOfActorOneShot"):
        """
        Setup trigger distance
        """
        self._oneshot_status = py_trees.common.Status.INVALID

        super(InProximityViewOfActorOneShot, self).__init__(
                 actor,
                 target,
                 distance,
                 view_angle=view_angle,
                 name=name)        

    def update(self):
        
        if self._oneshot_status != py_trees.common.Status.SUCCESS:
            self._oneshot_status = InProximityViewOfActor.update(self)
        
        return self._oneshot_status


class InTriggerDistanceToLocationOneShot(InTriggerDistanceToLocation):
    def __init__(self,
                 actor,
                 target_location,
                 distance,
                 comparison_operator=operator.lt,
                 name="InTriggerDistanceToLocationOneShot"):
        """
        Setup trigger distance
        """
        self._oneshot_status = py_trees.common.Status.INVALID

        super(InTriggerDistanceToLocationOneShot, self).__init__(
                 actor,
                 target_location,
                 distance,
                 comparison_operator=comparison_operator,name=name)        

    def update(self):
        
        if self._oneshot_status != py_trees.common.Status.SUCCESS:
            self._oneshot_status = InTriggerDistanceToLocation.update(self)
        
        return self._oneshot_status
