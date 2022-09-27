#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides all atomic scenario behaviors required to realize
complex, realistic scenarios such as "follow a leading vehicle", "lane change",
etc.

The atomic behaviors are implemented with py_trees.
"""

from __future__ import print_function

import copy
import math
import operator
import os
import random
import time
import subprocess

import numpy as np
import py_trees
from py_trees.blackboard import Blackboard
import networkx

import carla
from agents.navigation.basic_agent import BasicAgent, LocalPlanner
from agents.navigation.local_planner import RoadOption
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.tools.misc import is_within_distance

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.actorcontrols.actor_control import ActorControl
from srunner.scenariomanager.timer import GameTime
from srunner.tools.scenario_helper import detect_lane_obstacle
from srunner.tools.scenario_helper import generate_target_waypoint_list_multilane


from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (AtomicBehavior, ChangeAutoPilot)

import srunner.tools as sr_tools

EPSILON = 0.001

class ChangeEgoSpeed(AtomicBehavior):
    def __init__(self, actor):
        """
        Constructor
        """
        self._ego_actor = actor
        # ignore_walkers_percentage
        # self._traffic_mananger.global_percentage_speed_difference(80)
        self._tm = CarlaDataProvider.get_client().get_trafficmanager(
            CarlaDataProvider.get_traffic_manager_port())

        super(ChangeEgoSpeed, self).__init__(name="ChangeEgoSpeed")

    def update(self):
        """
        Execute one control loop step for all actor controls.
        returns:
            py_trees.common.Status.RUNNING
        """
        self._tm.global_percentage_speed_difference(80)
        # self._tm.distance_to_leading_vehicle(self._actor, 120)

        # import pdb; pdb.set_trace()
        velocity = CarlaDataProvider.get_velocity(self._ego_actor)
        self._ego_actor.update_target_speed(1.0)        
        
        return py_trees.common.Status.SUCCESS

class DecelerateToStop(AtomicBehavior):
    def __init__(self, actor, throttle_value=0, delta_velocity=10, trigger_distance=1,
                 max_distance=500, name="DecelerateToStop"):
        """
        Setup parameters
        The target_speet is calculated on the fly.
        """
        super(DecelerateToStop, self).__init__(name, actor)
        self._throttle_value = throttle_value
        self._delta_velocity = delta_velocity  # 1m/s=3.6km/h
        self._trigger_distance = trigger_distance
        self._max_distance = max_distance

        self._control, self._type = get_actor_control(actor)

        self._initial_actor_pos = None
        
        # ignore_walkers_percentage
        # self._traffic_mananger.global_percentage_speed_difference(80)
        self._tm = CarlaDataProvider.get_client().get_trafficmanager(
            CarlaDataProvider.get_traffic_manager_port())


    def initialise(self):

        # get initial actor position
        self._initial_actor_pos = CarlaDataProvider.get_location(self._actor)
        # self._last_actor_pos = self._initial_actor_pos
        super(DecelerateToStop, self).initialise()

    def update(self):

        # get actor speed
        actor_speed = CarlaDataProvider.get_velocity(self._actor)
        target_speed = 0

        # distance covered
        # distance = CarlaDataProvider.get_location(self._actor).distance( self._last_actor_pos )

        # self._tm.global_percentage_speed_difference(80)
        # self._tm.distance_to_leading_vehicle(self._actor, 120)
        # self._tm.vehicle_percentage_speed_difference( self._actor, 90)
        # self._tm.vehicle_percentage_speed_difference( self._actor, -80)

        # import pdb; pdb.set_trace()
        # velocity = CarlaDataProvider.get_velocity(self._actor)
        # self._actor.update_target_speed(1.0)     
        

        # driven distance of actor
        driven_distance = CarlaDataProvider.get_location(self._actor).distance(self._initial_actor_pos)

        self._control.throttle = 0

        # if actor_speed > target_speed:
        #     # set throttle to throttle_value to accelerate
        #     self._control.throttle = self._throttle_value

        # if actor_speed <= target_speed:
        #     # keep velocity until the actors are in trigger distance
        #     self._control.throttle = 0

        # self._actor.apply_control(self._control)

        # new status:EPSILON
        #if driven_distance >= self._trigger_distance and :
        if actor_speed <= target_speed:
            new_status = py_trees.common.Status.SUCCESS

        elif driven_distance > self._max_distance:
            new_status = py_trees.common.Status.FAILURE
        else:
            new_status = py_trees.common.Status.RUNNING

        return new_status


class HBSChangeAutoPilot(ChangeAutoPilot):

    """
    This class contains an atomic behavior to disable/enable the use of the autopilot.

    Important parameters:
    - actor: CARLA actor to execute the behavior
    - activate: True (=enable autopilot) or False (=disable autopilot)
    - lane_change: Traffic Manager parameter. True (=enable lane changes) or False (=disable lane changes)
    - distance_between_vehicles: Traffic Manager parameter
    - max_speed: Traffic Manager parameter. Max speed of the actor. This will only work for road segments
                 with the same speed limit as the first one

    The behavior terminates after changing the autopilot state
    """

    def update(self):
        """
        De/activate autopilot
        """
        ChangeAutoPilot.update(self)

        if self._parameters is not None:

            if "ignore_signs_percentage" in self._parameters:
                ignore_signs_percentage = self._parameters["ignore_signs_percentage"]
                self._tm.ignore_signs_percentage(self._actor, ignore_signs_percentage)

            if "ignore_lights_percentage" in self._parameters:
                ignore_lights_percentage = self._parameters["ignore_lights_percentage"]
                self._tm.ignore_lights_percentage(self._actor, ignore_lights_percentage)

        new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status