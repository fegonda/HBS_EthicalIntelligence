#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides Challenge routes as standalone scenarios
"""

from __future__ import print_function

import math
import traceback
import xml.etree.ElementTree as ET
from numpy import random

import py_trees

import carla

from agents.navigation.local_planner import RoadOption

# pylint: disable=line-too-long
from srunner.scenarioconfigs.scenario_configuration import ScenarioConfiguration, ActorConfigurationData
from srunner.hbs.scenarioconfigs.actor_configuration import HBSActorConfigurationData

# pylint: enable=line-too-long
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle, ScenarioTriggerer
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.route_parser import RouteParser, TRIGGER_THRESHOLD, TRIGGER_ANGLE_THRESHOLD
from srunner.hbs.tools.entity_parser import EntityParser
from srunner.tools.route_manipulation import interpolate_trajectory
from srunner.tools.py_trees_port import oneshot_behavior


from srunner.scenarios.route_scenario import RouteScenario
from srunner.scenarios.route_scenario import convert_json_to_transform, compare_scenarios, convert_json_to_actor

from srunner.scenarios.control_loss import ControlLoss
from srunner.scenarios.follow_leading_vehicle import FollowLeadingVehicle
from srunner.scenarios.follow_leading_vehicle import FollowLeadingVehicleWithObstacle
from srunner.scenarios.object_crash_vehicle import DynamicObjectCrossing
from srunner.scenarios.object_crash_intersection import VehicleTurningRoute
from srunner.scenarios.other_leading_vehicle import OtherLeadingVehicle
from srunner.scenarios.maneuver_opposite_direction import ManeuverOppositeDirection
from srunner.scenarios.junction_crossing_route import SignalJunctionCrossingRoute, NoSignalJunctionCrossingRoute
from srunner.hbs.scenarios.hbs_scenarios import FollowVehicle, PedestriansCrossingOtherVehiclesYield, FollowLeadingVehiclePedestriansCrossing, CyclistCrossing, CyclistsCrossing, PedestrianCrossing, PedestrianCrossingOppositeSidewalk,  PedestriansCrossing, CrowdCrossingOppositeSidewalk, FollowLeadingVehicleWithObstruction

from srunner.hbs.scenarios.traffic_negotiation import TrafficNegotiation

NUMBER_CLASS_TRANSLATION = {
    "Scenario1": ControlLoss,
    "Scenario2": FollowLeadingVehicle,
    "Scenario3": DynamicObjectCrossing,
    "Scenario4": VehicleTurningRoute,
    "Scenario5": OtherLeadingVehicle,
    "Scenario6": ManeuverOppositeDirection,
    "Scenario7": SignalJunctionCrossingRoute,
    "Scenario8": SignalJunctionCrossingRoute,
    "Scenario9": SignalJunctionCrossingRoute,
    "Scenario10": NoSignalJunctionCrossingRoute,
    "Scenario15": FollowLeadingVehicleWithObstruction,
    "Scenario16": FollowLeadingVehiclePedestriansCrossing,
    "Scenario11": CyclistCrossing,
    "Scenario12": PedestrianCrossing,
    "Scenario13": PedestrianCrossingOppositeSidewalk,
    "Scenario14": PedestriansCrossing,
    "Scenario15": PedestriansCrossingOtherVehiclesYield,
    "Scenario16": FollowVehicle,
    "Scenario17": CrowdCrossingOppositeSidewalk,
    "Scenario18": TrafficNegotiation
    # "Scenario18": PedestrianCrossingNoBlocker,
    # "Scenario19": CrowdCrossingOppositeSidewalkNoBlocker
}


def convert_json_to_actor_config(actor_dict, rolename):
    """
    Convert a JSON string to an HBSActorConfigurationData dictionary
    """
    node = ET.Element('pedestrian')
    node.set('x', actor_dict['x'])
    node.set('y', actor_dict['y'])
    node.set('z', actor_dict['z'])
    node.set('yaw', actor_dict['yaw'])
    node.set('speed', actor_dict['speed'])
    node.set('type', actor_dict['type'])
    node.set('rolename', rolename)

    
    invisible_to_ego = actor_dict['invisible_to_ego'] if 'invisible_to_ego' in actor_dict else 0
    node.set('invisible_to_ego', invisible_to_ego)  
    awareness_distance = actor_dict['awareness_distance'] if 'awareness_distance' in actor_dict else 30
    node.set('awareness_distance', actor_dict['awareness_distance'])
    trigger_distance = actor_dict['trigger_distance'] if 'trigger_distance' in actor_dict else 0
    node.set('trigger_distance', trigger_distance)  

    if 'path_is_terminal' in actor_dict:
        node.set('path_is_terminal', actor_dict['path_is_terminal'])

    if 'destination' in actor_dict:
        node.set('destination', actor_dict['destination'])

    if 'sync_point' in actor_dict:
        node.set('sync_point', actor_dict['sync_point'])

    if 'path' in actor_dict:
        node.set('path', actor_dict['path'])


    return HBSActorConfigurationData.parse_from_node(node, rolename)


class HBSRouteScenario(RouteScenario):

    """
    Implementation of a RouteScenario, i.e. a scenario that consists of driving along a pre-defined route,
    along which several smaller scenarios are triggered
    """

    def __init__(self, world, config, debug_mode=False, criteria_enable=True, timeout=300, enable_background_activty=False, event_names=[], entities_files=None):
        """
        Setup all relevant parameters and create scenarios along route
        """
        self._enable_background_activty = enable_background_activty
        self._event_names = event_names
        self._entities_files = entities_files

        super(HBSRouteScenario, self).__init__(
                                            config=config,
                                            world=world,
                                            debug_mode=debug_mode,
                                            timeout=timeout,
                                            criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Set other_actors to the superset of all scenario actors
        """

        # Create the background activity of the route
        town_amount = {
            'Town01': 120,
            'Town02': 100,
            'Town03': 120,
            'Town04': 200,
            'Town05': 120,
            'Town06': 150,
            'Town07': 110,
            'Town08': 180,
            'Town09': 300,
            'Town10': 120,
        }

        amount = town_amount[config.town] if self._enable_background_activty and config.town in town_amount else 0

        new_actors = CarlaDataProvider.request_new_batch_actors('vehicle.*',
                                                                amount,
                                                                carla.Transform(),
                                                                autopilot=True,
                                                                random_location=True,
                                                                rolename='background')

        if new_actors is None:
            raise Exception("Error: Unable to add the background activity, all spawn points were occupied")

        for _actor in new_actors:
            self.other_actors.append(_actor)

        # Add all the actors of the specific scenarios to self.other_actors
        for scenario in self.list_scenarios:
            self.other_actors.extend(scenario.other_actors)
        
        for i, actor in enumerate(self.other_actors):
            if actor.type_id == self.ego_vehicle.type_id:
                self.other_actors.pop(i) # removes reference to any DReyeVR ego vehicles that might've been spawned
                # NOTE: even if it thinks it spawned DReyeVR ego vehicles, it cant. Only one can be spawned by the
                # carla ActorRegistry (See RegisterActor)

        # add entities
        self._add_entities()

    def _add_entities(self):
        if not self._entities_files: return

        for entities_file in self._entities_files:
            entities = EntityParser.parse( entities_file )
            for entity in entities:
                try:
                    location = carla.Location( entity.x, entity.y, entity.z )
                    transform = carla.Transform( location, carla.Rotation(yaw=entity.yaw, roll=entity.roll, pitch=entity.pitch))
                    actor = CarlaDataProvider.request_new_actor( entity.type, transform)
                    self.other_actors.append( actor )

                    # static.set_simulate_physics(enabled=False)
                except RuntimeError as r:
                    # We keep retrying until we spawn
                    print("Unable to spawn actor " + entity.type, transform)


    def _build_scenario_instances(self, world, ego_vehicle, scenario_definitions,
                                  scenarios_per_tick=5, timeout=300, debug_mode=False):
        """
        Based on the parsed route and possible scenarios, build all the scenario classes.
        """
        scenario_instance_vec = []

        if debug_mode:
            for scenario in scenario_definitions:
                loc = carla.Location(scenario['trigger_position']['x'],
                                     scenario['trigger_position']['y'],
                                     scenario['trigger_position']['z']) + carla.Location(z=2.0)
                world.debug.draw_point(loc, size=0.3, color=carla.Color(255, 0, 0), life_time=100000)
                #<FG-changes>
                # world.debug.draw_string(loc, str(scenario['name']), draw_shadow=False,
                                        # color=carla.Color(0, 0, 255), life_time=100000, persistent_lines=True)

                scenario_info = str(scenario['name'])
                scenario_info += '\n\r' + str(NUMBER_CLASS_TRANSLATION[scenario['name']].__name__)
                scenario_info += '\n\r['
                scenario_info += str(scenario['trigger_position']['x']) +','
                scenario_info += str(scenario['trigger_position']['y']) +','
                scenario_info += str(scenario['trigger_position']['z']) + ']'
                world.debug.draw_string(loc,scenario_info, draw_shadow=False,
                                        color=carla.Color(0, 0, 255), life_time=100000, persistent_lines=True)

                # extent = carla.Location(111.5, 111.5, 111.5)
                # world.debug.draw_box(box=carla.BoundingBox(loc, extent * 1e-2), rotation=carla.Rotation(yaw=0), life_time=1000, thickness=0.15, color=carla.Color(r=0,g=255,b=0))


                #</FG-changes>


        for scenario_number, definition in enumerate(scenario_definitions):
            # Get the class possibilities for this scenario number
            scenario_class = NUMBER_CLASS_TRANSLATION[definition['name']]

            # Create the other actors that are going to appear
            if definition['other_actors'] is not None:
                list_of_actor_conf_instances = self._get_actors_instances(definition['other_actors'])
            else:
                list_of_actor_conf_instances = []
            # Create an actor configuration for the ego-vehicle trigger position

            egoactor_trigger_position = convert_json_to_transform(definition['trigger_position'])
            scenario_configuration = ScenarioConfiguration()
            scenario_configuration.other_actors = list_of_actor_conf_instances
            scenario_configuration.trigger_points = [egoactor_trigger_position]
            scenario_configuration.subtype = definition['scenario_type']
            scenario_configuration.ego_vehicles = [ActorConfigurationData(ego_vehicle.type_id,
                                                                          ego_vehicle.get_transform(),
                                                                          'hero')]
            route_var_name = "ScenarioRouteNumber{}".format(scenario_number)
            scenario_configuration.route_var_name = route_var_name

            try:
                scenario_instance = scenario_class(world, [ego_vehicle], scenario_configuration,
                                                   criteria_enable=False, timeout=timeout)
                # Do a tick every once in a while to avoid spawning everything at the same time
                if scenario_number % scenarios_per_tick == 0:
                    if CarlaDataProvider.is_sync_mode():
                        world.tick()
                    else:
                        world.wait_for_tick()

                scenario_number += 1
            except Exception as e:      # pylint: disable=broad-except
                if debug_mode:
                    traceback.print_exc()
                print("Skipping scenario '{}' due to setup error: {}".format(definition['name'], e))
                continue

            scenario_instance_vec.append(scenario_instance)

        return scenario_instance_vec                



    def _scenario_sampling(self, potential_scenarios_definitions, random_seed=0):
        """
        The function used to sample the scenarios that are going to happen for this route.
        """

        # fix the random seed for reproducibility
        rng = random.RandomState(random_seed)

        def position_sampled(scenario_choice, sampled_scenarios):
            """
            Check if a position was already sampled, i.e. used for another scenario
            """
            for existent_scenario in sampled_scenarios:
                # If the scenarios have equal positions then it is true.
                if compare_scenarios(scenario_choice, existent_scenario):
                    return True

            return False

        def filter_scenarios(scenarios, event_names):
            results = []
            if len(event_names) > 0:
                for scenario in scenarios:
                    if scenario['event_id'] in event_names:
                        results.append( scenario )
            else:
                results = scenarios
            return results

        # The idea is to randomly sample a scenario per trigger position.
        sampled_scenarios = []
        for trigger in potential_scenarios_definitions.keys():
            possible_scenarios = filter_scenarios( potential_scenarios_definitions[trigger], self._event_names )

            if len( possible_scenarios ) == 0: continue

            #possible_scenarios = potential_scenarios_definitions[trigger]
            print('<trigger>:', trigger)

            scenario_choice = rng.choice(possible_scenarios)
            del possible_scenarios[possible_scenarios.index(scenario_choice)]
            # We keep sampling and testing if this position is present on any of the scenarios.
            while position_sampled(scenario_choice, sampled_scenarios):
                if possible_scenarios is None or not possible_scenarios:
                    scenario_choice = None
                    break
                scenario_choice = rng.choice(possible_scenarios)
                del possible_scenarios[possible_scenarios.index(scenario_choice)]

            if scenario_choice is not None:
                print('<scenario_choice>:', scenario_choice)
                sampled_scenarios.append(scenario_choice)

        return sampled_scenarios        


    def _get_actors_instances(self, list_of_antagonist_actors):
        """
        Get the full list of actor instances.
        """

        list_of_actors = RouteScenario._get_actors_instances(self, list_of_antagonist_actors)

        # import pdb; pdb.set_trace()
        if 'pedestrians' in list_of_antagonist_actors:
            for actor_def in list_of_antagonist_actors['pedestrians']:
                list_of_actors.append(convert_json_to_actor_config(actor_def, 'pedestrian'))

        if 'vehicles' in list_of_antagonist_actors:
            for actor_def in list_of_antagonist_actors['vehicles']:
                list_of_actors.append(convert_json_to_actor_config(actor_def, 'vehicle'))

        if 'cyclists' in list_of_antagonist_actors:
            for actor_def in list_of_antagonist_actors['cyclists']:
                list_of_actors.append(convert_json_to_actor_config(actor_def, 'cyclist'))

        return list_of_actors        