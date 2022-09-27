#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the key configuration parameters for an XML-based scenario
"""

from re import X
import carla

class SmartPoint:
    def __init__(self, x, y, z, pitch=0, yaw=0, roll=0, radius=3):
        self.transform = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw, roll=roll, pitch=pitch))
        self.radius = radius

    @staticmethod
    def parse_from_xml_node(node, rolename):
        """
        static method to initialize an SmartPoint from a given ET tree
        """
        x = float(node.attrib.get('x', 0))
        y = float(node.attrib.get('y', 0))
        z = float(node.attrib.get('z', 0))
        yaw = float(node.attrib.get('yaw', 0))        
        roll = float(node.attrib.get('roll', 0))        
        pitch = float(node.attrib.get('pitch', 0))        
        radius = float(node.attrib.get('radius', 0))   
        return SmartPoint(x, y, z, pitch, yaw, roll, radius)     

    @staticmethod
    def parse_from_json(node):
        """
        static method to initialize an SmartPoint from a given ET tree
        """
        x = float(node['x'])
        y = float(node['y'])
        z = float(node['z'])
        yaw = float(node['yaw'])        
        radius = float(node['radius'])   
        return SmartPoint(x, y, z, yaw=yaw, radius=radius)     


class ActorConfigurationData(object):

    """
    This is a configuration base class to hold model and transform attributes
    """

    def __init__(self, model, transform, rolename='other', speed=0, autopilot=False,
                 random=False, color=None, type="walker.*", category="car", 
                 destination_transform=None, 
                 sync_point=None, 
                 path=[],
                 path_is_terminal=False,
                 safety_radius=3, awareness_distance=30, trigger_distance=0, destination_is_waypoint=False, invisible_to_ego=False, args=None):
        self.model = model
        self.rolename = rolename
        self.transform = transform
        self.speed = speed
        self.awareness_distance = awareness_distance
        self.trigger_distance = trigger_distance
        self.invisible_to_ego = invisible_to_ego
        self.autopilot = autopilot
        self.random_location = random
        self.color = color
        self.type = type
        self.category = category
        self.destination_transform = destination_transform
        self.sync_point = sync_point
        self.path = path
        self.path_is_terminal = path_is_terminal
        self.safety_radius = safety_radius
        self.destination_is_waypoint = destination_is_waypoint
        self.args = args

    @staticmethod
    def parse_from_node(node, rolename):
        """
        static method to initialize an ActorConfigurationData from a given ET tree
        """

        model = node.attrib.get('model', 'vehicle.*')

        pos_x = float(node.attrib.get('x', 0))
        pos_y = float(node.attrib.get('y', 0))
        pos_z = float(node.attrib.get('z', 0))
        yaw = float(node.attrib.get('yaw', 0))

        transform = carla.Transform(carla.Location(x=pos_x, y=pos_y, z=pos_z), carla.Rotation(yaw=yaw))

        rolename = node.attrib.get('rolename', rolename)
        trigger_distance = float(node.attrib.get('trigger_distance', 0))
        awareness_distance = float(node.attrib.get('awareness_distance', 30))
        invisible_to_ego = int(node.attrib.get('trigger_distance', 0))
        invisible_to_ego = True if invisible_to_ego == 1 else False

        path_is_terminal = False
        if 'path_is_terminal' in node.keys():
            path_is_terminal = True if node.attrib.get('path_is_terminal', 'False') == 'True' else False

        speed = node.attrib.get('speed', 0)
        type = node.attrib.get('type', 0)

        safety_radius = 3
        destination_transform = None
        destination_is_waypoint = False
        if 'destination' in node.keys():
            destination = node.attrib.get('destination', None)
            if destination:
                x = float(destination['x'])
                y = float(destination['y'])
                z = float(destination['z'])            
                dyaw = float(destination['yaw'])
                safety_radius = float(destination['radius'])
                destination_transform = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=dyaw))
                if 'is_waypoint' in destination:
                    destination_is_waypoint = True if destination['is_waypoint'] == 1 else False

        sync_point = None
        if 'sync_point' in node.keys():
            sync_point = SmartPoint.parse_from_json( node.attrib.get('sync_point', None) )

        path = []
        path_data =  node.attrib.get('path', None) if 'path' in node.keys() else None
        if path_data:
            for loc in path_data:
                p_x = float(loc['x'])
                p_y = float(loc['y'])
                p_z = float(loc['z'])            
                p_yaw = float(loc['yaw'])
                p_transform = carla.Transform(carla.Location(x=p_x, y=p_y, z=p_z), carla.Rotation(yaw=p_yaw))
                path.append( p_transform )


        autopilot = False
        if 'autopilot' in node.keys():
            autopilot = True

        random_location = False
        if 'random_location' in node.keys():
            random_location = True

        color = node.attrib.get('color', None)

        return ActorConfigurationData(model, transform, rolename, speed, autopilot, random_location, color, type, 
        destination_transform=destination_transform, 
        sync_point=sync_point,  
        path=path,      
        path_is_terminal=path_is_terminal,
        safety_radius=safety_radius,awareness_distance=awareness_distance, trigger_distance=trigger_distance, destination_is_waypoint=destination_is_waypoint,
        invisible_to_ego=invisible_to_ego)


class ScenarioConfiguration(object):

    """
    This class provides a basic scenario configuration incl.:
    - configurations for all actors
    - town, where the scenario should be executed
    - name of the scenario (e.g. ControlLoss_1)
    - type is the class of scenario (e.g. ControlLoss)
    """

    trigger_points = []
    ego_vehicles = []
    other_actors = []
    town = None
    name = None
    type = None
    route = None
    agent = None
    weather = carla.WeatherParameters()
    friction = None
    subtype = None
    route_var_name = None
