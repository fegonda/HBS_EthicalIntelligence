import carla
from agents.navigation.local_planner import RoadOption


class EntityConfiguration(object):

    """
    This class provides the basic configuration for an entity
    """

    def __init__(self, data):
        self.parse_xml( data )

    def parse_xml(self, node):
        """
        Parse Entity config XML
        """
        self.x = float(node.attrib.get('x', 0))
        self.y = float(node.attrib.get('y', 0))
        self.z = float(node.attrib.get('z', 0))
        self.yaw = float(node.attrib.get('yaw', 0))
        self.roll = float(node.attrib.get('roll', 0))
        self.pitch = float(node.attrib.get('pitch', 0))
        self.type = node.attrib.get('type', '')
        self.route = node.attrib.get('route', '')