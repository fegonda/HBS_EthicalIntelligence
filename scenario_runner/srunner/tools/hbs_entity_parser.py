import xml.etree.ElementTree as ET

import carla
from srunner.scenarioconfigs.hbs_entity_configuration import EntityConfiguration

class EntityParser(object):

    """
    Pure static class used to parse all the crowd parameter
    """

    def parse( filename, town_name=None ):
        entities = []
        tree = ET.parse(filename)

        for town in tree.iter("town"):
            name = town.attrib['name']
            if town_name and town_name != name:
                continue

            for entity in town.iter('entity'):
                config =EntityConfiguration(entity)
                entities.append( config )

        return entities