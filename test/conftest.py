import os
from os.path import dirname

import pytest

from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.world import World


@pytest.fixture
def apartment_world() -> World:
    """
    Return the apartment world parsed from the URDF file.
    """
    urdf_dir = os.path.join(dirname(__file__), "worlds", "urdf")
    apartment = os.path.join(urdf_dir, "apartment.urdf")
    parser = URDFParser.from_file(file_path=apartment)
    world = parser.parse()
    world.validate()
    return world


@pytest.fixture
def kitchen_world() -> World:
    urdf_dir = os.path.join(dirname(__file__), "worlds", "urdf")
    kitchen = os.path.join(urdf_dir, "kitchen.urdf")
    parser = URDFParser.from_file(file_path=kitchen)
    world = parser.parse()
    world.validate()
    return world
