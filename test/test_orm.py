import logging
import os
import sys
from dataclasses import dataclass
from os.path import dirname
from typing import Optional

from ormatic.ormatic import logger, ORMatic
from ormatic.utils import recursive_subclasses
from sqlacodegen.generators import TablesGenerator
from sqlalchemy import create_engine
from sqlalchemy.orm import registry, Session

# ----------------------------------------------------------------------------------------------------------------------
# This script generates the ORM classes for the segmind package.
# Dataclasses can be mapped automatically to the ORM model
# using the ORMatic library, they just have to be registered in the classes list.
# Classes that are self_mapped and explicitly_mapped are already mapped in the model.py file. Look there for more
# information on how to map them.
# ----------------------------------------------------------------------------------------------------------------------

@dataclass
class ParentMappedClass:
    a: int
    b: Optional[int] = None

@dataclass
class ChildMappedClass(ParentMappedClass):
    c: Optional[int] = None

@dataclass
class ChildNotMappedClass(ParentMappedClass):
    d: Optional[int] = None


def generate_orm(classes):
    """
    Generate the ORM classes for the pycram package.
    """
    # Set up logging
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))

    logger.addHandler(handler)
    logger.setLevel(logging.INFO)

    # Create an ORMatic object with the classes to be mapped
    ormatic = ORMatic(list(classes))

    # Generate the ORM classes
    ormatic.make_all_tables()

    with open(os.path.join(dirname(__file__), '../src/segmind/orm/ormatic_interface.py'), 'w') as f:
        ormatic.to_sqlalchemy_file(f)


def test_generate_orm():

    import pycram
    from ormatic.utils import classes_of_module, recursive_subclasses
    from pycram.datastructures import pose
    from pycram.datastructures.dataclasses import FrozenObject, RayResult, MultiverseRayResult, MultiverseContactPoint, \
        ReasoningResult, \
        MultiverseMetaData, VirtualMobileBaseJoints, Rotations, TextAnnotation, VirtualJoint, ContactPointsList, \
        ClosestPointsList, State, CollisionCallbacks, MultiBody, Colors, ManipulatorData

    from segmind.datastructures import events, mixins
    from segmind.datastructures.events import InsertionEvent
    from segmind.detectors.atomic_event_detectors import AtomicEventDetector

    # create set of classes that should be mapped
    classes = set()
    # classes |= set(recursive_subclasses(ORMaticExplicitMapping))
    classes |= set(classes_of_module(events)) # - {InsertionEvent}
    # classes |= set(classes_of_module(mixins))
    pycram_dataclasses = set(classes_of_module(pycram.datastructures.dataclasses))
    pycram_dataclasses -= {RayResult, MultiverseRayResult, MultiverseContactPoint, ReasoningResult, MultiverseMetaData,
                           VirtualMobileBaseJoints, Rotations, TextAnnotation, VirtualJoint,
                           MultiBody, CollisionCallbacks, Colors, ManipulatorData}
    pycram_dataclasses -= set(recursive_subclasses(State)) | {State}
    classes |= pycram_dataclasses
    # classes |= {pycram.has_parameters.HasParameters}
    classes |= set(classes_of_module(pose))
    classes -= set(recursive_subclasses(AtomicEventDetector)) | {AtomicEventDetector}
    generate_orm(classes)



def test_generate_rm_with_multiple_inheritance():
    # This will Succeed
    child_not_mapped = ChildNotMappedClass(a=1, b=2 , d=3)
    assert child_not_mapped.a == 1
    assert child_not_mapped.b == 2
    assert child_not_mapped.d == 3
    # create set of classes that should be mapped
    classes = set()
    classes |= {ParentMappedClass, ChildMappedClass}
    generate_orm(classes)
    child_mapped = ChildMappedClass(a=1, b=2, c=3)
    assert child_mapped.a == 1
    assert child_mapped.b == 2
    assert child_mapped.c == 3
    # This will Fail
    child_not_mapped = ChildNotMappedClass(a=1, b=2, d=3)
    assert child_not_mapped.a == 1
    assert child_not_mapped.b == 2
    assert child_not_mapped.d == 3
