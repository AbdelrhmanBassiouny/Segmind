from pycram.plan import Plan
from pycram.designators.action_designator import PickUpAction, PlaceAction
from pycrap.ontologies.crax.classes import Agent, Floor, Location, PhysicalObject, Supporter
from ripple_down_rules.rdr_decorators import RDRDecorator
from typing_extensions import Callable, Dict, List, Optional, Tuple, Type, Union
from pycram.datastructures.world import UseProspectionWorld, World
from pycram.ros.ros2.logging import logdebug, loginfo, logwarn
from abc import ABC
from datetime import timedelta
from queue import Full, Queue
from segmind.datastructures.mixins import HasPrimaryTrackedObject, HasSecondaryTrackedObject
from segmind.episode_player import EpisodePlayer
from segmind.detectors.motion_detection_helpers import DataFilter, LowPassFilter, has_consistent_direction, is_displaced
from pycram.tf_transformations import euler_from_quaternion
from pycram.datastructures.dataclasses import Color, ContactPointsList, ObjectState, TextAnnotation
from pycram.datastructures.pose import Pose
from pycram.datastructures.world_entity import PhysicalBody
from pycram.world_concepts.world_object import Object
from segmind.event_logger import EventLogger
from segmind.datastructures.events import AbstractAgentContact, AbstractAgentObjectInteractionEvent, AbstractContactEvent, AgentContactEvent, AgentLossOfContactEvent, ContactEvent, Event, EventWithOneTrackedObject, EventWithTrackedObjects, EventWithTwoTrackedObjects, InsertionEvent, LossOfContactEvent, LossOfSurfaceEvent, MotionEvent, NewObjectEvent, PickUpEvent, PlacingEvent, RotationEvent, StopMotionEvent, StopRotationEvent, StopTranslationEvent, TranslationEvent
from segmind.utils import PropagatingThread, calculate_quaternion_difference, calculate_translation, get_angle_between_vectors, get_support
from segmind.detectors.atomic_event_detectors import AbstractContactDetector, AtomicEventDetector, ContactDetector, DetectorWithTrackedObject, DetectorWithTwoTrackedObjects, LossOfContactDetector, LossOfSurfaceDetector, MotionDetector, NewObjectDetector, RotationDetector, TranslationDetector
from segmind.datastructures.object_tracker import ObjectTrackerFactory
from pycram.description import ObjectDescription
from segmind.detectors.coarse_event_detectors import AbstractInteractionDetector, AbstractPickUpDetector, DetectorWithStarterEvent, DetectorWithTrackedObjectAndStarterEvent, GeneralPickUpDetector, MotionPickUpDetector, PlacingDetector, check_for_supporting_surface, select_transportable_objects, select_transportable_objects_from_contact_event, select_transportable_objects_from_loss_of_contact_event
from ripple_down_rules.datastructures.case import Case
Link = ObjectDescription.Link

def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
    """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
    if isinstance(event, AbstractContactEvent):
        pickable_objects = select_transportable_objects(event.objects + event.tracked_objects)
    else:
        pickable_objects = select_transportable_objects(event.tracked_objects)
    currently_tracked_objects = cls_.currently_tracked_objects
    skip = []
    for obj in pickable_objects:
        if obj in currently_tracked_objects and abs(currently_tracked_objects[obj].starter_event.timestamp - event.timestamp) < timedelta(milliseconds=100).total_seconds():
            skip.append(obj)
    if len(skip) == len(pickable_objects):
        return True
    return False
