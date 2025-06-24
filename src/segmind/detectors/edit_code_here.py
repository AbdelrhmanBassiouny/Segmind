from pycram.plan import Plan
from pycram.designators.action_designator import PickUpAction, PlaceAction
from pycrap.ontologies.crax.classes import Agent, Floor, Location, PhysicalObject, Supporter
from ripple_down_rules.rdr_decorators import RDRDecorator
from typing_extensions import Callable, Dict, List, Optional, Tuple, Type, Union
from pycram.datastructures.world import UseProspectionWorld, World
from pycram.ros.ros1.logging import logdebug, loginfo, logwarn
from abc import ABC, abstractmethod
from datetime import timedelta
from queue import Full, Queue
from segmind.datastructures.mixins import HasPrimaryTrackedObject, HasSecondaryTrackedObject
from segmind.detectors.motion_detection_helpers import DataFilter, ExponentialMovingAverage, has_consistent_direction, is_displaced, is_stopped
from segmind.episode_player import EpisodePlayer
from pycram.tf_transformations import euler_from_quaternion
from pycram.datastructures.dataclasses import AxisAlignedBoundingBox, Color, ContactPointsList, ObjectState, TextAnnotation
from pycram.datastructures.pose import Pose, PoseStamped
from pycram.datastructures.world_entity import PhysicalBody
from pycram.world_concepts.world_object import Object
from pycram.description import ObjectDescription
from segmind.event_logger import EventLogger
from segmind.datastructures.events import AbstractAgentContact, AbstractAgentObjectInteractionEvent, AbstractContactEvent, AgentContactEvent, AgentInterferenceEvent, AgentLossOfContactEvent, AgentLossOfInterferenceEvent, ContactEvent, ContainmentEvent, DefaultEventWithTwoTrackedObjects, Event, EventWithOneTrackedObject, EventWithTrackedObjects, EventWithTwoTrackedObjects, InsertionEvent, InterferenceEvent, LossOfContactEvent, LossOfInterferenceEvent, LossOfSupportEvent, LossOfSurfaceEvent, MotionEvent, NewObjectEvent, PickUpEvent, PlacingEvent, RotationEvent, StopMotionEvent, StopRotationEvent, StopTranslationEvent, SupportEvent, TranslationEvent
from segmind.utils import PropagatingThread, calculate_quaternion_difference, calculate_translation, get_angle_between_vectors, get_support, is_object_supported_by_container_body
from segmind.detectors.atomic_event_detectors import AbstractContactDetector, AtomicEventDetector, ContactDetector, DetectorWithTrackedObject, DetectorWithTwoTrackedObjects, LossOfContactDetector, LossOfSurfaceDetector, MotionDetector, NewObjectDetector, RotationDetector, TranslationDetector
from pycram.datastructures.partial_designator import PartialDesignator
from pycram.designator import ActionDescription, ObjectDesignatorDescription
from segmind.datastructures.object_tracker import ObjectTrackerFactory
from segmind.detectors.coarse_event_detectors import AbstractInteractionDetector, AbstractPickUpDetector, DetectorWithStarterEvent, DetectorWithTrackedObjectAndStarterEvent, GeneralPickUpDetector, PlacingDetector, check_for_supporting_surface, select_transportable_objects, select_transportable_objects_from_contact_event, select_transportable_objects_from_loss_of_contact_event
from ripple_down_rules.datastructures.case import Case
Link = ObjectDescription.Link

def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
    """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
    support = event.with_object
    return support.contains_body(event.tracked_object)