from typing_extensions import Callable, Dict, List, Optional, Tuple, Type, Union
from abc import ABC
from pycram.ros.ros2.logging import logdebug, loginfo, logwarn
from segmind.datastructures.mixins import HasPrimaryTrackedObject, HasSecondaryTrackedObject
from segmind.datastructures.object_tracker_SDT import ObjectTracker, ObjectTrackerFactory
from pycram.datastructures.dataclasses import Color, ContactPointsList, ObjectState, TextAnnotation
from pycram.datastructures.pose import Pose
from pycram.datastructures.world import UseProspectionWorld, World
from pycram.datastructures.world_entity import PhysicalBody
from pycram.world_concepts.world_object import Object
from pycram.description import ObjectDescription
from segmind.datastructures.events_SDT import AbstractAgentContact, AbstractAgentObjectInteractionEvent, AbstractContactEvent, AgentContactEvent, AgentInterferenceEvent, AgentLossOfContactEvent, AgentLossOfInterferenceEvent, ContactEvent, ContainmentEvent, Event, EventWithOneTrackedObject, EventWithTrackedObjects, EventWithTwoTrackedObjects, InsertionEvent, InterferenceEvent, LossOfContactEvent, LossOfInterferenceEvent, MotionEvent, NewObjectEvent, PickUpEvent, PlacingEvent, RotationEvent, StopMotionEvent, StopRotationEvent, StopTranslationEvent, TranslationEvent
from pycram.plan import Plan
#from pycrap.ontologies.crax.classes import Agent, Floor, Location, PhysicalObject, Supporter
from ripple_down_rules.rdr_decorators import RDRDecorator
from datetime import timedelta
from queue import Full, Queue
from segmind.detectors.motion_detection_helpers import DataFilter, has_consistent_direction, is_displaced
from segmind.episode_player import EpisodePlayer
from pycram.tf_transformations import euler_from_quaternion
from segmind.event_logger import EventLogger
from segmind.utils import Imaginator, PropagatingThread, calculate_quaternion_difference, calculate_translation, check_if_object_is_supported, get_angle_between_vectors, get_support
from segmind.detectors.atomic_event_detectors_SDT2 import AbstractContactDetector, AtomicEventDetector, ContactDetector, DetectorWithTrackedObject, DetectorWithTwoTrackedObjects, LossOfContactDetector, MotionDetector, NewObjectDetector, RotationDetector, TranslationDetector
from segmind.detectors.coarse_event_detectors_SDT import AbstractInteractionDetector, AbstractPickUpDetector, DetectorWithStarterEvent, DetectorWithTrackedObjectAndStarterEvent, GeneralPickUpDetector, PlacingDetector, check_for_supporting_surface, select_transportable_objects, select_transportable_objects_from_contact_event, select_transportable_objects_from_loss_of_contact_event
from segmind.episode_segmenter_SDT import AgentEpisodeSegmenter, EpisodeSegmenter, NoAgentEpisodeSegmenter
from ripple_down_rules.datastructures.case import Case
from semantic_digital_twin.world_description.world_entity import Body, Agent



def conditions_for_episode_segmenter_is_detector_redundant(self_: NoAgentEpisodeSegmenter, detector_type: Union[Type[ContactDetector], Type[LossOfContactDetector], Type[MotionDetector], Type[TranslationDetector], Type[RotationDetector], Type[NewObjectDetector], Type[PlacingDetector]], starter_event: Union[NewObjectEvent, MotionEvent, StopMotionEvent, ContactEvent, LossOfContactEvent, AgentContactEvent, AgentLossOfContactEvent, PickUpEvent, PlacingEvent], output_: bool) -> bool | None:
    """Get conditions on whether it's possible to conclude a value for EpisodeSegmenter_is_detector_redundant.output_  of type ."""
    pick_up_detectors = [detector for (_,_), detector in self_.starter_event_to_detector_thread_map.items()
     if isinstance(detector, GeneralPickUpDetector)]
    is_being_picked_objects = [detector.tracked_object for detector in pick_up_detectors if detector.is_alive()]
    if starter_event.tracked_object in is_being_picked_objects:
        return True
    object_tracker = ObjectTrackerFactory.get_tracker(starter_event.tracked_object)
    latest_pick_up_event = object_tracker.get_latest_event_of_type(PickUpEvent)
    if latest_pick_up_event is not None:
        latest_placing_event = object_tracker.get_first_event_of_type_after_event(PlacingEvent, latest_pick_up_event)
        latest_insertion_event = object_tracker.get_first_event_of_type_after_event(InsertionEvent, latest_pick_up_event)
        if latest_placing_event is None and latest_insertion_event is None:
            return True
    picked_objects = [detector.tracked_object for detector in pick_up_detectors if not detector.is_alive()]
    if starter_event.tracked_object in picked_objects:
        contact_points_list = getattr(starter_event.tracked_object, "contact_points", None)
        if contact_points_list is None or len(contact_points_list.contacts) == 0:
            return True

        contacted_bodies = contact_points_list.get_all_bodies()
        if len(contacted_bodies) == 0:
            return True

        agents = [body for body in contacted_bodies if
                    (isinstance(body, Body) and isinstance(body.parent_kinematic_structure_entity, Agent)) or
                    (isinstance(body, Body) and issubclass(body.obj_type, Agent))]
        if len(agents) == len(contacted_bodies):
            return True
        return False