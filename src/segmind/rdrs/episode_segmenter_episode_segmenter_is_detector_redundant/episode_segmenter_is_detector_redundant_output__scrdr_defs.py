from pycram.description import Link
from segmind.detectors.coarse_event_detectors import AbstractInteractionDetector, GeneralPickUpDetector, PlacingDetector, select_transportable_objects
from types import NoneType
from segmind.datastructures.events import AbstractContactEvent, AgentContactEvent, AgentLossOfContactEvent, \
    ContactEvent, InsertionEvent, LossOfContactEvent, LossOfSurfaceEvent, MotionEvent, NewObjectEvent, PickUpEvent, \
    PlacingEvent, StopMotionEvent, EventWithTrackedObjects, EventWithOneTrackedObject
from segmind.detectors.atomic_event_detectors import ContactDetector, LossOfContactDetector, LossOfSurfaceDetector, MotionDetector, NewObjectDetector, RotationDetector, TranslationDetector
from pycrap.ontologies.crax.classes import Agent
from pycram.world_concepts.world_object import Object
from datetime import timedelta
from typing_extensions import Dict, Optional, Type, Union
from segmind.datastructures.object_tracker import ObjectTrackerFactory
from segmind.episode_segmenter import NoAgentEpisodeSegmenter


def conditions_318535409151373315477142163500790537263(case) -> bool:
    def conditions_for_episode_segmenter_is_detector_redundant(self_: NoAgentEpisodeSegmenter, detector_type: Union[Type[ContactDetector], Type[LossOfContactDetector], Type[LossOfSurfaceDetector], Type[MotionDetector], Type[TranslationDetector], Type[RotationDetector], Type[NewObjectDetector], Type[PlacingDetector]], starter_event: Union[NewObjectEvent, MotionEvent, StopMotionEvent, ContactEvent, LossOfContactEvent, AgentContactEvent, AgentLossOfContactEvent, LossOfSurfaceEvent, PickUpEvent, PlacingEvent], output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for EpisodeSegmenter_is_detector_redundant.output_  of type ."""
        return False
        return issubclass(detector_type, AbstractInteractionDetector)
    return conditions_for_episode_segmenter_is_detector_redundant(**case)


def conditions_217503528191875472672592688900935027547(case) -> bool:
    def conditions_for_episode_segmenter_is_detector_redundant(self_: NoAgentEpisodeSegmenter, detector_type: Union[Type[ContactDetector], Type[LossOfContactDetector], Type[LossOfSurfaceDetector], Type[MotionDetector], Type[TranslationDetector], Type[RotationDetector], Type[NewObjectDetector], Type[PlacingDetector]], starter_event: Union[NewObjectEvent, MotionEvent, StopMotionEvent, ContactEvent, LossOfContactEvent, AgentContactEvent, AgentLossOfContactEvent, LossOfSurfaceEvent, PickUpEvent, PlacingEvent], output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for EpisodeSegmenter_is_detector_redundant.output_  of type ."""
        return False
        if not issubclass(detector_type, GeneralPickUpDetector):
            return False
        if not isinstance(starter_event, EventWithOneTrackedObject):
            return False
        pick_up_detetectors = [detector for (_,_), detector in self_.starter_event_to_detector_thread_map.items()
         if isinstance(detector, GeneralPickUpDetector)]
        is_being_picked_objects = [detector.tracked_object for detector in pick_up_detetectors if detector.is_alive()]
        if starter_event.tracked_object in is_being_picked_objects:
            return True
        # object_tracker = ObjectTrackerFactory.get_tracker(starter_event.tracked_object)
        # latest_pick_up_event = object_tracker.get_latest_event_of_type(PickUpEvent)
        # if latest_pick_up_event is not None:
        #     latest_placing_event = object_tracker.get_first_event_of_type_after_event(PlacingEvent, latest_pick_up_event)
        #     latest_insertion_event = object_tracker.get_first_event_of_type_after_event(InsertionEvent, latest_pick_up_event)
        #     if latest_placing_event is None and latest_insertion_event is None:
        #         return True
        picked_objects = [detector.tracked_object for detector in pick_up_detetectors if not detector.is_alive()]
        if starter_event.tracked_object in picked_objects:
            contacted_bodies = starter_event.tracked_object.contact_points.get_all_bodies()
            if len(contacted_bodies) == 0:
                return True
            agents = [body for body in contacted_bodies if (isinstance(body, Link) and issubclass(body.parent_entity.obj_type, Agent)) or
             (isinstance(body, Object) and issubclass(body.obj_type, Agent))]
            if len(agents) == len(contacted_bodies):
                return True
        return False
    return conditions_for_episode_segmenter_is_detector_redundant(**case)


def conclusion_217503528191875472672592688900935027547(case) -> bool:
    def episode_segmenter_is_detector_redundant(self_: NoAgentEpisodeSegmenter, detector_type: Union[Type[ContactDetector], Type[LossOfContactDetector], Type[LossOfSurfaceDetector], Type[MotionDetector], Type[TranslationDetector], Type[RotationDetector], Type[NewObjectDetector], Type[PlacingDetector]], starter_event: Union[NewObjectEvent, MotionEvent, StopMotionEvent, ContactEvent, LossOfContactEvent, AgentContactEvent, AgentLossOfContactEvent, LossOfSurfaceEvent, PickUpEvent, PlacingEvent], output_: bool) -> bool:
        """Get possible value(s) for EpisodeSegmenter_is_detector_redundant.output_  of type ."""
        return True
    return episode_segmenter_is_detector_redundant(**case)


def conclusion_318535409151373315477142163500790537263(case) -> bool:
    def episode_segmenter_is_detector_redundant(self_: NoAgentEpisodeSegmenter, detector_type: Union[Type[ContactDetector], Type[LossOfContactDetector], Type[LossOfSurfaceDetector], Type[MotionDetector], Type[TranslationDetector], Type[RotationDetector], Type[NewObjectDetector], Type[PlacingDetector]], starter_event: Union[NewObjectEvent, MotionEvent, StopMotionEvent, ContactEvent, LossOfContactEvent, AgentContactEvent, AgentLossOfContactEvent, LossOfSurfaceEvent, PickUpEvent, PlacingEvent], output_: bool) -> bool:
        """Get possible value(s) for EpisodeSegmenter_is_detector_redundant.output_  of type ."""
        similar_detectors = [(se, de, de_inst) for (se, de), de_inst in self_.starter_event_to_detector_thread_map.items()
        if isinstance(se, starter_event.__class__) and issubclass(de, detector_type)]
    
        for prev_start_event, _, prev_detector in similar_detectors:
            candidate_objects = starter_event.tracked_objects
            if isinstance(starter_event, AbstractContactEvent):
                candidate_objects.extend(starter_event.objects)
            pickable_objects = select_transportable_objects(candidate_objects)
            new_pickable_objects = [obj for obj in pickable_objects if obj in prev_detector.currently_tracked_objects]
            if len(new_pickable_objects) > 0:
                return False
            common_pickable_objects = [obj for obj in pickable_objects if obj not in new_pickable_objects]
            for obj in common_pickable_objects:
                if starter_event.timestamp - prev_start_event.timestamp < timedelta(milliseconds=200).total_seconds():
                    return True
        return False
    return episode_segmenter_is_detector_redundant(**case)


