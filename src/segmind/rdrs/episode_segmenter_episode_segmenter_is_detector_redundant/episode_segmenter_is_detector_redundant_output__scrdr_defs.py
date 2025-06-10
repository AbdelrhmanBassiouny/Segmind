from segmind.datastructures.events import AbstractContactEvent, AgentContactEvent, AgentLossOfContactEvent, ContactEvent, LossOfContactEvent, LossOfSurfaceEvent, MotionEvent, NewObjectEvent, PickUpEvent, PlacingEvent, StopMotionEvent
from segmind.detectors.atomic_event_detectors import ContactDetector, LossOfContactDetector, LossOfSurfaceDetector, MotionDetector, NewObjectDetector, RotationDetector, TranslationDetector
from segmind.detectors.coarse_event_detectors import AbstractInteractionDetector, GeneralPickUpDetector, MotionPickUpDetector, PlacingDetector, select_transportable_objects
from typing_extensions import Dict, Optional, Type, Union
from ripple_down_rules.datastructures.case import Case
from segmind.episode_segmenter import NoAgentEpisodeSegmenter
from datetime import timedelta
from types import NoneType


def conditions_318535409151373315477142163500790537263(case) -> bool:
    def conditions_for_episode_segmenter_is_detector_redundant(self_: NoAgentEpisodeSegmenter, detector_type: Union[Type[ContactDetector], Type[LossOfContactDetector], Type[LossOfSurfaceDetector], Type[MotionDetector], Type[TranslationDetector], Type[RotationDetector], Type[NewObjectDetector], Type[MotionPickUpDetector], Type[PlacingDetector]], starter_event: Union[NewObjectEvent, MotionEvent, StopMotionEvent, ContactEvent, LossOfContactEvent, AgentContactEvent, AgentLossOfContactEvent, LossOfSurfaceEvent, PickUpEvent, PlacingEvent], output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for EpisodeSegmenter_is_detector_redundant.output_  of type ."""
        return issubclass(detector_type, AbstractInteractionDetector)
    return conditions_for_episode_segmenter_is_detector_redundant(**case)


def conclusion_318535409151373315477142163500790537263(case) -> bool:
    def episode_segmenter_is_detector_redundant(self_: NoAgentEpisodeSegmenter, detector_type: Union[Type[ContactDetector], Type[LossOfContactDetector], Type[LossOfSurfaceDetector], Type[MotionDetector], Type[TranslationDetector], Type[RotationDetector], Type[NewObjectDetector], Type[MotionPickUpDetector], Type[PlacingDetector]], starter_event: Union[NewObjectEvent, MotionEvent, StopMotionEvent, ContactEvent, LossOfContactEvent, AgentContactEvent, AgentLossOfContactEvent, LossOfSurfaceEvent, PickUpEvent, PlacingEvent], output_: bool) -> bool:
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


