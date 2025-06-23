from typing_extensions import Dict, Optional, Type, Union
from segmind.datastructures.events import Event, EventWithOneTrackedObject, InterferenceEvent, MotionEvent, StopTranslationEvent, TranslationEvent
from ripple_down_rules.datastructures.case import Case
from pycram.object_descriptors.urdf import ObjectDescription
from types import NoneType
from segmind.utils import get_support
from segmind.detectors.coarse_event_detectors import PlacingDetector


def conditions_213633301887999429662908232055793196406(case) -> bool:
    def conditions_for_placing_detector_start_condition_checker(cls_: Type[PlacingDetector], event: EventWithOneTrackedObject, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for PlacingDetector_start_condition_checker.output_  of type ."""
        return not isinstance(event, StopTranslationEvent)
    return conditions_for_placing_detector_start_condition_checker(**case)


def conclusion_213633301887999429662908232055793196406(case) -> bool:
    def placing_detector_start_condition_checker(cls_: Type[PlacingDetector], event: EventWithOneTrackedObject, output_: bool) -> bool:
        """Get possible value(s) for PlacingDetector_start_condition_checker.output_  of type ."""
        return False
    return placing_detector_start_condition_checker(**case)


def conditions_324552974989252417774318641412101963311(case) -> bool:
    def conditions_for_placing_detector_start_condition_checker(cls_: Type[PlacingDetector], event: EventWithOneTrackedObject, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for PlacingDetector_start_condition_checker.output_  of type ."""
        return get_support(event.tracked_object) is not None
    return conditions_for_placing_detector_start_condition_checker(**case)


def conditions_200496988155257780428139539183300226414(case) -> bool:
    def conditions_for_placing_detector_start_condition_checker(cls_: Type[PlacingDetector], event: EventWithOneTrackedObject, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for PlacingDetector_start_condition_checker.output_  of type ."""
        latest_translation_event = event.object_tracker.get_first_event_of_type_before_event(TranslationEvent, event)
        support = get_support(event.tracked_object)
    
        def conditions(event_to_check: Event) -> bool:
            return (isinstance(event_to_check, InterferenceEvent)
                    and event_to_check.timestamp > latest_translation_event.timestamp
                    and support is event_to_check.with_object
                    and not any(isinstance(e, MotionEvent) for e in event_to_check.object_tracker.get_events_between_two_events(event_to_check, event)))
    
        support_interference_event = event.object_tracker.get_nearest_event_to_event_with_conditions(event, conditions)
    
        if support_interference_event is None:
            return True  # i.e. it is not a placing event
        else:
            return False  # i.e. it is a placing event
    
    return conditions_for_placing_detector_start_condition_checker(**case)


def conclusion_200496988155257780428139539183300226414(case) -> bool:
    def placing_detector_start_condition_checker(cls_: Type[PlacingDetector], event: EventWithOneTrackedObject, output_: bool) -> bool:
        """Get possible value(s) for PlacingDetector_start_condition_checker.output_  of type ."""
        return False
    return placing_detector_start_condition_checker(**case)


def conditions_308324129151228864976597656067893575149(case) -> bool:
    def conditions_for_placing_detector_start_condition_checker(cls_: Type[PlacingDetector], event: EventWithOneTrackedObject, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for PlacingDetector_start_condition_checker.output_  of type ."""
        return len([b for b in event.tracked_object.contact_points.get_all_bodies() if 'hole' in b.name]) > 0
    return conditions_for_placing_detector_start_condition_checker(**case)


def conclusion_308324129151228864976597656067893575149(case) -> bool:
    def placing_detector_start_condition_checker(cls_: Type[PlacingDetector], event: EventWithOneTrackedObject, output_: bool) -> bool:
        """Get possible value(s) for PlacingDetector_start_condition_checker.output_  of type ."""
        return False
    return placing_detector_start_condition_checker(**case)


def conclusion_324552974989252417774318641412101963311(case) -> bool:
    def placing_detector_start_condition_checker(cls_: Type[PlacingDetector], event: EventWithOneTrackedObject, output_: bool) -> bool:
        """Get possible value(s) for PlacingDetector_start_condition_checker.output_  of type ."""
        return True
    return placing_detector_start_condition_checker(**case)


