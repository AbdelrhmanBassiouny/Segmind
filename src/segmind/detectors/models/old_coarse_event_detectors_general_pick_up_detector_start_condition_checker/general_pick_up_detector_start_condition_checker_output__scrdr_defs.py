from typing_extensions import Dict, Optional, Type, Union
from segmind.datastructures.events import Event, InterferenceEvent, LossOfInterferenceEvent, LossOfSupportEvent
from ripple_down_rules.datastructures.case import Case
from segmind.detectors.coarse_event_detectors import GeneralPickUpDetector
from segmind.utils import get_support
from types import NoneType


def conditions_204802191293408881662554642895906550581(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return not isinstance(event, LossOfInterferenceEvent)
    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conclusion_204802191293408881662554642895906550581(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return False
    return general_pick_up_detector_start_condition_checker(**case)


def conditions_161580757619499966579994522098509194470(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        no_bodies_in_contact = len(event.tracked_object.contact_points.get_all_bodies()) == 0
        return no_bodies_in_contact and get_support(event.tracked_object, [event.with_object]) is not None
    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conclusion_161580757619499966579994522098509194470(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return True
    return general_pick_up_detector_start_condition_checker(**case)


def conditions_38026252761470320656686246787772237017(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        has_contact_bodies = len(event.tracked_object.contact_points.get_all_bodies()) > 0
        non_of_the_contact_bodies_is_a_support = get_support(event.tracked_object, event.tracked_object.contact_points.get_all_bodies()) is None
        lost_contact_with_support = get_support(event.tracked_object, [event.with_object]) is not None
        return has_contact_bodies and non_of_the_contact_bodies_is_a_support and lost_contact_with_support
    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conclusion_38026252761470320656686246787772237017(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return True
    return general_pick_up_detector_start_condition_checker(**case)


