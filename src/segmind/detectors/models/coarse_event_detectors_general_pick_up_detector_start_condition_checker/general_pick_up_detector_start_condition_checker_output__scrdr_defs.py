from ripple_down_rules.datastructures.case import Case
from segmind.datastructures.events import Event, LossOfSupportEvent
from typing_extensions import Dict, Optional, Type, Union
from types import NoneType
from segmind.detectors.coarse_event_detectors import GeneralPickUpDetector


def conditions_313968436519149281932112885344716145224(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return isinstance(event, LossOfSupportEvent)
    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conditions_228774677018865599164759854498246622113(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return "hole" in event.with_object.name
    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conclusion_228774677018865599164759854498246622113(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return False
    return general_pick_up_detector_start_condition_checker(**case)


def conclusion_313968436519149281932112885344716145224(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return True
    return general_pick_up_detector_start_condition_checker(**case)


