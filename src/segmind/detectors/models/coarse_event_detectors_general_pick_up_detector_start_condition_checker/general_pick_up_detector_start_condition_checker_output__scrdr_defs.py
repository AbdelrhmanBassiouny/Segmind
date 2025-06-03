from typing_extensions import Dict, Optional, Type, Union
from ripple_down_rules.datastructures.case import Case
from types import NoneType
from ...coarse_event_detectors import GeneralPickUpDetector, select_transportable_objects
from ....datastructures.events import AgentContactEvent


def conditions_79409294830217498801042528243955850723(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return True
    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conclusion_79409294830217498801042528243955850723(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return isinstance(event, AgentContactEvent) and len(select_transportable_objects(event.objects)) > 0
    return general_pick_up_detector_start_condition_checker(**case)


