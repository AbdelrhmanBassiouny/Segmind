from typing_extensions import Dict, Optional, Type, Union
from ...coarse_event_detectors import GeneralPickUpDetector, select_transportable_objects
from ....datastructures.events import AbstractContactEvent, AgentContactEvent, Event, LossOfContactEvent
from types import NoneType
from ....utils import get_support
from datetime import timedelta


def conditions_79409294830217498801042528243955850723(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return True
    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conditions_8274239634455277150877461420723135533(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return isinstance(event, LossOfContactEvent)
    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conditions_175987223108804549769623056194939396888(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        all_objects = event.tracked_objects
        if isinstance(event, AbstractContactEvent):
            all_objects.extend(event.objects)
        transportable_objects = select_transportable_objects(all_objects)
        if len(transportable_objects) == 0:
            return False
        return all(obj in cls_.currently_tracked_objects and abs(cls_.currently_tracked_objects[obj].starter_event.timestamp - event.timestamp) < timedelta(milliseconds=100).total_seconds()\
                   for obj in transportable_objects)
    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conclusion_175987223108804549769623056194939396888(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return False
    return general_pick_up_detector_start_condition_checker(**case)


def conclusion_8274239634455277150877461420723135533(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        transportable_objects = select_transportable_objects(event.objects + [event.tracked_object])
        for obj in transportable_objects:
            support = get_support(obj, event.links)
            if support is not None:
                return True
        return False
    return general_pick_up_detector_start_condition_checker(**case)


def conclusion_79409294830217498801042528243955850723(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return isinstance(event, AgentContactEvent) and len(select_transportable_objects(event.objects)) > 0
    return general_pick_up_detector_start_condition_checker(**case)


