from ripple_down_rules.datastructures.case import Case
from segmind.datastructures.events import AbstractContactEvent, AgentContactEvent, Event, InsertionEvent, LossOfContactEvent, LossOfInterferenceEvent, PickUpEvent, PlacingEvent
from typing_extensions import Dict, Optional, Type, Union
from segmind.detectors.coarse_event_detectors import GeneralPickUpDetector, check_for_supporting_surface, select_transportable_objects
from pycrap.ontologies.crax.classes import Location
from segmind.datastructures.object_tracker import ObjectTrackerFactory
from datetime import timedelta
from segmind.utils import get_support, is_object_supported_by_container_body
from types import NoneType


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
        obj_in_currently_tracked_objects = all(obj in cls_.currently_tracked_objects
                                               and abs(cls_.currently_tracked_objects[obj].starter_event.timestamp
                                                       - event.timestamp) < timedelta(milliseconds=500).total_seconds()
                                               for obj in transportable_objects)
        return obj_in_currently_tracked_objects
    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conclusion_175987223108804549769623056194939396888(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return False
    return general_pick_up_detector_start_condition_checker(**case)


def conditions_137657377818990651652995783903795984105(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return isinstance(event, LossOfInterferenceEvent)
    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conclusion_137657377818990651652995783903795984105(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return False
    return general_pick_up_detector_start_condition_checker(**case)


def conditions_127762420515884983148248561408247495267(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return check_for_supporting_surface(event.tracked_object) is not None
    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conclusion_127762420515884983148248561408247495267(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return False
    return general_pick_up_detector_start_condition_checker(**case)


def conditions_199058987582084612752317307773823561830(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        contact_points = event.tracked_object.contact_points
        return (any(issubclass(obj.obj_type, Location) for obj in contact_points.get_objects_that_have_points())
                or is_object_supported_by_container_body(event.tracked_object))
    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conclusion_199058987582084612752317307773823561830(case) -> bool:
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


