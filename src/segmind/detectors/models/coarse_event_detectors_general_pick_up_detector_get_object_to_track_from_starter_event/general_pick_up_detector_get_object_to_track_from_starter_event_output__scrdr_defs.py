from pycram.world_concepts.world_object import Object
from ripple_down_rules.datastructures.case import Case
from segmind.datastructures.events import AgentContactEvent, AgentLossOfContactEvent, ContactEvent, LossOfContactEvent, LossOfSurfaceEvent, MotionEvent, NewObjectEvent, PickUpEvent, PlacingEvent, StopMotionEvent
from segmind.detectors.coarse_event_detectors import GeneralPickUpDetector, select_transportable_objects
from typing import Dict, Type, Union


def conditions_257779009639527613634091190332753064691(case):
    def conditions_for_general_pick_up_detector_get_object_to_track_from_starter_event(cls_: Type[GeneralPickUpDetector], starter_event: Union[NewObjectEvent, MotionEvent, StopMotionEvent, ContactEvent, LossOfContactEvent, AgentContactEvent, AgentLossOfContactEvent, LossOfSurfaceEvent, PickUpEvent, PlacingEvent], output_: Object) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_get_object_to_track_from_starter_event.output_  of type Object."""
        return isinstance(starter_event, ContactEvent)
    return conditions_for_general_pick_up_detector_get_object_to_track_from_starter_event(**case)


def conclusion_257779009639527613634091190332753064691(case):
    def general_pick_up_detector_get_object_to_track_from_starter_event(cls_: Type[GeneralPickUpDetector], starter_event: Union[NewObjectEvent, MotionEvent, StopMotionEvent, ContactEvent, LossOfContactEvent, AgentContactEvent, AgentLossOfContactEvent, LossOfSurfaceEvent, PickUpEvent, PlacingEvent], output_: Object) -> Object:
        """Get possible value(s) for GeneralPickUpDetector_get_object_to_track_from_starter_event.output_  of type Object."""
        objects = select_transportable_objects(starter_event.objects)
        return objects[0] if len(objects) > 0 else None
    return general_pick_up_detector_get_object_to_track_from_starter_event(**case)


