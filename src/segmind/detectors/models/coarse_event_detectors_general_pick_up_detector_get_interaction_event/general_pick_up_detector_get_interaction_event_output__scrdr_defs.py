from typing_extensions import Dict, Optional, Union
from ...coarse_event_detectors import GeneralPickUpDetector, check_for_supporting_surface
from types import NoneType
from pycram.ros import logdebug
from ....utils import get_support
from ....datastructures.events import AgentContactEvent, LossOfContactEvent, PickUpEvent, PlacingEvent, InsertionEvent


def conditions_87074858769394720739688305292375760638(case) -> bool:
    def conditions_for_general_pick_up_detector_get_interaction_event(self_: GeneralPickUpDetector, output_: Union[PickUpEvent, NoneType]) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_get_interaction_event.output_  of type PickUpEvent."""
        return isinstance(self_.starter_event, AgentContactEvent)
    return conditions_for_general_pick_up_detector_get_interaction_event(**case)


def conclusion_87074858769394720739688305292375760638(case) -> Optional[PickUpEvent]:
    def general_pick_up_detector_get_interaction_event(self_: GeneralPickUpDetector, output_: Union[PickUpEvent, NoneType]) -> Union[PickUpEvent, NoneType]:
        """Get possible value(s) for GeneralPickUpDetector_get_interaction_event.output_  of type PickUpEvent."""
        loss_of_contact_event = self_.check_for_event_post_starter_event(LossOfContactEvent)
        if loss_of_contact_event is None:
            return None
        if self_.starter_event.agent in loss_of_contact_event.objects:
            self_.kill_event.set()
            logdebug(f"Agent lost contact with tracked_object: {self_.tracked_object.name}")
            return None
        surface_object = get_support(self_.tracked_object, loss_of_contact_event.links)
        if surface_object is None:
            return None
        return PickUpEvent(self_.tracked_object, self_.starter_event.agent, timestamp=self_.starter_event.timestamp, end_timestamp=loss_of_contact_event.timestamp)
    return general_pick_up_detector_get_interaction_event(**case)


def conditions_121320552838186729138877771657303489240(case) -> bool:
    def conditions_for_general_pick_up_detector_get_interaction_event(self_: GeneralPickUpDetector, output_: Union[NoneType, PickUpEvent]) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_get_interaction_event.output_  of type PickUpEvent."""
        return isinstance(self_.starter_event, LossOfContactEvent)
    return conditions_for_general_pick_up_detector_get_interaction_event(**case)


def conditions_320416996501934194262144719758568379805(case) -> bool:
    def conditions_for_general_pick_up_detector_get_interaction_event(self_: GeneralPickUpDetector, output_: Union[NoneType, PickUpEvent]) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_get_interaction_event.output_  of type PickUpEvent."""
        latest_pick_up_event = self_.object_tracker.get_latest_event_of_type(PickUpEvent)
        if latest_pick_up_event is not None:
            any_placing_event_after_pick_up_event = self_.object_tracker.get_first_event_of_type_after_event(PlacingEvent, latest_pick_up_event)
            any_insertion_event_after_pick_up_event = self_.object_tracker.get_first_event_of_type_after_event(InsertionEvent, latest_pick_up_event)
            if any_placing_event_after_pick_up_event is None and any_insertion_event_after_pick_up_event is None:
                return True
        return False
    return conditions_for_general_pick_up_detector_get_interaction_event(**case)


def conclusion_320416996501934194262144719758568379805(case) -> Optional[PickUpEvent]:
    def general_pick_up_detector_get_interaction_event(self_: GeneralPickUpDetector, output_: Union[NoneType, PickUpEvent]) -> Union[NoneType, PickUpEvent]:
        """Get possible value(s) for GeneralPickUpDetector_get_interaction_event.output_  of type PickUpEvent."""
        logdebug(f"Stopping duplicate pickup event {self_}")
        self_.currently_tracked_objects.pop(self_.tracked_object)
        self_.kill_event.set()
        return None
    return general_pick_up_detector_get_interaction_event(**case)


def conclusion_121320552838186729138877771657303489240(case) -> Optional[PickUpEvent]:
    def general_pick_up_detector_get_interaction_event(self_: GeneralPickUpDetector, output_: Union[NoneType, PickUpEvent]) -> Union[NoneType, PickUpEvent]:
        """Get possible value(s) for GeneralPickUpDetector_get_interaction_event.output_  of type PickUpEvent."""
        # if get_support(self_.tracked_object) is None:
        return PickUpEvent(self_.tracked_object, timestamp=self_.starter_event.timestamp - self_.wait_time.total_seconds(), end_timestamp=self_.starter_event.timestamp)
        # return None
    return general_pick_up_detector_get_interaction_event(**case)


