from typing_extensions import Dict, Optional, Union
from types import NoneType
from ....utils import get_support
from ....datastructures.events import AgentContactEvent, LossOfContactEvent, PickUpEvent
from pycram.ros.ros1.logging import logdebug
from ...coarse_event_detectors import GeneralPickUpDetector


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


