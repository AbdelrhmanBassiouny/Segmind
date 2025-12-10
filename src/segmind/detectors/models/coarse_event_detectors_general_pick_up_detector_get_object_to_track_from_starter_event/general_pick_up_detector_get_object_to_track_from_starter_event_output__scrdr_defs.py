from typing_extensions import Dict, Optional, Type, Union
from segmind.datastructures.events import (
    AbstractContactEvent,
    AgentContactEvent,
    AgentLossOfContactEvent,
    ContactEvent,
    LossOfContactEvent,
    LossOfInterferenceEvent,
    MotionEvent,
    NewObjectEvent,
    PickUpEvent,
    PlacingEvent,
    StopMotionEvent,
    EventWithOneTrackedObject,
)
from ripple_down_rules.datastructures.case import Case

from segmind.datastructures.mixins import HasPrimaryTrackedObject
from segmind.detectors.coarse_event_detectors import GeneralPickUpDetector
from types import NoneType
from semantic_digital_twin.world_description.world_entity import Body


def conditions_132752200528268219213179872006260451656(case) -> bool:
    def conditions_for_general_pick_up_detector_get_object_to_track_from_starter_event(
        cls_: Type[GeneralPickUpDetector],
        starter_event: Union[
            NewObjectEvent,
            MotionEvent,
            StopMotionEvent,
            ContactEvent,
            LossOfContactEvent,
            AgentContactEvent,
            AgentLossOfContactEvent,
            PickUpEvent,
            PlacingEvent,
        ],
        output_: Union[NoneType, Body],
    ) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_get_object_to_track_from_starter_event.output_  of type Object."""
        return isinstance(starter_event, HasPrimaryTrackedObject)

    return (
        conditions_for_general_pick_up_detector_get_object_to_track_from_starter_event(
            **case
        )
    )


def conclusion_132752200528268219213179872006260451656(case) -> Optional[Body]:
    def general_pick_up_detector_get_object_to_track_from_starter_event(
        cls_: Type[GeneralPickUpDetector],
        starter_event: Union[
            NewObjectEvent,
            MotionEvent,
            StopMotionEvent,
            ContactEvent,
            LossOfContactEvent,
            AgentContactEvent,
            AgentLossOfContactEvent,
            PickUpEvent,
            PlacingEvent,
        ],
        output_: Union[NoneType, Body],
    ) -> Union[NoneType, Body]:
        """Get possible value(s) for GeneralPickUpDetector_get_object_to_track_from_starter_event.output_  of type Object."""
        return starter_event.tracked_object

    return general_pick_up_detector_get_object_to_track_from_starter_event(**case)
