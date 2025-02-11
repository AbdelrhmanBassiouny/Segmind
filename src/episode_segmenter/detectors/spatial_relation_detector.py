from typing_extensions import Optional, List

from coarse_event_detectors import DetectorWithStarterEvent
from episode_segmenter.event_logger import EventLogger
from ..datastructures.events import Event, ContactEvent, StopMotionEvent


class SpatialRelationDetector(DetectorWithStarterEvent):
    """
    A class that detects spatial relations between objects.
    """

    @classmethod
    def start_condition_checker(cls, event: Event) -> bool:
        if isinstance(event, (ContactEvent, StopMotionEvent)):
            return True

    def detect_events(self) -> List[Event]:
        pass

    def __str__(self):
        pass

    def detect(self) -> Optional[Event]:
        pass
