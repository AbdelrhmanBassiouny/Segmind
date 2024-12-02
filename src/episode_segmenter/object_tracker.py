from __future__ import annotations

import numpy as np

from pycram.world_concepts.world_object import Object
from threading import RLock
from typing_extensions import List, Type, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from .events import Event


class ObjectTracker:

    def __init__(self, obj: Object):
        self.obj = obj
        self._lock: RLock = RLock()
        self._event_history: List[Event] = []

    def add_event(self, event: Event):
        with self._lock:
            self._event_history.append(event)

    def get_event_history(self) -> List[Event]:
        with self._lock:
            return self._event_history

    def clear_event_history(self):
        with self._lock:
            self._event_history.clear()

    def get_latest_event(self) -> Optional[Event]:
        with self._lock:
            try:
                return self._event_history[-1]
            except IndexError:
                return None

    def get_latest_event_of_type(self, event_type: Type[Event]) -> Optional[Event]:
        with self._lock:
            for event in reversed(self._event_history):
                if isinstance(event, event_type):
                    return event
            return None

    def get_first_event_before(self, timestamp: float) -> Optional[Event]:
        with self._lock:
            time_stamps = self.time_stamps_array
            try:
                first_event_index = np.where(time_stamps < timestamp)[0][-1]
                return self._event_history[first_event_index]
            except IndexError:
                return None

    def get_first_event_after(self, timestamp: float) -> Optional[Event]:
        with self._lock:
            time_stamps = self.time_stamps_array
            try:
                first_event_index = np.where(time_stamps > timestamp)[0][0]
                return self._event_history[first_event_index]
            except IndexError:
                return None

    def get_nearest_event_to(self, timestamp: float) -> Optional[Event]:
        with self._lock:
            time_stamps = self.time_stamps_array
            nearest_event_index = np.argmin(np.abs(time_stamps - timestamp))
            return self._event_history[nearest_event_index]

    @property
    def time_stamps_array(self) -> np.ndarray:
        return np.array(self.time_stamps)

    @property
    def time_stamps(self) -> List[float]:
        with self._lock:
            return [event.timestamp for event in self._event_history]


