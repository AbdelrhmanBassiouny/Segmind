from __future__ import annotations

from datetime import timedelta
from threading import RLock
from typing_extensions import List, Type, Optional, TYPE_CHECKING, Dict

import numpy as np

from pycram.datastructures.dataclasses import ObjectState
from pycram.world_concepts.world_object import Object
from pycram.ros.logging import logwarn

if TYPE_CHECKING:
    from .events import Event, EventUnion


class ObjectTracker:

    def __init__(self, obj: Object):
        self.obj = obj
        self._lock: RLock = RLock()
        self._event_history: List[Event] = []

    @property
    def current_state(self) -> ObjectState:
        return self.obj.current_state

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
            first_event_index = self.get_index_of_first_event_before(timestamp)
            return self._event_history[first_event_index] if first_event_index is not None else None

    def get_first_event_after(self, timestamp: float) -> Optional[Event]:
        with self._lock:
            first_event_index = self.get_index_of_first_event_after(timestamp)
            return self._event_history[first_event_index] if first_event_index is not None else None

    def get_nearest_event_of_type_to_event(self, event: Event, event_type: Type[Event],
                                           tolerance: Optional[timedelta] = None) -> Optional[EventUnion]:
        return self.get_nearest_event_of_type_to_timestamp(event.timestamp, event_type, tolerance)

    def get_nearest_event_of_type_to_timestamp(self, timestamp: float, event_type: Type[Event],
                                               tolerance: Optional[timedelta] = None) -> Optional[Event]:
        with self._lock:
            time_stamps = self.time_stamps_array
            type_cond = np.array([isinstance(event, event_type) for event in self._event_history])
            valid_indices = np.where(type_cond)[0]
            if len(valid_indices) > 0:
                time_stamps = time_stamps[valid_indices]
                nearest_event_index = self._get_nearest_index(time_stamps, timestamp, tolerance)
                if nearest_event_index is not None:
                    return self._event_history[valid_indices[nearest_event_index]]

    def get_nearest_event_to(self, timestamp: float, tolerance: Optional[timedelta] = None) -> Optional[Event]:
        with self._lock:
            time_stamps = self.time_stamps_array
            nearest_event_index = self._get_nearest_index(time_stamps, timestamp, tolerance)
            if nearest_event_index is not None:
                return self._event_history[nearest_event_index]

    def _get_nearest_index(self, time_stamps: np.ndarray,
                           timestamp: float, tolerance: Optional[timedelta] = None) -> Optional[int]:
        with self._lock:
            nearest_event_index = np.argmin(np.abs(time_stamps - timestamp))
            if tolerance is not None and abs(time_stamps[nearest_event_index] - timestamp) > tolerance.total_seconds():
                return None
            return nearest_event_index

    def get_first_event_of_type_after_event(self, event_type: Type[Event], event: Event) -> Optional[EventUnion]:
        return self.get_first_event_of_type_after_timestamp(event_type, event.timestamp)

    def get_first_event_of_type_after_timestamp(self, event_type: Type[Event], timestamp: float) -> Optional[Event]:
        with self._lock:
            start_index = self.get_index_of_first_event_after(timestamp)
            if start_index is not None:
                for event in self._event_history[start_index:]:
                    if isinstance(event, event_type):
                        return event

    def get_first_event_of_type_before_event(self, event_type: Type[Event], event: Event) -> Optional[EventUnion]:
        return self.get_first_event_of_type_before_timestamp(event_type, event.timestamp)

    def get_first_event_of_type_before_timestamp(self, event_type: Type[Event], timestamp: float) -> Optional[Event]:
        with self._lock:
            start_index = self.get_index_of_first_event_before(timestamp)
            if start_index is not None:
                for event in reversed(self._event_history[:min(start_index+1, len(self._event_history))]):
                    if isinstance(event, event_type):
                        return event

    def get_index_of_first_event_after(self, timestamp: float) -> Optional[int]:
        with self._lock:
            time_stamps = self.time_stamps_array
            try:
                return np.where(time_stamps > timestamp)[0][0]
            except IndexError:
                logwarn(f"No events after timestamp {timestamp}")
                return None

    def get_index_of_first_event_before(self, timestamp: float) -> Optional[int]:
        with self._lock:
            time_stamps = self.time_stamps_array
            try:
                return np.where(time_stamps < timestamp)[0][-1]
            except IndexError:
                logwarn(f"No events before timestamp {timestamp}")
                return None

    @property
    def time_stamps_array(self) -> np.ndarray:
        return np.array(self.time_stamps)

    @property
    def time_stamps(self) -> List[float]:
        with self._lock:
            return [event.timestamp for event in self._event_history]


class ObjectTrackerFactory:

    _trackers: Dict[Object, ObjectTracker] = {}
    _lock: RLock = RLock()

    @classmethod
    def get_all_trackers(cls) -> List[ObjectTracker]:
        with cls._lock:
            return list(cls._trackers.values())

    @classmethod
    def get_tracker(cls, obj: Object) -> ObjectTracker:
        with cls._lock:
            if obj not in cls._trackers:
                cls._trackers[obj] = ObjectTracker(obj)
            return cls._trackers[obj]


