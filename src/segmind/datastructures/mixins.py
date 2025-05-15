from __future__ import annotations

from abc import ABC
from functools import cached_property

from typing_extensions import List, Optional, TYPE_CHECKING

from .object_tracker import ObjectTrackerFactory, ObjectTracker
from pycram.world_concepts.world_object import Object

if TYPE_CHECKING:
    from pycram.datastructures.dataclasses import ObjectState


class HasTrackedObjects(ABC):
    """
    A mixin class that provides the tracked object for the event.
    """
    def __init__(self, tracked_objects: List[Object]):
        self._involved_objects = tracked_objects

    @property
    def involved_objects(self) -> List[Object]:
        """
        The objects involved in the event.
        """
        return self._involved_objects


class HasPrimaryTrackedObject(ABC):
    """
    A mixin class that provides the tracked object for the event.
    """
    def __init__(self, tracked_object: Object):
        self.tracked_object: Object = tracked_object
        self.tracked_object_state: ObjectState = tracked_object.current_state

    @cached_property
    def object_tracker(self) -> ObjectTracker:
        return ObjectTrackerFactory.get_tracker(self.tracked_object)


class HasSecondaryTrackedObject(ABC):
    """
    A mixin class that provides the tracked objects for the event.
    """
    def __init__(self, with_object: Optional[Object] = None):
        self.with_object: Optional[Object] = with_object
        self.with_object_state: Optional[ObjectState] = with_object.current_state if with_object is not None else None

    @cached_property
    def with_object_tracker(self) -> Optional[ObjectTracker]:
        return ObjectTrackerFactory.get_tracker(self.with_object) if self.with_object is not None else None
