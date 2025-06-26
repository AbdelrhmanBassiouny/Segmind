from __future__ import annotations

from abc import ABC
from dataclasses import dataclass
from functools import cached_property

from typing_extensions import List, Optional, TYPE_CHECKING

from .object_tracker import ObjectTrackerFactory, ObjectTracker
from pycram.world_concepts.world_object import Object

if TYPE_CHECKING:
    from pycram.datastructures.dataclasses import ObjectState


@dataclass
class HasTrackedObjects(ABC):
    """
    A mixin class that provides the tracked object for the event.
    """
    _involved_objects: List[Object]

    @property
    def involved_objects(self) -> List[Object]:
        """
        The objects involved in the event.
        """
        return self._involved_objects


@dataclass(unsafe_hash=True)
class HasPrimaryTrackedObject(ABC):
    """
    A mixin class that provides the tracked object for the event.
    """
    tracked_object: Object

    @property
    def tracked_object_state(self) -> ObjectState:
        return self.tracked_object.current_state

    @cached_property
    def object_tracker(self) -> ObjectTracker:
        return ObjectTrackerFactory.get_tracker(self.tracked_object)


@dataclass
class HasSecondaryTrackedObject(ABC):
    """
    A mixin class that provides the tracked objects for the event.
    """
    with_object: Optional[Object] = None

    @property
    def with_object_state(self) -> Optional[ObjectState]:
        return self.with_object.current_state if self.with_object is not None else None

    @cached_property
    def with_object_tracker(self) -> Optional[ObjectTracker]:
        return ObjectTrackerFactory.get_tracker(self.with_object) if self.with_object is not None else None
