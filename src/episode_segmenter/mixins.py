from __future__ import annotations

from abc import ABC

from typing_extensions import List, Optional

from pycram.datastructures.dataclasses import ObjectState
from pycram.world_concepts.world_object import Object


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


class HasOneTrackedObject(ABC):
    """
    A mixin class that provides the tracked object for the event.
    """
    def __init__(self, tracked_object: Object):
        self.tracked_object: Object = tracked_object
        self.tracked_object_state: ObjectState = tracked_object.current_state


class HasTwoTrackedObjects(HasOneTrackedObject, ABC):
    """
    A mixin class that provides the tracked objects for the event.
    """
    def __init__(self, tracked_object: Object, with_object: Optional[Object] = None):
        HasOneTrackedObject.__init__(self, tracked_object)
        self.with_object: Optional[Object] = with_object
        self.with_object_state: Optional[ObjectState] = with_object.current_state if with_object is not None else None
