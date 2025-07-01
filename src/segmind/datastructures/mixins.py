from __future__ import annotations

from dataclasses import dataclass, field
from functools import cached_property

from pycram.world_concepts.world_object import Object
from typing_extensions import List, Optional, TYPE_CHECKING

from .object_tracker import ObjectTrackerFactory, ObjectTracker

from pycram.datastructures.dataclasses import ObjectState, FrozenObject, FrozenWorldState


@dataclass
class HasTrackedObjects:
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


@dataclass(kw_only=True, unsafe_hash=True)
class HasPrimaryTrackedObject:
    """
    A mixin class that provides the tracked object for the event.
    """
    tracked_object: Object
    tracked_object_frozen_cp: Optional[FrozenObject] = field(init=False, default=None, repr=False, hash=False)
    world_frozen_cp: Optional[FrozenWorldState] = field(init=False, default=None, repr=False, hash=False)

    def __post_init__(self):
        self.tracked_object_frozen_cp = self.tracked_object.frozen_copy()
        self.world_frozen_cp = self.tracked_object.world.frozen_copy()

    @property
    def tracked_object_state(self) -> ObjectState:
        return self.tracked_object.current_state

    @cached_property
    def object_tracker(self) -> ObjectTracker:
        return ObjectTrackerFactory.get_tracker(self.tracked_object)


@dataclass(kw_only=True, unsafe_hash=True)
class HasSecondaryTrackedObject:
    """
    A mixin class that provides the tracked objects for the event.
    """
    with_object: Optional[Object] = None
    with_object_frozen_cp: Optional[FrozenObject] = field(init=False, default=None, repr=False, hash=False)

    def __post_init__(self):
        if self.with_object is not None:
            self.with_object_frozen_cp = self.with_object.frozen_copy()

    @property
    def with_object_state(self) -> Optional[ObjectState]:
        return self.with_object.current_state if self.with_object is not None else None

    @cached_property
    def with_object_tracker(self) -> Optional[ObjectTracker]:
        return ObjectTrackerFactory.get_tracker(self.with_object) if self.with_object is not None else None


@dataclass(kw_only=True, unsafe_hash=True)
class HasPrimaryAndSecondaryTrackedObjects(HasPrimaryTrackedObject, HasSecondaryTrackedObject):
    """
    A mixin class that provides the tracked objects for the event.
    """

    def __post_init__(self):
        HasPrimaryTrackedObject.__post_init__(self)
        HasSecondaryTrackedObject.__post_init__(self)
