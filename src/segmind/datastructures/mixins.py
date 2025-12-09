from __future__ import annotations

from dataclasses import dataclass, field
from functools import cached_property

from typing_extensions import List, Optional, TYPE_CHECKING

from .object_tracker import ObjectTrackerFactory, ObjectTracker

from semantic_digital_twin.world_description.world_entity import (
    Body,
    SemanticAnnotation,
)
from semantic_digital_twin.world_description.world_state import WorldState


@dataclass
class HasTrackedObjects:
    """
    A mixin class that provides the tracked object for the event.
    """

    _involved_objects: List[Body]

    @property
    def involved_objects(self) -> List[Body]:
        """
        The objects involved in the event.
        """
        return self._involved_objects


@dataclass(kw_only=True, unsafe_hash=True)
class HasPrimaryTrackedObject:
    """
    A mixin class that provides the tracked object for the event.
    """

    tracked_object: Body
    tracked_object_frozen_cp: Optional[SemanticAnnotation] = field(
        init=False, default=None, repr=False, hash=False
    )
    world_frozen_cp: Optional[WorldState] = field(
        init=False, default=None, repr=False, hash=False
    )

    def __post_init__(self):
        # Create a snapshot of the tracked object using SemanticAnnotation
        self.tracked_object_frozen_cp = SemanticAnnotation()
        self.tracked_object_frozen_cp._bodies = [self.tracked_object]

        # Create a snapshot of the world state
        world_state: WorldState = getattr(self.tracked_object, "world_state", None)
        if world_state is not None:
            # Shallow copy of the numpy data to mimic frozen behavior
            frozen_data = world_state.data.copy()
            self.world_frozen_cp = WorldState()
            self.world_frozen_cp.data = frozen_data
            self.world_frozen_cp.version = world_state.version
            self.world_frozen_cp.state_change_callbacks = list(
                world_state.state_change_callbacks
            )
        else:
            self.world_frozen_cp = None

    @property
    def tracked_object_state(self) -> Body:
        return self.tracked_object

    @cached_property
    def object_tracker(self) -> ObjectTracker:
        return ObjectTrackerFactory.get_tracker(self.tracked_object)


@dataclass(kw_only=True, unsafe_hash=True)
class HasSecondaryTrackedObject:
    """
    A mixin class that provides the tracked objects for the event.
    """

    with_object: Optional[Body] = None
    with_object_frozen_cp: Optional[SemanticAnnotation] = field(
        init=False, default=None, repr=False, hash=False
    )

    def __post_init__(self):
        if self.with_object is not None:
            self.with_object_frozen_cp = SemanticAnnotation()
            self.with_object_frozen_cp._bodies = [self.with_object]
        else:
            self.with_object_frozen_cp = None

    @property
    def with_object_state(self) -> Optional[Body]:
        return self.with_object

    @cached_property
    def with_object_tracker(self) -> Optional[ObjectTracker]:
        return (
            ObjectTrackerFactory.get_tracker(self.with_object)
            if self.with_object is not None
            else None
        )


@dataclass(kw_only=True, unsafe_hash=True)
class HasPrimaryAndSecondaryTrackedObjects(
    HasPrimaryTrackedObject, HasSecondaryTrackedObject
):
    """
    A mixin class that provides the tracked objects for the event.
    """

    def __post_init__(self):
        HasPrimaryTrackedObject.__post_init__(self)
        HasSecondaryTrackedObject.__post_init__(self)
