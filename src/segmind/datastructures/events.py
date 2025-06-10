import time
from abc import abstractmethod, ABC

from typing_extensions import Optional, List, Union

from pycram.ros import logwarn
from segmind.datastructures.mixins import HasPrimaryTrackedObject, HasSecondaryTrackedObject
from segmind.datastructures.object_tracker import ObjectTrackerFactory
from pycram.datastructures.dataclasses import ContactPointsList, TextAnnotation, ObjectState
from pycram.datastructures.dataclasses import Color
from pycram.datastructures.pose import Pose
from pycram.datastructures.world import World
from pycram.datastructures.world_entity import PhysicalBody
from pycram.world_concepts.world_object import Object, Link


class Event(ABC):

    annotation_size: float = 1
    """
    The size of the annotation text.
    """

    def __init__(self, timestamp: Optional[float] = None):
        self.timestamp = time.time() if timestamp is None else timestamp
        self.text_id: Optional[int] = None
        self.detector_thread_id: Optional[str] = None

    @abstractmethod
    def __eq__(self, other):
        pass

    @abstractmethod
    def __hash__(self):
        pass

    def annotate(self, position: Optional[List[float]] = None, size: Optional[float] = None,
                 color: Optional[Color] = None) -> TextAnnotation:
        """
        Annotates the event with the text from the :meth:`__str__` method using the specified position, size, and color.

        :param position: The position of the annotation text.
        :param size: The size of the annotation text.
        :param color: The color of the annotation text and/or object.
        :return: The TextAnnotation object that references the annotation text.
        """
        position = position if position is not None else [2, 1, 2]
        size = size if size is not None else self.annotation_size
        self.set_color(color)
        self.text_id = World.current_world.add_text(
            self.annotation_text,
            position,
            color=self.color,
            size=size)
        return TextAnnotation(self.annotation_text, position, self.text_id, color=self.color, size=size)

    @abstractmethod
    def set_color(self, color: Optional[Color] = None):
        pass

    @property
    @abstractmethod
    def color(self) -> Color:
        pass

    @property
    def annotation_text(self) -> str:
        return self.__str__()

    @abstractmethod
    def __str__(self):
        pass

    def __repr__(self):
        return self.__str__()


class EventWithTrackedObjects(Event, ABC):
    """
    An abstract class that represents an event that involves one or more tracked objects.
    """
    @property
    @abstractmethod
    def tracked_objects(self) -> List[Object]:
        """
        The tracked objects involved in the event.
        """
        pass

    @property
    @abstractmethod
    def involved_bodies(self) -> List[PhysicalBody]:
        """
        The bodies involved in the event.
        """
        pass

    @abstractmethod
    def update_object_trackers_with_event(self) -> None:
        """
        Update the object trackers of the involved objects with the event.
        """
        pass


class EventWithOneTrackedObject(EventWithTrackedObjects, HasPrimaryTrackedObject, ABC):
    """
    An abstract class that represents an event that involves one tracked object.
    """
    def __init__(self, tracked_object: Object, timestamp: Optional[float] = None):
        EventWithTrackedObjects.__init__(self, timestamp)
        HasPrimaryTrackedObject.__init__(self, tracked_object)

    @property
    def tracked_objects(self) -> List[Object]:
        return [self.tracked_object]

    def update_object_trackers_with_event(self) -> None:
        ObjectTrackerFactory.get_tracker(self.tracked_object).add_event(self)

    def __str__(self):
        return f"{self.__class__.__name__}: {self.tracked_object.name} - {self.timestamp}"

    def __eq__(self, other):
        return (other.__class__ == self.__class__
                and self.tracked_object == other
                and round(self.timestamp, 1) == round(other.timestamp, 1))

    def __hash__(self):
        return hash((self.__class__, self.tracked_object, round(self.timestamp, 1)))


class EventWithTwoTrackedObjects(EventWithOneTrackedObject, HasSecondaryTrackedObject, ABC):
    """
    An abstract class that represents an event that involves two tracked objects.
    """
    def __init__(self, tracked_object: Object, with_object: Optional[Object] = None, timestamp: Optional[float] = None):
        EventWithOneTrackedObject.__init__(self, tracked_object, timestamp)
        HasSecondaryTrackedObject.__init__(self, with_object)

    @property
    def tracked_objects(self) -> List[Object]:
        return [self.tracked_object, self.with_object] if self.with_object is not None else [self.tracked_object]

    def update_object_trackers_with_event(self) -> None:
        ObjectTrackerFactory.get_tracker(self.tracked_object).add_event(self)
        if self.with_object is not None:
            ObjectTrackerFactory.get_tracker(self.with_object).add_event(self)

    def __str__(self):
        with_object_name = f" - {self.with_object.name}" if self.with_object is not None else ""
        return f"{self.__class__.__name__}: {self.tracked_object.name}{with_object_name} - {self.timestamp}"

    def __eq__(self, other):
        return (other.__class__ == self.__class__
                and self.tracked_object == other.tracked_object
                and self.with_object == other.with_object
                and round(self.timestamp, 1) == round(other.timestamp, 1))

    def __hash__(self):
        hash_tuple = (self.__class__, self.tracked_object, round(self.timestamp, 1))
        if self.with_object is not None:
            hash_tuple += (self.with_object,)
        return hash(hash_tuple)


class NewObjectEvent(EventWithOneTrackedObject):
    """
    The NewObjectEvent class is used to represent an event that involves the addition of a new object to the world.
    """

    def __init__(self, new_object: Object, timestamp: Optional[float] = None):
        EventWithOneTrackedObject.__init__(self, new_object, timestamp)

    @property
    def involved_bodies(self) -> List[Object]:
        return self.tracked_objects

    def set_color(self, color: Optional[Color] = None):
        ...

    @property
    def color(self) -> Color:
        return self.tracked_object.color


class MotionEvent(EventWithOneTrackedObject, ABC):
    """
    The MotionEvent class is used to represent an event that involves an object that was stationary and then moved or
    vice versa.
    """

    def __init__(self, tracked_object: Object, start_pose: Pose, current_pose: Pose,
                 timestamp: Optional[float] = None):
        EventWithOneTrackedObject.__init__(self, tracked_object, timestamp)
        self.start_pose: Pose = start_pose
        self.current_pose: Pose = current_pose

    @property
    def involved_bodies(self) -> List[PhysicalBody]:
        return self.tracked_objects

    def set_color(self, color: Optional[Color] = None):
        color = color if color is not None else self.color
        self.tracked_object.set_color(color)


class TranslationEvent(MotionEvent):
    @property
    def color(self) -> Color:
        return Color(0, 1, 1, 1)


class RotationEvent(MotionEvent):
    @property
    def color(self) -> Color:
        return Color(0, 1, 1, 1)


class StopMotionEvent(MotionEvent):
    @property
    def color(self) -> Color:
        return Color(1, 0, 0, 1)


class StopTranslationEvent(StopMotionEvent):
    ...


class StopRotationEvent(StopMotionEvent):
    ...


class AbstractContactEvent(EventWithTwoTrackedObjects, ABC):

    def __init__(self,
                 contact_points: ContactPointsList,
                 of_object: Object,
                 latest_contact_points: Optional[ContactPointsList] = None,
                 with_object: Optional[Object] = None,
                 timestamp: Optional[float] = None):

        EventWithTwoTrackedObjects.__init__(self, of_object, with_object, timestamp)
        self.contact_points = contact_points
        self.latest_contact_points = latest_contact_points

    @property
    def involved_bodies(self) -> List[PhysicalBody]:
        return list(set(self.links))

    def set_color(self, color: Optional[Color] = None):
        color = color if color is not None else self.color
        self.main_link.color = color
        [link.set_color(color) for link in self.links]

    @property
    def object_names(self) -> List[str]:
        return [obj.name for obj in self.objects]

    @property
    def link_names(self) -> List[str]:
        return [link.name for link in self.links]

    @property
    @abstractmethod
    def main_link(self) -> PhysicalBody:
        pass

    @property
    @abstractmethod
    def links(self) -> List[PhysicalBody]:
        pass

    @property
    @abstractmethod
    def objects(self) -> List[Object]:
        pass


class ContactEvent(AbstractContactEvent):

    @property
    def color(self) -> Color:
        return Color(0, 0, 1, 1)

    @property
    def objects(self):
        return self.contact_points.get_objects_that_have_points()

    @property
    def main_link(self):
        if len(self.contact_points) > 0:
            return self.contact_points[0].link_a
        else:
            logwarn(f"No contact points found for {self.tracked_object.name} in {self.__class__.__name__}")

    @property
    def links(self):
        return self.contact_points.get_all_bodies()


class InterferenceEvent(ContactEvent):
    ...


class LossOfContactEvent(AbstractContactEvent):

    @property
    def latest_objects_that_got_removed(self):
        return self.get_objects_that_got_removed(self.latest_contact_points)

    def get_objects_that_got_removed(self, contact_points: ContactPointsList):
        return self.contact_points.get_objects_that_got_removed(contact_points)

    @property
    def color(self) -> Color:
        return Color(1, 0, 0, 1)

    @property
    def main_link(self) -> PhysicalBody:
        return self.latest_contact_points[0].link_a

    @property
    def links(self) -> List[PhysicalBody]:
        return self.contact_points.get_bodies_that_got_removed(self.latest_contact_points)

    @property
    def objects(self):
        return self.contact_points.get_objects_that_got_removed(self.latest_contact_points)


class LossOfInterferenceEvent(LossOfContactEvent):
    ...


class AbstractAgentContact(AbstractContactEvent, ABC):
    @property
    def agent(self) -> Object:
        return self.tracked_object

    @property
    def agent_link(self) -> PhysicalBody:
        return self.main_link

    def with_object_contact_link(self) -> PhysicalBody:
        if self.with_object is not None:
            return [link for link in self.links if link.parent_entity == self.with_object][0]

    @property
    @abstractmethod
    def object_link(self) -> Link:
        pass


class AgentContactEvent(ContactEvent, AbstractAgentContact):

    @property
    def object_link(self) -> PhysicalBody:
        if self.with_object is not None:
            return self.with_object_contact_link()
        else:
            return self.contact_points[0].link_b


class AgentInterferenceEvent(InterferenceEvent, AgentContactEvent):
    ...


class AgentLossOfContactEvent(LossOfContactEvent, AbstractAgentContact):

    @property
    def object_link(self) -> PhysicalBody:
        if self.with_object is not None:
            return self.with_object_contact_link()
        else:
            return self.latest_contact_points[0].link_b


class AgentLossOfInterferenceEvent(LossOfInterferenceEvent, AgentLossOfContactEvent):
    ...


class LossOfSurfaceEvent(LossOfContactEvent):
    def __init__(self, contact_points: ContactPointsList,
                 latest_contact_points: ContactPointsList,
                 of_object: Object,
                 surface: Optional[PhysicalBody] = None,
                 timestamp: Optional[float] = None):
        super().__init__(contact_points, latest_contact_points, of_object, surface, timestamp)
        self.surface: Optional[PhysicalBody] = surface


class AbstractAgentObjectInteractionEvent(EventWithTwoTrackedObjects, ABC):

    def __init__(self, participating_object: Object,
                 agent: Optional[Object] = None,
                 timestamp: Optional[float] = None,
                 end_timestamp: Optional[float] = None):
        EventWithTwoTrackedObjects.__init__(self, participating_object, agent, timestamp)
        self.end_timestamp: Optional[float] = end_timestamp
        self.text_id: Optional[int] = None
        self.agent: Optional[Object] = agent

    @property
    def involved_bodies(self) -> List[PhysicalBody]:
        return self.tracked_objects

    @property
    def agent_state(self) -> Optional[ObjectState]:
        if self.agent is None:
            return None
        return self.agent.state

    def __eq__(self, other):
        if self.end_timestamp is None:
            return super().__eq__(other)
        return (super().__eq__(other)
                and round(self.end_timestamp, 1) == round(other.end_timestamp, 1))
    
    @property
    def hash_tuple(self):
        hash_tuple = (self.__class__, self.agent, self.tracked_object, round(self.timestamp, 1))
        if self.end_timestamp is not None:
            hash_tuple += (round(self.end_timestamp, 1),)
        return hash_tuple

    def __hash__(self):
        return hash(self.hash_tuple)

    def record_end_timestamp(self):
        self.end_timestamp = time.time()

    def duration(self):
        if self.end_timestamp is None:
            return None
        return self.end_timestamp - self.timestamp

    def set_color(self, color: Optional[Color] = None):
        color = color if color is not None else self.color
        if self.agent is not None:
            self.agent.set_color(color)
        self.tracked_object.set_color(color)


class PickUpEvent(AbstractAgentObjectInteractionEvent):

    @property
    def color(self) -> Color:
        return Color(0, 1, 0, 1)


class PlacingEvent(AbstractAgentObjectInteractionEvent):

    @property
    def color(self) -> Color:
        return Color(1, 0, 1, 1)


class InsertionEvent(AbstractAgentObjectInteractionEvent):
    def __init__(self, inserted_object: Object,
                 inserted_into_objects: List[Object],
                 through_hole: PhysicalBody,
                 agent: Optional[Object] = None,
                 timestamp: Optional[float] = None,
                 end_timestamp: Optional[float] = None):
        super().__init__(inserted_object, agent, timestamp, end_timestamp)
        self.inserted_into_objects: List[Object] = inserted_into_objects
        self.through_hole: PhysicalBody = through_hole
        self.with_object: Optional[Object] = through_hole

    def hash_tuple(self):
        hash_tuple = (*super().hash_tuple, *(obj.name for obj in self.inserted_into_objects))
        return hash_tuple

    def __str__(self):
        with_object_name = " - " + f" - ".join([obj.name for obj in self.inserted_into_objects])
        return f"{self.__class__.__name__}: {self.tracked_object.name}{with_object_name} - {self.timestamp}"

    @property
    def color(self) -> Color:
        return Color(1, 0, 1, 1)


class ContainmentEvent(AbstractAgentObjectInteractionEvent):
    def __init__(self, inserted_object: Object,
                 inserted_into_objects: List[Object],
                 agent: Optional[Object] = None,
                 timestamp: Optional[float] = None,
                 end_timestamp: Optional[float] = None):
        super().__init__(inserted_object, agent, timestamp, end_timestamp)
        self.inserted_into_objects: List[Object] = inserted_into_objects

    def hash_tuple(self):
        hash_tuple = (*super().hash_tuple, *(obj.name for obj in self.inserted_into_objects))
        return hash_tuple

    def __str__(self):
        with_object_name = " - " + f" - ".join([obj.name for obj in self.inserted_into_objects])
        return f"{self.__class__.__name__}: {self.tracked_object.name}{with_object_name} - {self.timestamp}"

    @property
    def color(self) -> Color:
        return Color(1, 0, 1, 1)


# Create a type that is the union of all event types
EventUnion = Union[NewObjectEvent,
                   MotionEvent,
                   StopMotionEvent,
                   ContactEvent,
                   LossOfContactEvent,
                   AgentContactEvent,
                   AgentLossOfContactEvent,
                   LossOfSurfaceEvent,
                   PickUpEvent,
                   PlacingEvent]
