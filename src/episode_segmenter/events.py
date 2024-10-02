import time
from abc import abstractmethod, ABC

import rospy
from typing_extensions import Optional, List, Union

from pycram.datastructures.dataclasses import ContactPointsList, Color, TextAnnotation
from pycram.datastructures.pose import Pose
from pycram.datastructures.world import World
from pycram.world_concepts.world_object import Object, Link


class Event(ABC):

    annotation_size: float = 1
    """
    The size of the annotation text.
    """

    def __init__(self, timestamp: Optional[float] = None):
        self.timestamp = time.time() if timestamp is None else timestamp
        self.text_id: Optional[int] = None

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


class MotionEvent(Event):
    """
    The MotionEvent class is used to represent an event that involves an object that was stationary and then moved or
    vice versa.
    """

    def __init__(self, tracked_object: Object, start_pose: Pose, current_pose: Pose,
                 timestamp: Optional[float] = None):
        super().__init__(timestamp)
        self.start_pose: Pose = start_pose
        self.current_pose: Pose = current_pose
        self.tracked_object: Object = tracked_object

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        return (self.tracked_object == other
                and self.start_pose == other.start_pose
                and self.timestamp == other.timestamp)

    def __hash__(self):
        return hash((self.tracked_object, self.start_pose, self.timestamp))

    def set_color(self, color: Optional[Color] = None):
        color = color if color is not None else self.color
        self.tracked_object.set_color(color)

    @property
    def color(self) -> Color:
        return Color(0, 1, 1, 1)

    def __str__(self):
        return f"{self.__class__.__name__}: {self.tracked_object.name}"


class StopMotionEvent(MotionEvent):
    @property
    def color(self) -> Color:
        return Color(1, 1, 1, 1)


class AbstractContactEvent(Event, ABC):

    def __init__(self,
                 contact_points: ContactPointsList,
                 of_object: Object,
                 with_object: Optional[Object] = None,
                 timestamp: Optional[float] = None):
        super().__init__(timestamp)
        self.contact_points = contact_points
        self.of_object: Object = of_object
        self.with_object: Optional[Object] = with_object

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        return self.of_object == other.of_object and self.with_object == other.with_object

    def __hash__(self):
        return hash((self.of_object.name, self.with_object.name if self.with_object is not None else '',
                     self.__class__.__name__))

    def set_color(self, color: Optional[Color] = None):
        color = color if color is not None else self.color
        self.main_link.color = color
        [link.set_color(color) for link in self.links]

    def __str__(self):
        return f"{self.__class__.__name__}: {self.of_object.name} - {self.with_object.name if self.with_object else ''}"

    def __repr__(self):
        return self.__str__()

    @property
    def object_names(self):
        return [obj.name for obj in self.objects]

    @property
    def link_names(self):
        return [link.name for link in self.links]

    @property
    @abstractmethod
    def main_link(self) -> Link:
        pass

    @property
    @abstractmethod
    def links(self) -> List[Link]:
        pass

    @property
    @abstractmethod
    def objects(self):
        pass


class ContactEvent(AbstractContactEvent):

    @property
    def color(self) -> Color:
        return Color(0, 0, 1, 1)

    @property
    def objects(self):
        return self.contact_points.get_objects_that_have_points()

    @property
    def main_link(self) -> Link:
        if len(self.contact_points) > 0:
            return self.contact_points[0].link_a
        else:
            rospy.logwarn(f"No contact points found for {self.of_object.name} in {self.__class__.__name__}")

    @property
    def links(self) -> List[Link]:
        return self.contact_points.get_links_in_contact()


class LossOfContactEvent(AbstractContactEvent):
    def __init__(self, contact_points: ContactPointsList,
                 latest_contact_points: ContactPointsList,
                 of_object: Object,
                 with_object: Optional[Object] = None,
                 timestamp: Optional[float] = None):
        super().__init__(contact_points, of_object, with_object, timestamp)
        self.latest_contact_points = latest_contact_points

    @property
    def latest_objects_that_got_removed(self):
        return self.get_objects_that_got_removed(self.latest_contact_points)

    def get_objects_that_got_removed(self, contact_points: ContactPointsList):
        return self.contact_points.get_objects_that_got_removed(contact_points)

    @property
    def color(self) -> Color:
        return Color(1, 0, 0, 1)

    @property
    def main_link(self) -> Link:
        return self.latest_contact_points[0].link_a

    @property
    def links(self) -> List[Link]:
        return self.contact_points.get_links_that_got_removed(self.latest_contact_points)

    @property
    def objects(self):
        return self.contact_points.get_objects_that_got_removed(self.latest_contact_points)


class AbstractAgentContact(AbstractContactEvent, ABC):
    @property
    def agent(self) -> Object:
        return self.of_object

    @property
    def agent_link(self) -> Link:
        return self.main_link

    def with_object_contact_link(self) -> Link:
        if self.with_object is not None:
            return [link for link in self.links if link.object == self.with_object][0]

    @property
    @abstractmethod
    def object_link(self) -> Link:
        pass


class AgentContactEvent(ContactEvent, AbstractAgentContact):

    @property
    def object_link(self) -> Link:
        if self.with_object is not None:
            return self.with_object_contact_link()
        else:
            return self.contact_points[0].link_b


class AgentLossOfContactEvent(LossOfContactEvent, AbstractAgentContact):

    @property
    def object_link(self) -> Link:
        if self.with_object is not None:
            return self.with_object_contact_link()
        else:
            return self.latest_contact_points[0].link_b


class LossOfSurfaceEvent(LossOfContactEvent):
    def __init__(self, contact_points: ContactPointsList,
                 latest_contact_points: ContactPointsList,
                 of_object: Object,
                 surface: Optional[Object] = None,
                 timestamp: Optional[float] = None):
        super().__init__(contact_points, latest_contact_points, of_object, surface, timestamp)
        self.surface: Optional[Object] = surface


class PickUpEvent(Event):

    def __init__(self, picked_object: Object,
                 agent: Optional[Object] = None,
                 timestamp: Optional[float] = None):
        super().__init__(timestamp)
        self.agent: Optional[Object] = agent
        self.picked_object: Object = picked_object
        self.end_timestamp: Optional[float] = None
        self.text_id: Optional[int] = None

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        return self.agent == other.agent and self.picked_object == other.picked_object

    def __hash__(self):
        return hash((self.agent, self.picked_object, self.__class__))

    def record_end_timestamp(self):
        self.end_timestamp = time.time()

    def duration(self):
        if self.end_timestamp is None:
            return None
        return self.end_timestamp - self.timestamp

    def set_color(self, color: Optional[Color] = None):
        color = color if color is not None else self.color
        self.agent.set_color(color)
        self.picked_object.set_color(color)

    @property
    def color(self) -> Color:
        return Color(0, 1, 0, 1)

    def __str__(self):
        return f"Pick up event: Agent:{self.agent.name}, Object: {self.picked_object.name}, Timestamp: {self.timestamp}"

    def __repr__(self):
        return self.__str__()


# Create a type that is the union of all event types
EventUnion = Union[MotionEvent,
                   StopMotionEvent,
                   ContactEvent,
                   LossOfContactEvent,
                   AgentContactEvent,
                   AgentLossOfContactEvent,
                   LossOfSurfaceEvent,
                   PickUpEvent]
