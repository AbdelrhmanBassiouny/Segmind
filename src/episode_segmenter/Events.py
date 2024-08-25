import queue
import threading
import time
from abc import abstractmethod, ABC

from typing_extensions import Optional, List, Dict

from pycram.datastructures.dataclasses import ContactPointsList, Color, TextAnnotation
from pycram.datastructures.world import World
from pycram.world_concepts.world_object import Object, Link


class Event(ABC):
    def __init__(self, timestamp: Optional[float] = None):
        self.timestamp = time.time() if timestamp is None else timestamp

    @abstractmethod
    def __eq__(self, other):
        pass

    @abstractmethod
    def __hash__(self):
        pass


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
        self.text_id: Optional[int] = None

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        return self.of_object == other.of_object and self.with_object == other.with_object

    def __hash__(self):
        return hash((self.of_object, self.with_object, self.__class__))

    def annotate(self, position: Optional[List[float]] = None, size: Optional[float] = 2) -> TextAnnotation:
        if position is None:
            position = [2, 1, 2]
        self.main_link.color = self.color
        [link.set_color(self.color) for link in self.links]
        self.text_id = World.current_world.add_text(
            self.annotation_text,
            position,
            color=self.color,
            size=size)
        return TextAnnotation(self.annotation_text, position, self.text_id, color=self.color, size=size)

    @property
    @abstractmethod
    def color(self) -> Color:
        pass

    @property
    def annotation_text(self) -> str:
        return f"{self.main_link.name} lost contact with {self.link_names}"

    @property
    @abstractmethod
    def object_names(self):
        pass

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
    def object_names(self):
        return self.contact_points.get_names_of_objects_that_have_points()

    @property
    def objects(self):
        return self.contact_points.get_objects_that_have_points()

    @property
    def main_link(self) -> Link:
        return self.contact_points[0].link_a

    @property
    def links(self) -> List[Link]:
        links = []
        for obj in self.objects:
            links.extend(self.contact_points.get_links_in_contact_of_object(obj))
        return links

    def check_if_two_objects_are_in_contact(self, object_a: Object, object_b: Object) -> bool:
        """
        Check if two objects are in contact by checking if one of the contact points is between the two objects,
        or if the two objects are the :attr:`of_object` and :attr:`with_object` of the contact event.
        """
        if self.with_object is None:
            if object_a == self.of_object:
                return object_b in self.contact_points.get_objects_that_have_points()
            elif object_b == self.of_object:
                return object_a in self.contact_points.get_objects_that_have_points()
        else:
            if object_a == self.with_object:
                return object_b == self.of_object
            elif object_b == self.with_object:
                return object_a == self.of_object
        return False

    def __str__(self):
        return f"Contact {self.contact_points[0].link_a.object.name}: {self.object_names}"

    def __repr__(self):
        return self.__str__()


class LossOfContactEvent(AbstractContactEvent):
    def __init__(self, contact_points: ContactPointsList,
                 latest_contact_points: ContactPointsList,
                 of_object: Object,
                 with_object: Optional[Object] = None,
                 timestamp: Optional[float] = None):
        super().__init__(contact_points, of_object, with_object, timestamp)
        self.latest_contact_points = latest_contact_points

    @property
    def color(self) -> Color:
        return Color(1, 0, 0, 1)

    @property
    def object_names(self):
        return [obj.name for obj in self.contact_points.get_objects_that_got_removed(self.latest_contact_points)]

    @property
    def main_link(self) -> Link:
        return self.latest_contact_points[0].link_a

    @property
    def links(self) -> List[Link]:
        links = []
        for obj in self.objects:
            links.extend(self.latest_contact_points.get_links_in_contact_of_object(obj))
        return links

    @property
    def objects(self):
        return self.contact_points.get_objects_that_got_removed(self.latest_contact_points)

    def __str__(self):
        return f"Loss of contact {self.latest_contact_points[0].link_a.object.name}: {self.object_names}"

    def __repr__(self):
        return self.__str__()


class AbstractAgentContactEvent(ContactEvent, ABC):

    def __init__(self, contact_points: ContactPointsList,
                 agent: Object,
                 with_object: Optional[Object] = None,
                 timestamp: Optional[float] = None):
        super().__init__(contact_points, agent, with_object, timestamp)
        self.agent: Object = agent

    @property
    def agent_name(self):
        return self.agent.name

    @property
    @abstractmethod
    def agent_link(self):
        pass

    @property
    def agent_link_name(self):
        return self.agent_link.name

    @property
    def object_name(self):
        return self.contacted_object.name

    @property
    def contacted_object(self):
        return self.object_link.object

    @property
    def object_link_name(self):
        return self.object_link.name

    @property
    @abstractmethod
    def object_link(self):
        pass


class AgentContactEvent(AbstractAgentContactEvent):

    @property
    def agent_link(self):
        return self.contact_points[0].link_a

    @property
    def object_link(self):
        return self.contact_points[0].link_b


class AgentLossOfContactEvent(AbstractAgentContactEvent):

    def __init__(self, contact_points: ContactPointsList,
                 latest_contact_points: ContactPointsList,
                 agent: Object,
                 with_object: Optional[Object] = None,
                 timestamp: Optional[float] = None):
        super().__init__(contact_points, agent, with_object, timestamp)
        self.agent: Object = agent
        self.latest_contact_points = latest_contact_points

    @property
    def agent_link(self):
        return self.latest_contact_points[0].link_a

    @property
    def object_link(self):
        return self.latest_contact_points[0].link_b


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

    def annotate(self, position: Optional[List[float]] = None, size: Optional[float] = 2) -> TextAnnotation:
        if position is None:
            position = [2, 1, 2]
        color = Color(0, 1, 0, 1)
        self.agent.set_color(color)
        self.picked_object.set_color(color)
        text = f"Picked {self.picked_object.name}"
        self.text_id = World.current_world.add_text(text,
                                                    position,
                                                    color=color,
                                                    size=size)
        return TextAnnotation(text, position, self.text_id, color=color, size=size)

    def __str__(self):
        return f"Pick up event: Agent:{self.agent.name}, Object: {self.picked_object.name}, Timestamp: {self.timestamp}"

    def __repr__(self):
        return self.__str__()


class EventLogger:
    def __init__(self, annotate_events: bool = False):
        self.timeline = {}
        self.event_queue = queue.Queue()
        self.lock = threading.Lock()
        self.annotate_events = annotate_events
        if annotate_events:
            self.annotation_queue = queue.Queue()
            self.annotation_thread = EventAnnotationThread(self)
            self.annotation_thread.start()

    def log_event(self, thread_id, event: Event):
        self.event_queue.put((thread_id, event))
        if self.annotate_events:
            self.annotation_queue.put(event)
        with self.lock:
            if thread_id not in self.timeline:
                self.timeline[thread_id] = []
            self.timeline[thread_id].append(event)

    def print_events(self):
        print("Events:")
        print(self)

    def get_events(self) -> Dict[str, List[Event]]:
        with self.lock:
            events = self.timeline.copy()
        return events

    def get_latest_event_of_thread(self, thread_id: str):
        with self.lock:
            if thread_id not in self.timeline:
                return None
            return self.timeline[thread_id][-1]

    def get_next_event(self):
        try:
            thread_id, event = self.event_queue.get(block=False)
            self.event_queue.task_done()
            return thread_id, event
        except queue.Empty:
            return None, None

    def join(self):
        if self.annotate_events:
            self.annotation_thread.stop()
            self.annotation_queue.join()
        self.event_queue.join()

    def __str__(self):
        return '\n'.join([' '.join([str(v) for v in values]) for values in self.get_events().values()])


class EventAnnotationThread(threading.Thread):
    def __init__(self, logger: EventLogger,
                 initial_z_offset: float = 2,
                 step_z_offset: float = 0.2,
                 max_annotations: int = 5):
        super().__init__()
        self.logger = logger
        self.initial_z_offset = initial_z_offset
        self.step_z_offset = step_z_offset
        self.current_annotations: List[TextAnnotation] = []
        self.max_annotations = max_annotations
        self.exit = False

    def get_next_z_offset(self):
        return self.initial_z_offset - self.step_z_offset * len(self.current_annotations)

    def run(self):
        while not self.exit:
            try:
                event = self.logger.annotation_queue.get(timeout=1)
            except queue.Empty:
                continue
            self.logger.annotation_queue.task_done()
            if len(self.current_annotations) >= self.max_annotations:
                # Move all annotations up and remove the oldest one
                for text_ann in self.current_annotations:
                    World.current_world.remove_text(text_ann.id)
                self.current_annotations.pop(0)
                for text_ann in self.current_annotations:
                    text_ann.position[2] += self.step_z_offset
                    text_ann.id = World.current_world.add_text(text_ann.text,
                                                               text_ann.position,
                                                               color=text_ann.color)
            z_offset = self.get_next_z_offset()
            text_ann = event.annotate([1.5, 1, z_offset])
            self.current_annotations.append(text_ann)
            time.sleep(0.1)

    def stop(self):
        self.exit = True
        self.join()
