from copy import copy
from queue import Queue, Empty
import time

from typing_extensions import Any, Dict, List, Optional, Type, Callable

from pycram.datastructures.world_entity import abstractmethod, PhysicalBody
from pycram.ros import logdebug, loginfo
from .atomic_event_detectors import AtomicEventDetector
from ..datastructures.events import MotionEvent, EventUnion, StopMotionEvent, NewObjectEvent, InsertionEvent, Event, \
    ContainmentEvent, ContactEvent, InterferenceEvent
from ..datastructures.object_tracker import ObjectTrackerFactory

EventCondition = Callable[[EventUnion], bool]


def default_condition(event: EventUnion) -> bool:
    return True


class SpatialRelationDetector(AtomicEventDetector):
    """
    A class that detects spatial relations between objects.
    """
    check_on_events = {NewObjectEvent: default_condition, StopMotionEvent: default_condition}

    def __init__(self, check_on_events: Optional[Dict[Type[Event], EventCondition]] = None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.event_queue: Queue[MotionEvent] = Queue()
        self.queues.append(self.event_queue)
        self.check_on_events = check_on_events if check_on_events is not None else self.check_on_events
        self.bodies_states: Dict[PhysicalBody, Any] = {}
        self.update_initial_state()
        for event, cond in self.check_on_events.items():
            self.logger.add_callback(event, self.on_event, cond)
    
    def update_initial_state(self):
        for body in self.world.objects:
            self.update_body_state(body)

    @abstractmethod
    def update_body_state(self, body: PhysicalBody):
        """
        Update the state of a body.
        """
        pass
    
    def on_event(self, event: MotionEvent):
        """
        A callback that is called when a MotionEvent occurs, and adds the event to the event queue.

        :param event: The MotionEvent that occurred.
        """
        loginfo(f"Adding event {event} to event queue")
        self.event_queue.put(event)

    def detect_events(self) -> None:
        """
        Detect spatial relations between objects.
        """
        try:
            checked_bodies: List[PhysicalBody] = []
            while True:
                event = self.event_queue.get_nowait()
                self.event_queue.task_done()
                logdebug(f"Checking event {event}")
                involved_bodies = event.involved_bodies
                bodies_to_check = list(filter(lambda body: body not in checked_bodies, involved_bodies))
                checked_bodies.extend(self.world.update_containment_for(bodies_to_check))
                logdebug(f"Checked bodies: {[body.name for body in checked_bodies]}")
        except Empty:
            pass

    def __str__(self):
        return self.__class__.__name__

    def _join(self, timeout=None):
        pass


class ContainmentDetector(SpatialRelationDetector):

    def update_body_state(self, body: PhysicalBody):
        """
        Update the state of a body.
        """
        body.update_containment(intersection_ratio=0.5)
        self.bodies_states[body] = copy(body.contained_in_bodies)

    def detect_events(self) -> None:
        """
        Detect Containment relations between objects.
        """
        try:
            checked_bodies: List[PhysicalBody] = []
            while True:
                if self.exc is not None:
                    break
                event = self.event_queue.get_nowait()
                self.event_queue.task_done()
                if event.tracked_object in checked_bodies:
                    continue
                logdebug(f"Checking containment for {event.tracked_object.name}")
                if event.tracked_object in self.bodies_states:
                    known_containments = self.bodies_states[event.tracked_object]
                else:
                    known_containments = []
                self.update_body_state(event.tracked_object)
                new_containments = set(event.tracked_object.contained_in_bodies) - set(known_containments)
                if len(new_containments) == 0:
                    continue
                agent = event.agent if hasattr(event, "agent") else None
                end_timestamp = event.end_timestamp if hasattr(event, "end_timestamp") else None
                self.logger.log_event(ContainmentEvent(event.tracked_object, new_containments,
                 agent=agent, timestamp=event.timestamp, end_timestamp=end_timestamp))
                time.sleep(self.wait_time.total_seconds())
        except Empty:
            pass
    
    @classmethod
    def event_type(cls) -> Type[ContainmentEvent]:
        return ContainmentEvent


class InsertionDetector(SpatialRelationDetector):

    @staticmethod
    def event_condition(event: InterferenceEvent) -> bool:
        logdebug(f"Checking bodies {event.link_names} with tracked object {event.tracked_object.name}")
        return any(['hole' in body.name for body in event.links])
    check_on_events = {InterferenceEvent: event_condition}

    def update_body_state(self, body: PhysicalBody):
        """
        Update the state of a body.
        """
        body.update_containment(intersection_ratio=0.5)
        self.bodies_states[body] = copy(body.contained_in_bodies)

    def detect_events(self) -> None:
        """
        Detect Containment relations between objects.
        """
        try:
            checked_bodies: List[PhysicalBody] = []
            while True:
                event = self.event_queue.get_nowait()
                self.event_queue.task_done()
                if event.tracked_object in checked_bodies:
                    logdebug(f"tracked object {event.tracked_object.name} is already checked")
                    continue
                while True:
                    # time.sleep(self.wait_time.total_seconds())
                    hole: PhysicalBody = [link for link in event.links if 'hole' in link.name][0]
                    # if len(event.tracked_object.get_contact_points_with_body(hole)) > 0:
                    #     logdebug(f"hole {hole.name} not in contact {event.tracked_object.name}")
                    #     break
                    logdebug(f"Checking insertion for {event.tracked_object.name} through hole {hole.name}")
                    self.update_body_state(event.tracked_object)
                    if hole not in event.tracked_object.contained_in_bodies:
                        if event.tracked_object.is_moving:
                            continue
                        else:
                            break
                    agent = event.agent if hasattr(event, "agent") else None
                    end_timestamp = event.end_timestamp if hasattr(event, "end_timestamp") else None
                    self.logger.log_event(InsertionEvent(event.tracked_object, [hole], hole,
                                                         agent=agent, timestamp=event.timestamp,
                                                         end_timestamp=end_timestamp))
                    break
                # time.sleep(self.wait_time.total_seconds())
        except Empty:
            pass

    @classmethod
    def event_type(cls) -> Type[InsertionEvent]:
        return InsertionEvent