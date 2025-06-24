from copy import copy
from queue import Queue, Empty
import time
from os.path import dirname

from ripple_down_rules.rdr_decorators import RDRDecorator
from segmind.datastructures.events import PlacingEvent

from pycram.world_concepts.world_object import Object
from segmind.episode_player import EpisodePlayer
from typing_extensions import Any, Dict, List, Optional, Type, Callable

from pycram.datastructures.world_entity import abstractmethod, PhysicalBody
from pycram.ros import logdebug, loginfo
from .atomic_event_detectors import AtomicEventDetector
from ..datastructures.events import MotionEvent, EventUnion, StopMotionEvent, NewObjectEvent, InsertionEvent, Event, \
    ContainmentEvent, ContactEvent, InterferenceEvent, LossOfInterferenceEvent, StopTranslationEvent, \
    LossOfSurfaceEvent, LossOfSupportEvent, SupportEvent
from ..datastructures.object_tracker import ObjectTrackerFactory
from ..utils import get_support

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

    def reset(self):
        self.bodies_states = {}
        self.update_initial_state()
        self.event_queue = Queue()
    
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
    @staticmethod
    def event_condition(event: PlacingEvent) -> bool:
        # logdebug(f"Checking bodies {event.link_names} with tracked object {event.tracked_object.name}")
        # return any(['hole' in body.name for body in event.links])
        def conditions(event_to_check: EventUnion) -> bool:
            return (isinstance(event_to_check, InterferenceEvent) and
                    event_to_check.timestamp < event.timestamp and
                    "hole" in event.with_object.name)

        latest_interference_with_hole = event.object_tracker.get_nearest_event_to_event_with_conditions(event)

    check_on_events = {PlacingEvent: None}

    def update_body_state(self, body: PhysicalBody):
        """
        Update the state of a body.
        """
        if isinstance(body, Object):
            for link in body.links.values():
                self.bodies_states[link] = link.update_containment()
        self.bodies_states[body] = body.update_containment()

    def detect_events(self) -> None:
        """
        Detect Containment relations between objects.
        """
        try:
            # checked_bodies: List[PhysicalBody] = []
            while True:
                event = self.event_queue.get_nowait()
                self.event_queue.task_done()
                # if event.tracked_object in checked_bodies:
                #     continue
                if event.tracked_object in self.bodies_states:
                    known_containments = self.bodies_states[event.tracked_object]
                else:
                    known_containments = []
                self.update_body_state(event.tracked_object)
                new_containments = set(self.bodies_states[event.tracked_object]) - set(known_containments)
                for new_containment in new_containments:
                    self.logger.log_event(ContainmentEvent(event.tracked_object, new_containment,
                                                           timestamp=event.timestamp))
                time.sleep(self.wait_time.total_seconds())
        except Empty:
            pass
    
    @classmethod
    def event_type(cls) -> Type[ContainmentEvent]:
        return ContainmentEvent


class InsertionDetector(SpatialRelationDetector):

    @staticmethod
    def event_condition(event: PlacingEvent) -> bool:
        # logdebug(f"Checking bodies {event.link_names} with tracked object {event.tracked_object.name}")
        # return any(['hole' in body.name for body in event.links])
        def conditions(event_to_check: EventUnion) -> bool:
            return (isinstance(event_to_check, InterferenceEvent) and
                    event_to_check.timestamp < event.timestamp and
                    "hole" in event_to_check.with_object.name)
        latest_interference_with_hole = event.object_tracker.get_nearest_event_to_event_with_conditions(event, conditions)
        return latest_interference_with_hole is not None
    check_on_events = {ContainmentEvent: event_condition}

    def update_body_state(self, body: PhysicalBody, with_bodies: Optional[List[PhysicalBody]] = None):
        """
        Update the state of a body.
        """
        # body.update_containment(intersection_ratio=0.7)
        # self.bodies_states[body] = copy(body.contained_in_bodies)
        pass

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
                    # hole: PhysicalBody = [link for link in event.links if 'hole' in link.name][0]
                    # logdebug(f"Checking insertion for {event.tracked_object.name} through hole {hole.name}")#
                    # import pdb; pdb.set_trace()
                    def conditions(event_to_check: EventUnion) -> bool:
                        return (isinstance(event_to_check, InterferenceEvent) and
                                event_to_check.timestamp < event.timestamp and
                                "hole" in event_to_check.with_object.name)

                    latest_interference_with_hole = event.object_tracker.get_nearest_event_to_event_with_conditions(
                        event, conditions)
                    hole = latest_interference_with_hole.with_object
                    if not self.hole_insertion_verifier(hole, event):
                        if event.tracked_object.is_moving:
                            continue
                        else:
                            break
                    agent = event.agent if hasattr(event, "agent") else None
                    end_timestamp = event.end_timestamp if hasattr(event, "end_timestamp") else None
                    insertion_event = InsertionEvent(event.tracked_object, [hole], hole,
                                                         agent=agent, timestamp=event.timestamp,
                                                         end_timestamp=end_timestamp)
                    insertion_event.update_action_description()
                    self.logger.log_event(insertion_event)
                    break
                time.sleep(self.wait_time.total_seconds())
        except Empty:
            pass

    @staticmethod
    def ask_now(case_dict):
        self_ = case_dict['self_']
        hole = case_dict['hole']
        event = case_dict['event']
        return "object_3" in event.tracked_object.name
    hole_insertion_verifier_rdr = RDRDecorator(f"{dirname(__file__)}/models", (bool,),
                                               True, fit=False,
                                               fitting_decorator=EpisodePlayer.pause_resume,
                                               package_name="segmind",
                                               ask_now=ask_now)

    @hole_insertion_verifier_rdr.decorator
    def hole_insertion_verifier(self, hole: PhysicalBody, event: InterferenceEvent) -> bool:
        pass
        # return hole not in event.tracked_object.contained_in_bodies

    @classmethod
    def event_type(cls) -> Type[InsertionEvent]:
        return InsertionEvent


class SupportDetector(SpatialRelationDetector):

    @staticmethod
    def event_condition(event: StopTranslationEvent) -> bool:
        # logdebug(f"Checking bodies {event.link_names} with tracked object {event.tracked_object.name}")
        return len(event.tracked_object.contact_points) > 0
    check_on_events = {StopTranslationEvent: None, LossOfInterferenceEvent: None}

    def update_body_state(self, body: PhysicalBody, with_bodies: Optional[List[PhysicalBody]] = None):
        """
        Update the state of a body.
        """
        support = get_support(body)
        if body in self.bodies_states:
            if support is None:
                if self.bodies_states[body] is None:
                    return
                else:
                    self.logger.log_event(LossOfSupportEvent(body, self.bodies_states[body]))
            else:
                if self.bodies_states[body] is None:
                    self.logger.log_event(SupportEvent(body, support))
                elif support != self.bodies_states[body]:
                    self.logger.log_event(LossOfSupportEvent(body, self.bodies_states[body]))
                    self.logger.log_event(SupportEvent(body, support))
                else:
                    return
        self.bodies_states[body] = support
        ObjectTrackerFactory.get_tracker(body).support = support

    def detect_events(self) -> None:
        """
        Detect Containment relations between objects.
        """
        try:
            while True:
                event = self.event_queue.get_nowait()
                self.event_queue.task_done()
                self.update_body_state(event.tracked_object)
                time.sleep(self.wait_time.total_seconds())
        except Empty:
            pass

    @staticmethod
    def ask_now(case_dict):
        self_ = case_dict['self_']
        hole = case_dict['hole']
        event = case_dict['event']
        return "object_3" in event.tracked_object.name
    hole_insertion_verifier_rdr = RDRDecorator(f"{dirname(__file__)}/models", (bool,),
                                               True, fit=False,
                                               fitting_decorator=EpisodePlayer.pause_resume,
                                               package_name="segmind",
                                               ask_now=ask_now)

    @hole_insertion_verifier_rdr.decorator
    def hole_insertion_verifier(self, hole: PhysicalBody, event: InterferenceEvent) -> bool:
        pass
        # return hole not in event.tracked_object.contained_in_bodies

    @classmethod
    def event_type(cls) -> Type[InsertionEvent]:
        return InsertionEvent