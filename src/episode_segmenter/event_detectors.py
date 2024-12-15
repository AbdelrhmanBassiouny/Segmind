import threading
import time
from abc import ABC, abstractmethod
from datetime import timedelta
from functools import cached_property
from math import ceil
from queue import Queue

import numpy as np
import rospy
from tf.transformations import euler_from_quaternion
from typing_extensions import Optional, List, Union, Type, Tuple

import pycrap
from pycram import World
from pycram.datastructures.dataclasses import ContactPointsList
from pycram.datastructures.pose import Pose
from pycram.ros.logging import logdebug
from pycram.world_concepts.world_object import Object
from pycrap import PhysicalObject
from .event_logger import EventLogger
from .events import Event, ContactEvent, LossOfContactEvent, PickUpEvent, AgentContactEvent, \
    AgentLossOfContactEvent, EventUnion, LossOfSurfaceEvent, TranslationEvent, StopTranslationEvent, NewObjectEvent, \
    RotationEvent, StopRotationEvent, PlacingEvent, MotionEvent, StopMotionEvent
from .object_tracker import ObjectTracker, ObjectTrackerFactory
from .utils import get_angle_between_vectors, calculate_euclidean_distance, calculate_quaternion_difference, \
    check_if_object_is_supported, check_if_object_is_supported_using_contact_points, \
    check_if_object_is_supported_by_another_object


class PrimitiveEventDetector(threading.Thread, ABC):
    """
    A thread that detects events in another thread and logs them. The event detector is a function that has no arguments
    and returns an object that represents the event. The event detector is called in a loop until the thread is stopped
    by setting the exit_thread attribute to True.
    """

    thread_prefix: str = ""
    """
    A string that is used as a prefix for the thread ID.
    """

    def __init__(self, logger: EventLogger, wait_time: Optional[float] = None, *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """

        super().__init__()

        self.logger = logger
        self.wait_time = wait_time
        self.objects_to_track: List[Object] = []
        self._tracked_object: Optional[Object] = None
        self._with_object: Optional[Object] = None
        self.kill_event = threading.Event()

        self.run_once = False
        self._pause: bool = False

    @property
    def tracked_object(self) -> Object:
        return self._tracked_object

    @tracked_object.setter
    def tracked_object(self, obj: Object):
        self._tracked_object = obj
        self.update_tracked_objects(obj)

    @property
    def with_object(self) -> Object:
        return self._with_object

    @with_object.setter
    def with_object(self, obj: Object):
        self._with_object = obj
        self.update_tracked_objects(obj)

    def update_tracked_objects(self, obj: Object):
        if obj is not None:
            self.objects_to_track.append(obj)

    def stop(self):
        """
        Stop the event detector.
        """
        self.kill_event.set()

    @property
    def thread_id(self) -> str:
        return f"{self.thread_prefix}{'_'.join([obj.name for obj in self.objects_to_track])}"

    @abstractmethod
    def detect_events(self) -> List[Event]:
        """
        The event detector function that is called in a loop until the thread is stopped.
        :return: A list of Event instances.
        """
        pass

    def pause(self):
        """
        Pause the event detector.
        """
        self._pause: bool = True

    def resume(self):
        """
        Resume the event detector.
        """
        self._pause: bool = False

    def run(self):
        """
        The main loop of the thread. The event detector is called in a loop until the thread is stopped by setting the
        exit_thread attribute to True. Additionally, there is an optional wait_time attribute that can be set to a float
        value to introduce a delay between calls to the event detector.
        """
        while not self.kill_event.is_set():
            self._wait_if_paused()
            last_processing_time = time.time()
            self.detect_and_log_events()
            if self.run_once:
                break
            self._wait_to_maintain_loop_rate(last_processing_time)

    def detect_and_log_events(self):
        """
        Detect and log the events.
        """
        events = self.detect_events()
        [self.log_event(event) for event in events]

    def _wait_if_paused(self):
        """
        Wait if the event detector is paused.
        """
        while self._pause and not self.kill_event.is_set():
            time.sleep(0.1)

    def _wait_to_maintain_loop_rate(self, last_processing_time: float):
        """
        Wait to maintain the loop rate of the event detector.

        :param last_processing_time: The time of the last processing.
        """
        if self.wait_time is None:
            return
        time_diff = time.time() - last_processing_time
        if time_diff < self.wait_time:
            time.sleep(self.wait_time - time_diff)

    def log_event(self, event: Event) -> None:
        """
        Logs the event using the logger instance.
        :param event: An object that represents the event.
        :return: None
        """
        event.detector_thread_id = self.thread_id
        self.logger.log_event(event)

    @property
    def detected_before(self) -> bool:
        """
        Checks if the event was detected before.

        :return: A boolean value that represents if the event was detected before.
        """
        return self.thread_id in self.logger.get_events_per_thread().keys()


class NewObjectDetector(PrimitiveEventDetector):
    """
    A thread that detects if a new object is added to the scene and logs the NewObjectEvent.
    """

    thread_prefix = "new_object_"
    """
    A string that is used as a prefix for the thread ID.
    """

    def __init__(self, logger: EventLogger, wait_time: Optional[float] = 0.1,
                 avoid_objects: Optional[List[str]] = None, *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        :param avoid_objects: An optional list of strings that represent the names of the objects to avoid.
        """
        super().__init__(logger, wait_time, *args, **kwargs)
        self.new_object_queue: Queue[Object] = Queue()
        self.avoid_objects = avoid_objects if avoid_objects is not None else []
        World.current_world.add_callback_on_add_object(self.on_add_object)

    def on_add_object(self, obj: Object):
        """
        Callback function that is called when a new object is added to the scene.
        """
        if not any([obj.name.lower() in name.lower() for name in self.avoid_objects]):
            self.new_object_queue.put(obj)

    def detect_events(self) -> List[Event]:
        """
        Detect if a new object is added to the scene and invoke the NewObjectEvent.

        :return: A NewObjectEvent that represents the addition of a new object to the scene.
        """
        event = NewObjectEvent(self.new_object_queue.get())
        self.new_object_queue.task_done()
        return [event]

    def join(self, timeout=None):
        """
        Remove the callback on the add object event and resume the thread to be able to join.
        """
        World.current_world.remove_callback_on_add_object(self.on_add_object)
        self.new_object_queue.join()
        super().join(timeout)


class AbstractContactDetector(PrimitiveEventDetector, ABC):
    def __init__(self, logger: EventLogger, starter_event: EventUnion, with_object: Optional[Object] = None,
                 max_closeness_distance: Optional[float] = 0.05, wait_time: Optional[float] = 0.01,
                 *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the Event class that represents the event to start the event detector.
        :param max_closeness_distance: An optional float value that represents the maximum distance between the object
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """
        super().__init__(logger, wait_time, *args, **kwargs)
        self.tracked_object = starter_event.tracked_object
        self.with_object = with_object
        self.max_closeness_distance = max_closeness_distance
        self.latest_contact_points: Optional[ContactPointsList] = ContactPointsList([])

    @property
    def obj_type(self) -> Type[PhysicalObject]:
        """
        The object type of the object to track.
        """
        return self.tracked_object.obj_type

    def detect_events(self) -> List[Event]:
        """
        Detects the closest points between the object to track and another object in the scene if the with_object
        attribute is set, else, between the object to track and all other objects in the scene.
        """
        contact_points = self.get_contact_points()

        events = self.trigger_events(contact_points)

        self.latest_contact_points = contact_points

        return events

    def get_contact_points(self) -> ContactPointsList:
        if self.with_object is not None:
            contact_points = self.tracked_object.closest_points_with_obj(self.with_object, self.max_closeness_distance)
        else:
            contact_points = self.tracked_object.closest_points(self.max_closeness_distance)
        return contact_points

    @abstractmethod
    def trigger_events(self, contact_points: ContactPointsList) -> List[Event]:
        """
        Checks if the detection condition is met, (e.g., the object is in contact with another object),
        and returns an object that represents the event.
        :param contact_points: The current contact points.
        :return: An object that represents the event.
        """
        pass


class ContactDetector(AbstractContactDetector):
    """
    A thread that detects if the object got into contact with another object.
    """

    thread_prefix = "contact_"
    """
    A string that is used as a prefix for the thread ID.
    """

    def trigger_events(self, contact_points: ContactPointsList) -> Union[List[ContactEvent], List[AgentContactEvent]]:
        """
        Check if the object got into contact with another object.

        :param contact_points: The current contact points.
        :return: An instance of the ContactEvent/AgentContactEvent class that represents the event if the object got
         into contact, else None.
        """
        new_objects_in_contact = contact_points.get_new_objects(self.latest_contact_points)
        if self.with_object is not None:
            new_objects_in_contact = [obj for obj in new_objects_in_contact if obj == self.with_object]
        if len(new_objects_in_contact) == 0:
            return []
        event_type = AgentContactEvent if issubclass(self.obj_type, pycrap.Agent) else ContactEvent
        return [event_type(contact_points.get_points_of_object(new_obj),
                           of_object=self.tracked_object, with_object=new_obj)
                for new_obj in new_objects_in_contact]


class LossOfContactDetector(AbstractContactDetector):
    """
    A thread that detects if the object lost contact with another object.
    """

    thread_prefix = "loss_contact_"
    """
    A string that is used as a prefix for the thread ID.
    """

    def trigger_events(self, contact_points: ContactPointsList) -> Union[List[LossOfContactEvent],
    List[AgentLossOfContactEvent]]:
        """
        Check if the object lost contact with another object.

        :param contact_points: The current contact points.
        :return: An instance of the LossOfContactEvent/AgentLossOfContactEvent class that represents the event if the
         object lost contact, else None.
        """
        objects_that_lost_contact = self.get_objects_that_lost_contact(contact_points)
        if len(objects_that_lost_contact) == 0:
            return []
        event_type = AgentLossOfContactEvent if issubclass(self.obj_type, pycrap.Agent) else LossOfContactEvent
        return [event_type(contact_points, self.latest_contact_points, of_object=self.tracked_object, with_object=obj)
                for obj in objects_that_lost_contact]

    def get_objects_that_lost_contact(self, contact_points: ContactPointsList) -> List[Object]:
        """
        Get the objects that lost contact with the object to track.

        :param contact_points: The current contact points.
        :return: A list of Object instances that represent the objects that lost contact with the object to track.
        """
        objects_that_lost_contact = contact_points.get_objects_that_got_removed(self.latest_contact_points)
        if self.with_object is not None:
            objects_that_lost_contact = [obj for obj in objects_that_lost_contact if obj == self.with_object]
        return objects_that_lost_contact


class LossOfSurfaceDetector(LossOfContactDetector):

    def trigger_events(self, contact_points: ContactPointsList) -> List[LossOfSurfaceEvent]:
        """
        Check if the object lost contact with the surface.

        :param contact_points: The current contact points.
        :return: An instance of the LossOfSurfaceEvent class that represents the event if the object lost contact with
        the surface, else None.
        """
        objects_that_lost_contact = self.get_objects_that_lost_contact(contact_points)
        if len(objects_that_lost_contact) == 0:
            return []
        supporting_surface = check_for_supporting_surface(objects_that_lost_contact, self.latest_contact_points)
        if supporting_surface is None:
            return []
        return [LossOfSurfaceEvent(contact_points, self.latest_contact_points, of_object=self.tracked_object,
                                   surface=supporting_surface)]


class MotionDetector(PrimitiveEventDetector, ABC):
    """
    A thread that detects if the object starts or stops moving and logs the TranslationEvent or StopTranslationEvent.
    """

    thread_prefix = "motion_"
    """
    A string that is used as a prefix for the thread ID.
    """

    def __init__(self, logger: EventLogger, starter_event: NewObjectEvent, velocity_threshold: float = 0.07,
                 wait_time: Optional[float] = 0.1,
                 time_between_frames: Optional[timedelta] = timedelta(milliseconds=50),
                 *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the NewObjectEvent class that represents the event to start the event.
        :param velocity_threshold: An optional float value that represents the velocity threshold for the object to be
        considered as moving.
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        :param time_between_frames: An optional timedelta value that represents the time between frames.
        """
        super().__init__(logger, wait_time, *args, **kwargs)
        self.tracked_object = starter_event.tracked_object
        self.latest_pose = self.tracked_object.pose
        self.latest_time = time.time()
        self.velocity_threshold = velocity_threshold

        self.measure_timestep: timedelta = timedelta(milliseconds=350)
        # frames per measure timestep
        self.measure_frame_rate: float = ceil(self.measure_timestep.total_seconds() /
                                              time_between_frames.total_seconds()) + 0.5
        self.measure_timestep = time_between_frames * self.measure_frame_rate

        self.distance_threshold: float = self.velocity_threshold * self.measure_timestep.total_seconds()
        self.was_moving: bool = False

    def update_latest_pose_and_time(self):
        """
        Update the latest pose and time of the object.
        """
        self.latest_pose, self.latest_time = self.get_current_pose_and_time()

    def get_current_pose_and_time(self) -> Tuple[Pose, float]:
        """
        Get the current pose and time of the object.
        """
        return self.tracked_object.pose, time.time()

    def detect_events(self) -> List[Union[TranslationEvent, StopTranslationEvent]]:
        """
        Detect if the object starts or stops moving.

        :return: An instance of the TranslationEvent class that represents the event if the object is moving, else None.
        """
        if self.time_since_last_event < self.measure_timestep.total_seconds():
            time.sleep(self.measure_timestep.total_seconds() - self.time_since_last_event)
        is_moving = self.is_moving()
        if is_moving != self.was_moving:
            self.update_object_motion_state(is_moving)
            self.was_moving = not self.was_moving
            return [self.create_event()]
        self.update_latest_pose_and_time()
        return []

    @property
    def time_since_last_event(self) -> float:
        return time.time() - self.latest_time

    @abstractmethod
    def update_object_motion_state(self, moving: bool) -> None:
        """
        Update the object motion state.
        """
        pass

    def is_moving(self) -> bool:
        """
        Check if the object is moving by comparing the current pose with the previous pose.

        :return: A boolean value that represents if the object is moving.
        """
        distance = self.calculate_distance(self.tracked_object.pose)
        return distance > self.distance_threshold

    def create_event(self) -> Union[TranslationEvent, StopTranslationEvent]:
        """
        Create a motion event.

        :return: An instance of the TranslationEvent class that represents the event.
        """
        current_pose, current_time = self.get_current_pose_and_time()
        event_type = self.get_event_type()
        event = event_type(self.tracked_object, self.latest_pose, current_pose, timestamp=current_time)
        return event

    @abstractmethod
    def calculate_distance(self, current_pose: Pose):
        pass

    @abstractmethod
    def get_event_type(self):
        pass


class TranslationDetector(MotionDetector):
    thread_prefix = "translation_"

    def update_object_motion_state(self, moving: bool) -> None:
        """
        Update the object motion state.
        """
        self.tracked_object.is_translating = moving

    def calculate_distance(self, current_pose: Pose):
        """
        Calculate the Euclidean distance between the latest and current positions of the object.

        :param current_pose: The current pose of the object.
        """
        return calculate_euclidean_distance(self.latest_pose.position_as_list(), current_pose.position_as_list())

    def get_event_type(self):
        return TranslationEvent if self.was_moving else StopTranslationEvent


class RotationDetector(MotionDetector):
    thread_prefix = "rotation_"

    def __init__(self, logger: EventLogger, starter_event: NewObjectEvent,
                 angular_velocity_threshold: float = 10 * np.pi / 180,
                 wait_time: Optional[float] = 0.1,
                 time_between_frames: Optional[timedelta] = timedelta(milliseconds=50),
                 *args, **kwargs):
        super().__init__(logger, starter_event, velocity_threshold=angular_velocity_threshold, wait_time=wait_time,
                         time_between_frames=time_between_frames, *args, **kwargs)

    def update_object_motion_state(self, moving: bool) -> None:
        """
        Update the object motion state.
        """
        self.tracked_object.is_rotating = moving

    def calculate_distance(self, current_pose: Pose):
        """
        Calculate the angle between the latest and current quaternions of the object

        :param current_pose: The current pose of the object.
        """
        quat_diff = calculate_quaternion_difference(self.latest_pose.orientation_as_list(),
                                                    current_pose.orientation_as_list())
        angle = 2 * np.arccos(quat_diff[0])
        return angle

    def get_event_type(self):
        return RotationEvent if self.was_moving else StopRotationEvent


class EventDetector(PrimitiveEventDetector, ABC):

    def __init__(self, logger: EventLogger, starter_event: EventUnion, wait_time: Optional[float] = None,
                 *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the Event class that represents the event to start the event detector.
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """
        super().__init__(logger, wait_time, *args, **kwargs)
        self.starter_event: EventUnion = starter_event

    @classmethod
    @abstractmethod
    def start_condition_checker(cls, event: Event) -> bool:
        """
        Check if the event is a starter event.

        :param event: The Event instance that represents the event.
        """
        pass

    def check_for_event_post_starter_event(self, event_type: Type[Event]) -> Optional[EventUnion]:
        """
        Check if the tracked_object was involved in an event after the starter event.

        :param event_type: The event type to check for.
        :return: The event if the tracked_object was involved in an event, else None.
        """
        event = self.object_tracker.get_first_event_of_type_after_event(event_type, self.starter_event)
        if event is None:
            logdebug(f"{event_type.__name__} found no event after {self.start_timestamp} with object :"
                     f" {self.tracked_object.name}")
            return None
        return event

    @cached_property
    def object_tracker(self) -> ObjectTracker:
        return ObjectTrackerFactory.get_tracker(self.tracked_object)

    def check_for_event_near_starter_event(self, event_type: Type[Event],
                                           time_tolerance: timedelta) -> Optional[EventUnion]:
        """
        Check if the tracked_object was involved in an event near the starter event (i.e. could be before or after).

        :param event_type: The event type to check for.
        :param time_tolerance: The time tolerance to consider the event as near the starter event.
        :return: The event if the tracked_object was involved in an event, else None.
        """
        event = self.object_tracker.get_nearest_event_of_type_to_event(self.starter_event,
                                                                       tolerance=time_tolerance,
                                                                       event_type=event_type)
        if event is None:
            logdebug(f"{event_type.__name__} found no event after {self.start_timestamp} with object :"
                     f" {self.tracked_object.name}")
            return None
        return event

    @property
    def start_timestamp(self) -> float:
        return self.starter_event.timestamp


class MotionPlacingDetector(EventDetector, ABC):
    """
    A detector that detects if the tracked_object was placed on a surface by using motion and contact.
    """

    thread_prefix = "placing_"

    def __init__(self, logger: EventLogger, starter_event: EventUnion, *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of a type of Event that represents the event to
         start the event detector.
        """
        super().__init__(logger, starter_event, *args, **kwargs)
        self.tracked_object = self.get_object_to_place_from_event(starter_event)
        self.placing_event: Optional[PlacingEvent] = None
        self.run_once = True

    @classmethod
    @abstractmethod
    def get_object_to_place_from_event(cls, event: Event) -> Object:
        """
        Get the tracked_object to place from the event.
        """
        pass


class AbstractAgentObjectInteractionDetector(EventDetector, ABC):
    """
    An abstract detector that detects an interaction between the agent and an object.
    """

    thread_prefix = "agent_object_interaction_"
    """
    A string that is used as a prefix for the thread ID.
    """

    currently_tracked_objects: List[Object] = []
    """
    A list of Object instances that represent the objects that are currently being tracked.
    """

    def __init__(self, logger: EventLogger, starter_event: EventUnion, *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of a type of Event that represents the event to
         start the event detector.
        """
        super().__init__(logger, starter_event, *args, **kwargs)
        self.tracked_object = self.get_object_to_track_from_starter_event(starter_event)
        self.currently_tracked_objects.append(self.tracked_object)
        self.interaction_event: EventUnion = self._init_interaction_event()
        self.end_timestamp: Optional[float] = None
        self.run_once = True

    @abstractmethod
    def _init_interaction_event(self) -> EventUnion:
        """
        Initialize the interaction event.
        """
        pass

    def detect_events(self) -> List[EventUnion]:
        """
        Detect if the tracked_object was interacted with by the agent.

        :return: An instance of the interaction event if the tracked_object was interacted with, else None.
        """

        while not self.kill_event.is_set():

            if not self.interaction_checks():
                time.sleep(0.01)
                continue

            self.interaction_event.end_timestamp = self.end_timestamp

            break

        rospy.loginfo(f"{self.__class__.__name__} detected an interaction with: {self.tracked_object.name}")

        return [self.interaction_event]

    @abstractmethod
    def interaction_checks(self) -> bool:
        """
        Perform checks to determine if the object was interacted with.

        :return: A boolean value that represents if all the checks passed and the object was interacted with.
        """
        pass

    @classmethod
    @abstractmethod
    def get_object_to_track_from_starter_event(cls, starter_event: EventUnion) -> Object:
        """
        Get the object to track for interaction from the possible starter event.

        :param starter_event: The possible starter event that can be used to get the object to track.
        """
        pass


class AbstractPickUpDetector(AbstractAgentObjectInteractionDetector, ABC):
    """
    An abstract detector that detects if the tracked_object was picked up.
    """

    thread_prefix = "pick_up_"
    """
    A string that is used as a prefix for the thread ID.
    """

    def _init_interaction_event(self) -> EventUnion:
        return PickUpEvent(self.tracked_object, timestamp=self.start_timestamp)


class AgentPickUpDetector(AbstractPickUpDetector):
    """
    A detector that detects if the tracked_object was picked up by an agent, such as a human or a robot.
    """

    def __init__(self, logger: EventLogger, starter_event: AgentContactEvent, *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the AgentContactEvent class that represents the event to start the
        event detector, this is a contact between the agent and the tracked_object.
        """
        super().__init__(logger, starter_event, *args, **kwargs)
        self.surface_detector = LossOfSurfaceDetector(logger, NewObjectEvent(self.tracked_object))
        self.surface_detector.start()
        self.agent = starter_event.agent
        self.interaction_event.agent = self.agent

    @classmethod
    def get_object_to_track_from_starter_event(cls, event: AgentContactEvent) -> Object:
        return cls.get_new_transportable_objects(event)[0]

    @classmethod
    def get_new_transportable_objects(cls, event: AgentContactEvent) -> List[Object]:
        transportable_objects = select_transportable_objects_from_contact_event(event)
        new_transportable_objects = [obj for obj in transportable_objects if obj not in cls.currently_tracked_objects]
        return new_transportable_objects

    @classmethod
    def start_condition_checker(cls, event: Event) -> bool:
        """
        Check if an agent is in contact with the tracked_object.

        :param event: The ContactEvent instance that represents the contact event.
        """
        return isinstance(event, AgentContactEvent) and any(cls.get_new_transportable_objects(event))

    def interaction_checks(self) -> bool:
        """
        Perform extra checks to determine if the object was picked up.
        """
        loss_of_surface_event = self.check_for_event_post_starter_event(LossOfSurfaceEvent)

        if not loss_of_surface_event:
            return False

        if self.agent in loss_of_surface_event.latest_objects_that_got_removed:
            rospy.logdebug(f"Agent lost contact with tracked_object: {self.tracked_object.name}")
            self.kill_event.set()
            return False

        self.end_timestamp = loss_of_surface_event.timestamp

        return True

    def stop(self, timeout: Optional[float] = None):
        self.surface_detector.stop()
        self.surface_detector.join(timeout)
        super().stop()


class MotionPickUpDetector(AbstractPickUpDetector):

    def __init__(self, logger: EventLogger, starter_event: LossOfContactEvent, *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the ContactEvent class that represents the event to start the event
         detector.
        """
        super().__init__(logger, starter_event, *args, **kwargs)

    @classmethod
    def get_object_to_track_from_starter_event(cls, event: LossOfContactEvent) -> Object:
        return select_transportable_objects_from_loss_of_contact_event(event)[0]

    @classmethod
    def start_condition_checker(cls, event: Event) -> bool:
        """
        Check if an agent is in contact with the tracked_object.

        :param event: The ContactEvent instance that represents the contact event.
        """
        return (isinstance(event, LossOfContactEvent)
                and any(select_transportable_objects_from_loss_of_contact_event(event)))

    def interaction_checks(self) -> bool:
        """
        Check for upward motion after the object lost contact with the surface.
        """
        latest_event = self.check_for_event_near_starter_event(TranslationEvent, timedelta(milliseconds=1000))

        if not latest_event:
            return False

        z_motion = latest_event.current_pose.position.z - latest_event.start_pose.position.z
        if z_motion > 0.001:
            self.end_timestamp = max(latest_event.timestamp, self.start_timestamp)
            return True

        return False


class PlacingDetector(AbstractAgentObjectInteractionDetector):
    """
    An abstract detector that detects if the tracked_object was placed by the agent.
    """

    thread_prefix = "placing_"

    def _init_interaction_event(self) -> EventUnion:
        return PlacingEvent(self.tracked_object, timestamp=self.start_timestamp)

    def interaction_checks(self) -> bool:
        return self.initial_interaction_checkers()

    @classmethod
    def get_object_to_track_from_starter_event(cls, starter_event: MotionEvent) -> Object:
        return starter_event.tracked_object

    @classmethod
    def start_condition_checker(cls, event: Event) -> bool:
        """
        Check if an agent is in contact with the tracked_object.

        :param event: The ContactEvent instance that represents the contact event.
        """
        if isinstance(event, ContactEvent) and any(select_transportable_objects([event.tracked_object])):
            print('new placing detector for object:', event.tracked_object.name)
            return True
        return False

    def initial_interaction_checkers(self) -> bool:
        """
        Perform initial checks to determine if the object was placed.
        """
        stop_motion_event = self.check_for_event_near_starter_event(StopMotionEvent, timedelta(milliseconds=1000))
        if stop_motion_event and self.check_if_contact_event_is_with_surface(self.starter_event):
            self.end_timestamp = self.start_timestamp
            print(f"end_timestamp: {self.end_timestamp}")
            return True

        return False

    def check_if_contact_event_is_with_surface(self, contact_event: ContactEvent) -> bool:
        """
        Check if the contact event is with a surface.

        :param contact_event: The ContactEvent instance that represents the contact event.
        """
        return check_if_object_is_supported_by_another_object(self.tracked_object, contact_event.with_object,
                                                              contact_event.contact_points)


def check_for_supporting_surface(objects_that_lost_contact: List[Object],
                                 initial_contact_points: ContactPointsList) -> Optional[Object]:
    """
    Check if any of the objects that lost contact are supporting surfaces.

    :param objects_that_lost_contact: An instance of the Object class that represents the tracked_object to check.
    :param initial_contact_points: A list of ContactPoint instances that represent the contact points of the
     tracked_object before it lost contact.
    :return: An instance of the Object class that represents the supporting surface if found, else None.
    """
    supporting_surface = None
    opposite_gravity = [0, 0, 1]
    smallest_angle = np.pi / 4
    for obj in objects_that_lost_contact:
        normals = initial_contact_points.get_normals_of_object(obj)
        for normal in normals:
            # check if normal is pointing upwards opposite to gravity by finding the angle between the normal
            # and gravity vector.
            angle = get_angle_between_vectors(normal, opposite_gravity)
            if angle < smallest_angle:
                smallest_angle = angle
                supporting_surface = obj
    return supporting_surface


def get_latest_event_of_detector_for_object(detector_type: Type[PrimitiveEventDetector], obj: Object,
                                            after_timestamp: Optional[float] = None) -> Optional[EventUnion]:
    """
    Get the latest event for the detector for the object from the logger.

    :param obj: An instance of the Object class that represents the object to get the event for.
    :param detector_type: The type of the event detector.
    :param after_timestamp: A float value that represents the timestamp to get the event after.
    :return: The latest event of the detector for the object.
    """
    logger = EventLogger.current_logger
    latest_event = logger.get_latest_event_of_detector_for_object(detector_type.thread_prefix, obj)
    if latest_event is not None and after_timestamp is not None:
        latest_event = None if latest_event.timestamp < after_timestamp else latest_event
    if latest_event is None:
        time.sleep(0.01)
    return latest_event


def get_nearest_event_of_detector_for_object(detector_type: Type[PrimitiveEventDetector],
                                             obj: Object,
                                             timestamp: float,
                                             time_tolerance: timedelta = timedelta(milliseconds=200)) -> Optional[
    EventUnion]:
    """
    Get the event of the detector for the object near the timestamp.

    :param obj: An instance of the Object class that represents the object to get the event for.
    :param detector_type: The type of the event detector.
    :param timestamp: A float value that represents the timestamp to get the event near.
    :param time_tolerance: A float value that represents the time tolerance.
    :return: The event of the detector for the object near the timestamp.
    """
    logger = EventLogger.current_logger
    event = logger.get_nearest_event_of_detector_for_object(detector_type.thread_prefix, obj, timestamp)
    if (event is not None) and (abs(event.timestamp - timestamp) <= time_tolerance.total_seconds()):
        return event


def select_transportable_objects_from_contact_event(event: Union[ContactEvent, AgentContactEvent]) -> List[Object]:
    """
    Select the objects that can be transported from the contact event.

    :param event: The contact event
    """
    contacted_objects = event.contact_points.get_objects_that_have_points()
    return select_transportable_objects(contacted_objects + [event.tracked_object])


def select_transportable_objects_from_loss_of_contact_event(event: Union[LossOfContactEvent,
AgentLossOfContactEvent,
LossOfSurfaceEvent]) -> List[Object]:
    """
    Select the objects that can be transported from the loss of contact event.
    """
    objects_that_lost_contact = event.latest_objects_that_got_removed
    return select_transportable_objects(objects_that_lost_contact + [event.tracked_object])


def select_transportable_objects(objects: List[Object]) -> List[Object]:
    """
    Select the objects that can be transported

    :param objects: A list of Object instances.
    """
    transportable_objects = [obj for obj in objects
                             if
                             not issubclass(obj.obj_type, (pycrap.Agent, pycrap.Location, pycrap.Floor, pycrap.Genobj))]
    return transportable_objects


EventDetectorUnion = Union[ContactDetector, LossOfContactDetector, LossOfSurfaceDetector, MotionDetector,
TranslationDetector, RotationDetector, NewObjectDetector, AgentPickUpDetector, MotionPickUpDetector, EventDetector]
TypeEventDetectorUnion = Union[Type[ContactDetector], Type[LossOfContactDetector], Type[LossOfSurfaceDetector],
Type[MotionDetector], Type[TranslationDetector], Type[RotationDetector], Type[NewObjectDetector], Type[
    AgentPickUpDetector],
Type[MotionPickUpDetector], Type[EventDetector]
]
