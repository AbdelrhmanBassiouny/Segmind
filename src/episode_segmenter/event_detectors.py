import datetime
from queue import Queue
import threading
import time
from abc import ABC, abstractmethod
from math import ceil

import numpy as np
import rospy
from typing_extensions import Optional, List, Union, Type, Tuple

from pycram import World
from pycram.datastructures.dataclasses import ContactPointsList
from pycram.datastructures.enums import ObjectType
from pycram.datastructures.pose import Pose
from pycram.world_concepts.world_object import Object, Link
from pycram.ros.logging import logdebug
from .event_logger import EventLogger
from .events import Event, ContactEvent, LossOfContactEvent, PickUpEvent, AgentContactEvent, \
    AgentLossOfContactEvent, EventUnion, LossOfSurfaceEvent, MotionEvent, StopMotionEvent, NewObjectEvent
from .utils import get_angle_between_vectors, calculate_euclidean_distance


class PrimitiveEventDetector(threading.Thread, ABC):
    """
    A thread that detects events in another thread and logs them. The event detector is a function that has no arguments
    and returns an object that represents the event. The event detector is called in a loop until the thread is stopped
    by setting the exit_thread attribute to True.
    """

    agent_types: List[ObjectType] = [ObjectType.HUMAN, ObjectType.ROBOT]
    """
    A list of ObjectType values that represent the agent types.
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
        self.kill_event = threading.Event()

        self.run_once = False
        self._pause: bool = False

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

    def __init__(self, logger: EventLogger, wait_time: Optional[float] = 0.1, *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """
        super().__init__(logger, wait_time, *args, **kwargs)
        self.new_object_queue: Queue = Queue()
        World.current_world.add_callback_on_add_object(self.on_add_object)

    def on_add_object(self, obj: Object):
        """
        Callback function that is called when a new object is added to the scene.
        """
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
                 max_closeness_distance: Optional[float] = 0.05, wait_time: Optional[float] = 0.1,
                 *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the Event class that represents the event to start the event detector.
        :param max_closeness_distance: An optional float value that represents the maximum distance between the object
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """
        super().__init__(logger, wait_time, *args, **kwargs)
        self.objects_to_track = [starter_event.tracked_object]
        if with_object is not None:
            self.objects_to_track.append(with_object)
        self.tracked_object = starter_event.tracked_object
        self.with_object = with_object
        self.max_closeness_distance = max_closeness_distance
        self.latest_contact_points: Optional[ContactPointsList] = ContactPointsList([])

    @property
    def obj_type(self) -> ObjectType:
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
        event_type = AgentContactEvent if self.obj_type in self.agent_types else ContactEvent
        return [event_type(contact_points, of_object=self.tracked_object, with_object=new_obj)
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
        event_type = AgentLossOfContactEvent if self.obj_type in self.agent_types else LossOfContactEvent
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


class MotionDetector(PrimitiveEventDetector):
    """
    A thread that detects if the object starts or stops moving and logs the MotionEvent or StopMotionEvent.
    """

    thread_prefix = "motion_"
    """
    A string that is used as a prefix for the thread ID.
    """

    def __init__(self, logger: EventLogger, starter_event: NewObjectEvent, velocity_threshold: float = 0.07,
                 wait_time: Optional[float] = 0.1,
                 time_between_frames: Optional[datetime.timedelta] = datetime.timedelta(milliseconds=50),
                 *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the NewObjectEvent class that represents the event to start the event.
        :param velocity_threshold: An optional float value that represents the velocity threshold for the object to be
        considered as moving.
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        :param time_between_frames: An optional datetime.timedelta value that represents the time between frames.
        """
        super().__init__(logger, wait_time, *args, **kwargs)
        self.objects_to_track = [starter_event.tracked_object]
        self.tracked_object = starter_event.tracked_object
        self.latest_pose = self.tracked_object.pose
        self.latest_time = time.time()
        self.velocity_threshold = velocity_threshold

        self.measure_timestep: datetime.timedelta = datetime.timedelta(milliseconds=350)
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

    def detect_events(self) -> List[Union[MotionEvent, StopMotionEvent]]:
        """
        Detect if the object starts or stops moving.

        :return: An instance of the MotionEvent class that represents the event if the object is moving, else None.
        """
        if self.time_since_last_event < self.measure_timestep.total_seconds():
            time.sleep(self.measure_timestep.total_seconds() - self.time_since_last_event)
        if self.is_moving() != self.was_moving:
            self.was_moving = not self.was_moving
            return [self.create_event()]
        self.update_latest_pose_and_time()
        return []

    @property
    def time_since_last_event(self) -> float:
        return time.time() - self.latest_time

    def is_moving(self) -> bool:
        """
        Check if the object is moving by comparing the current pose with the previous pose.

        :return: A boolean value that represents if the object is moving.
        """
        current_pose = self.tracked_object.pose
        distance = calculate_euclidean_distance(self.latest_pose.position_as_list(), current_pose.position_as_list())
        return distance > self.distance_threshold

    def create_event(self) -> Union[MotionEvent, StopMotionEvent]:
        """
        Create a motion event.

        :return: An instance of the MotionEvent class that represents the event.
        """
        current_pose, current_time = self.get_current_pose_and_time()
        event_type = MotionEvent if self.was_moving else StopMotionEvent
        event = event_type(self.tracked_object, self.latest_pose, current_pose, timestamp=current_time)
        return event


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

    @property
    def start_timestamp(self) -> float:
        return self.starter_event.timestamp

    @classmethod
    @abstractmethod
    def filter_event(cls, event: EventUnion) -> EventUnion:
        """
        Filter the event before logging/using it.

        :param event: An object that represents the event.
        :return: An object that represents the filtered event.
        """
        pass


class AbstractPickUpDetector(EventDetector, ABC):
    """
    An abstract detector that detects if the tracked_object was picked up.
    """

    thread_prefix = "pick_up_"
    """
    A string that is used as a prefix for the thread ID.
    """

    def __init__(self, logger: EventLogger, starter_event: EventUnion, *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of a type of Event that represents the event to
         start the event detector.
        """
        super().__init__(logger, starter_event, *args, **kwargs)
        self.tracked_object = self.get_object_to_pick_from_event(starter_event)
        self.objects_to_track = [self.tracked_object]
        self.pick_up_event: Optional[PickUpEvent] = None
        self.run_once = True
        self.break_loop = False

    def detect_events(self) -> List[PickUpEvent]:
        """
        Detect if the tracked_object was picked up by the hand.
        Used Features are:
        1. The hand should still be in contact with the tracked_object.
        2. While the tracked_object that is picked should lose contact with the surface.
        Other features that can be used: Grasping Type, Object Type, and Object Motion.

        :return: An instance of the PickUpEvent class that represents the event if the tracked_object was picked up,
         else None.
        """

        self.pick_up_event = PickUpEvent(self.tracked_object, timestamp=self.start_timestamp)

        while not self.kill_event.is_set():

            if not self.extra_checks():
                if self.break_loop:
                    break
                else:
                    time.sleep(0.01)
                    continue

            self.pick_up_event.end_timestamp = self.get_end_timestamp()

            break

        rospy.loginfo(f"Object picked up: {self.tracked_object.name}")

        self.fill_pick_up_event()

        return [self.pick_up_event]

    @abstractmethod
    def get_end_timestamp(self) -> float:
        """
        Get the end timestamp of the pickup event.
        """
        pass

    @abstractmethod
    def fill_pick_up_event(self):
        """
        Fill the pickup event with the necessary information.
        """
        pass

    @abstractmethod
    def extra_checks(self) -> bool:
        """
        Perform extra checks to determine if the object was picked up.

        :return: A boolean value that represents if all the checks passed and the object was picked up.
        """
        pass

    def check_object_lost_contact_with_surface(self) -> Union[Tuple[Optional[LossOfSurfaceEvent],
                                                                    Optional[List[Object]]]]:
        """
        Check if the tracked_object lost contact with the surface.

        :return: A list of Object instances that represent the objects that lost contact with the tracked_object.
        """
        loss_of_surface_event = get_latest_event_of_detector_for_object(LossOfSurfaceDetector,
                                                                        self.tracked_object,
                                                                        after_timestamp=self.start_timestamp
                                                                        )
        if loss_of_surface_event is None:
            logdebug(f"continue, tracked_object: {self.tracked_object.name}")
            return None, None

        objects_that_lost_contact = loss_of_surface_event.latest_objects_that_got_removed

        return loss_of_surface_event, objects_that_lost_contact

    @classmethod
    @abstractmethod
    def get_object_to_pick_from_event(cls, event: Event) -> Object:
        """
        Get the tracked_object to pick up from the event.
        """
        pass

    @staticmethod
    def select_pickable_objects(objects: List[Object]) -> List[Object]:
        """
        Select the objects that can be picked up.

        :param objects: A list of Object instances.
        """
        return [obj for obj in objects
                if obj.obj_type not in [ObjectType.HUMAN, ObjectType.ROBOT, ObjectType.ENVIRONMENT,
                                        ObjectType.IMAGINED_SURFACE]]


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
        self.surface_detector = LossOfSurfaceDetector(logger, self.starter_event)
        self.surface_detector.start()
        self.agent = starter_event.agent
        self.agent_link = starter_event.agent_link
        self.object_link = self.get_object_link_from_event(starter_event)
        self.end_timestamp: Optional[float] = None

    def get_end_timestamp(self) -> float:
        return self.end_timestamp

    def fill_pick_up_event(self):
        self.pick_up_event.agent = self.agent

    @classmethod
    def filter_event(cls, event: AgentContactEvent) -> Event:
        """
        Filter the event by removing objects that are not in the list of objects to track.

        :param event: An object that represents the event.
        :return: An object that represents the filtered event.
        """
        event.with_object = cls.get_object_to_pick_from_event(event)
        return event

    @classmethod
    def get_object_to_pick_from_event(cls, event: AgentContactEvent) -> Object:
        """
        Get the tracked_object link from the event.

        :param event: The AgentContactEvent instance that represents the contact event.
        """
        return cls.get_object_link_from_event(event).object

    @classmethod
    def get_object_link_from_event(cls, event: AgentContactEvent) -> Link:
        """
        Get the tracked_object link from the event.

        :param event: The AgentContactEvent instance that represents the contact event.
        """
        pickable_objects = cls.find_pickable_objects_from_contact_event(event)
        links_in_contact = event.links
        return [link for link in links_in_contact if link.object in pickable_objects][0]

    @classmethod
    def start_condition_checker(cls, event: Event) -> bool:
        """
        Check if an agent is in contact with the tracked_object.

        :param event: The ContactEvent instance that represents the contact event.
        """
        return isinstance(event, AgentContactEvent) and any(cls.find_pickable_objects_from_contact_event(event))

    @classmethod
    def find_pickable_objects_from_contact_event(cls, event: AgentContactEvent) -> List[Object]:
        """
        Find the pickable objects from the contact event.

        :param event: The AgentContactEvent instance that represents the contact event.
        """
        contacted_objects = event.contact_points.get_objects_that_have_points()
        return cls.select_pickable_objects(contacted_objects)

    def extra_checks(self) -> bool:
        """
        Perform extra checks to determine if the object was picked up.
        """
        loss_of_surface_event, objects_that_lost_contact = self.check_object_lost_contact_with_surface()

        if objects_that_lost_contact is None:
            time.sleep(0.01)
            return False

        if self.agent in objects_that_lost_contact:
            rospy.logdebug(f"Agent lost contact with tracked_object: {self.tracked_object.name}")
            self.break_loop = True
            return False

        self.end_timestamp = loss_of_surface_event.timestamp

        return True

    @property
    def start_contact_points(self) -> ContactPointsList:
        return self.starter_event.contact_points

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
        self.end_timestamp: Optional[float] = None

    def get_end_timestamp(self) -> float:
        return self.end_timestamp

    @classmethod
    def get_object_to_pick_from_event(cls, event: LossOfSurfaceEvent) -> Object:
        return cls.find_pickable_objects_from_contact_event(event)[0]

    def fill_pick_up_event(self):
        pass

    @classmethod
    def filter_event(cls, event: LossOfContactEvent) -> Event:
        return event

    @classmethod
    def start_condition_checker(cls, event: Event) -> bool:
        """
        Check if an agent is in contact with the tracked_object.

        :param event: The ContactEvent instance that represents the contact event.
        """
        return isinstance(event, LossOfContactEvent) and any(cls.find_pickable_objects_from_contact_event(event))

    @classmethod
    def find_pickable_objects_from_contact_event(cls, event: LossOfContactEvent) -> List[Object]:
        """
        Find the pickable objects from the contact event.

        :param event: The AgentContactEvent instance that represents the contact event.
        """
        return cls.select_pickable_objects(event.latest_objects_that_got_removed + [event.tracked_object])

    def extra_checks(self) -> bool:
        """
        Check for upward motion after the object lost contact with the surface.
        """
        latest_event = get_latest_event_of_detector_for_object(MotionDetector, self.tracked_object,
                                                               self.start_timestamp)

        if latest_event is None:
            return False

        return True


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


EventDetectorUnion = Union[ContactDetector, LossOfContactDetector, LossOfSurfaceDetector, MotionDetector,
NewObjectDetector, AgentPickUpDetector, MotionPickUpDetector, EventDetector]
TypeEventDetectorUnion = Union[Type[ContactDetector], Type[LossOfContactDetector], Type[LossOfSurfaceDetector],
Type[MotionDetector], Type[NewObjectDetector], Type[AgentPickUpDetector],
Type[MotionPickUpDetector], Type[EventDetector]
]
