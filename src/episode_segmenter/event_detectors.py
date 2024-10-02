import threading
import time
from abc import ABC, abstractmethod

import numpy as np
import rospy
from typing_extensions import Optional, List, Union, Type

from pycram.datastructures.dataclasses import ContactPointsList
from pycram.datastructures.enums import ObjectType
from pycram.datastructures.pose import Pose
from pycram.world_concepts.world_object import Object, Link
from .event_logger import EventLogger
from .events import Event, ContactEvent, LossOfContactEvent, PickUpEvent, AgentContactEvent, \
    AgentLossOfContactEvent, AbstractContactEvent, EventUnion, LossOfSurfaceEvent, MotionEvent, StopMotionEvent
from .utils import get_angle_between_vectors, calculate_translation_difference, is_translation_difference_small


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

    def __init__(self, logger: EventLogger, wait_time: Optional[float] = None):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """

        super().__init__()

        self.logger = logger
        self.wait_time = wait_time
        self.objects_to_track: List[Object] = []

        self.exit_thread: Optional[bool] = False
        self.run_once = False

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

    def run(self):
        """
        The main loop of the thread. The event detector is called in a loop until the thread is stopped by setting the
        exit_thread attribute to True. Additionally, there is an optional wait_time attribute that can be set to a float
        value to introduce a delay between calls to the event detector.
        """
        while not self.exit_thread:
            events = self.detect_events()
            if self.wait_time is not None:
                time.sleep(self.wait_time)
            [self.log_event(event) for event in events]
            if self.run_once:
                break

    def log_event(self, event: Event) -> None:
        """
        Logs the event using the logger instance.
        :param event: An object that represents the event.
        :return: None
        """
        self.logger.log_event(self.thread_id, event)

    @property
    def detected_before(self) -> bool:
        """
        Checks if the event was detected before.

        :return: A boolean value that represents if the event was detected before.
        """
        return self.thread_id in self.logger.get_events().keys()


class AbstractContactDetector(PrimitiveEventDetector, ABC):
    def __init__(self, logger: EventLogger, tracked_object: Object, with_object: Optional[Object] = None,
                 max_closeness_distance: Optional[float] = 0.05, wait_time: Optional[float] = 0.1):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param tracked_object: An instance of the Object class that represents the object to track.
        :param max_closeness_distance: An optional float value that represents the maximum distance between the object
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """
        super().__init__(logger, wait_time)
        self.objects_to_track = [tracked_object]
        if with_object is not None:
            self.objects_to_track.append(with_object)
        self.tracked_object = tracked_object
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

    def __init__(self, logger: EventLogger, tracked_object: Object, translation_threshold: float = 0.01,
                 wait_time: Optional[float] = 0.1):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param tracked_object: An instance of the Object class that represents the object to track.
        :param translation_threshold: An optional float value that represents the translation threshold above which the
        object is considered to be moving.
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """
        super().__init__(logger, wait_time)
        self.objects_to_track = [tracked_object]
        self.tracked_object = tracked_object
        self.latest_pose = self.tracked_object.pose
        self.translation_threshold = translation_threshold
        self.was_moving: bool = False

    def detect_events(self) -> List[Union[MotionEvent, StopMotionEvent]]:
        """
        Detect if the object starts or stops moving.

        :return: An instance of the MotionEvent class that represents the event if the object is moving, else None.
        """
        current_pose = self.tracked_object.pose
        if self.is_moving(current_pose) != self.was_moving:
            self.was_moving = not self.was_moving
            return [self.create_event(current_pose)]
        return []

    def is_moving(self, current_pose: Pose) -> bool:
        """
        Check if the object is moving by comparing the current pose with the previous pose.

        :param current_pose: The current pose of the object.
        :return: A boolean value that represents if the object is moving.
        """
        return not is_translation_difference_small(calculate_translation_difference(self.latest_pose.position_as_list(),
                                                                                    current_pose.position_as_list()
                                                                                    ),
                                                   threshold=self.translation_threshold
                                                   )

    def create_event(self, current_pose: Pose) -> Union[MotionEvent, StopMotionEvent]:
        """
        Create a motion event.

        :param current_pose: The current pose of the object.
        :return: An instance of the MotionEvent class that represents the event.
        """
        event_type = MotionEvent if self.was_moving else StopMotionEvent
        event = event_type(self.tracked_object, self.latest_pose, current_pose)
        self.latest_pose = current_pose
        return event


class EventDetector(PrimitiveEventDetector, ABC):

    def __init__(self, logger: EventLogger, starter_event: EventUnion, wait_time: Optional[float] = None):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the Event class that represents the event to start the event detector.
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """
        super().__init__(logger, wait_time)
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

    def __init__(self, logger: EventLogger, starter_event: EventUnion):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of a type of Event that represents the event to
         start the event detector.
        """
        super().__init__(logger, starter_event)
        self.tracked_object = self.get_object_to_pick_from_event(starter_event)
        self.objects_to_track = [self.tracked_object]
        self.run_once = True

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
                if obj.obj_type not in [ObjectType.HUMAN, ObjectType.ROBOT, ObjectType.ENVIRONMENT]]


class AgentPickUpDetector(AbstractPickUpDetector):
    """
    A detector that detects if the tracked_object was picked up by an agent, such as a human or a robot.
    """

    def __init__(self, logger: EventLogger, starter_event: AgentContactEvent):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the AgentContactEvent class that represents the event to start the
        event detector, this is a contact between the agent and the tracked_object.
        """
        super().__init__(logger, starter_event)
        self.agent = starter_event.agent
        self.agent_link = starter_event.agent_link
        self.object_link = self.get_object_link_from_event(starter_event)
        self.surface_detector = LossOfSurfaceDetector(logger, self.tracked_object)
        self.surface_detector.start()

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

        pick_up_event = PickUpEvent(self.tracked_object, self.agent, self.start_timestamp)

        while True:
            loss_of_surface_event = get_latest_event_of_detector_for_object(LossOfSurfaceDetector,
                                                                            self.tracked_object,
                                                                            after_timestamp=self.start_timestamp
                                                                            )
            pick_up_event.record_end_timestamp()
            objects_that_lost_contact = loss_of_surface_event.get_objects_that_got_removed(self.start_contact_points)
            if len(objects_that_lost_contact) == 0:
                rospy.logdebug(f"continue, tracked_object: {self.tracked_object.name}")
                time.sleep(0.01)
                continue
            if self.agent in objects_that_lost_contact:
                rospy.logdebug(f"Agent lost contact with tracked_object: {self.tracked_object.name}")
                return []

            break

        rospy.loginfo(f"Object picked up: {self.tracked_object.name}")

        return [pick_up_event]

    @property
    def start_contact_points(self) -> ContactPointsList:
        return self.starter_event.contact_points

    def join(self, timeout: Optional[float] = None):
        self.surface_detector.exit_thread = True
        self.surface_detector.join(timeout)
        super().join(timeout)


class MotionPickUpDetector(AbstractPickUpDetector):

    def __init__(self, logger: EventLogger, starter_event: LossOfContactEvent):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the ContactEvent class that represents the event to start the event
         detector.
        """
        super().__init__(logger, starter_event)
        self.surface = self.get_surface_from_event(starter_event)


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
    latest_event = None
    logger = EventLogger.current_logger
    while latest_event is None:
        latest_event = logger.get_latest_event_of_detector_for_object(detector_type.thread_prefix, obj)
        if latest_event is not None and after_timestamp is not None:
            latest_event = None if latest_event.timestamp < after_timestamp else latest_event
        if latest_event is None:
            time.sleep(0.01)
    return latest_event
