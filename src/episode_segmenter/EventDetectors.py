import threading
import time
from abc import ABC, abstractmethod

import numpy as np
from typing_extensions import Optional, List, Union

from pycram.datastructures.dataclasses import ContactPointsList
from pycram.datastructures.enums import ObjectType
from pycram.world_concepts.world_object import Object, Link
from .Events import Event, ContactEvent, LossOfContactEvent, PickUpEvent, EventLogger, AgentContactEvent, \
    AgentLossOfContactEvent, AbstractContactEvent
from .utils import get_angle_between_vectors


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

    def __init__(self, logger: EventLogger, wait_time: Optional[float] = None):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """

        super().__init__()

        self.logger = logger
        self.wait_time = wait_time

        self.exit_thread: Optional[bool] = False
        self.run_once = False

    @property
    @abstractmethod
    def thread_prefix(self) -> str:
        """
        A string that is used as a prefix for the thread ID.
        """
        pass

    @property
    @abstractmethod
    def thread_id(self) -> str:
        """
        A string that identifies the thread.
        """
        pass

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

    @abstractmethod
    def filter_event(self, event: Event) -> Event:
        """
        Filters the event before logging/using it.
        :param event: An object that represents the event.
        :return: An object that represents the filtered event.
        """
        pass

    def get_latest_event(self) -> Event:
        """
        Get the latest event from the logger.

        :return: An instance of the ContactEvent class that represents the contact event.
        """
        latest_event = None
        while latest_event is None:
            latest_event = self.logger.get_latest_event_of_thread(self.thread_id)
            time.sleep(0.01)
        return latest_event

    def get_latest_event_after_timestamp(self, timestamp: float) -> Event:
        """
        Get the latest loss of contact event from the logger.

        :param timestamp: A float value that represents the timestamp to get the loss of contact event after.
        :return: An instance of the LossOfContactEvent class that represents the loss of contact event.
        """
        latest_event = None
        while latest_event is None:
            latest_event = self.logger.get_latest_event_of_thread(self.thread_id)
            if latest_event is not None:
                latest_event = None if latest_event.timestamp < timestamp else latest_event
            if latest_event is None:
                time.sleep(0.01)
        return latest_event


class AbstractContactDetector(PrimitiveEventDetector, ABC):
    def __init__(self, logger: EventLogger, object_to_track: Object, with_object: Optional[Object] = None,
                 max_closeness_distance: Optional[float] = 0.05, wait_time: Optional[float] = 0.1):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param object_to_track: An instance of the Object class that represents the object to track.
        :param max_closeness_distance: An optional float value that represents the maximum distance between the object
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """
        super().__init__(logger, wait_time)
        self.object_to_track = object_to_track
        self.with_object = with_object
        self.max_closeness_distance = max_closeness_distance
        self.latest_contact_points: Optional[ContactPointsList] = ContactPointsList([])

    @property
    def obj_type(self) -> ObjectType:
        """
        The object type of the object to track.
        """
        return self.object_to_track.obj_type

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
            contact_points = self.object_to_track.closest_points_with_obj(self.with_object, self.max_closeness_distance)
        else:
            contact_points = self.object_to_track.closest_points(self.max_closeness_distance)
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

    def filter_event(self, event: AbstractContactEvent) -> Event:
        """
        Filters the contact event by removing the contact points that are not in the list of objects to track.
        :param event: An object that represents the event.
        :return: An object that represents the filtered event.
        """
        event.with_object = self.with_object
        return event


class ContactDetector(AbstractContactDetector):
    """
    A thread that detects if the object got into contact with another object.
    """

    @property
    def thread_id(self) -> str:
        return self.thread_prefix + str(self.object_to_track.id)

    @property
    def thread_prefix(self) -> str:
        """
        A string that is used as a prefix for the thread ID.
        """
        return "contact_"

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
        return [event_type(contact_points, of_object=self.object_to_track, with_object=new_obj)
                for new_obj in new_objects_in_contact]


class LossOfContactDetector(AbstractContactDetector):
    """
    A thread that detects if the object lost contact with another object.
    """

    @property
    def thread_id(self) -> str:
        return self.thread_prefix + str(self.object_to_track.id)

    @property
    def thread_prefix(self) -> str:
        """
        A string that is used as a prefix for the thread ID.
        """
        return "loss_contact_"

    def trigger_events(self, contact_points: ContactPointsList) -> Union[List[LossOfContactEvent],
                                                                         List[AgentLossOfContactEvent]]:
        """
        Check if the object lost contact with another object.

        :param contact_points: The current contact points.
        :return: An instance of the LossOfContactEvent/AgentLossOfContactEvent class that represents the event if the
         object lost contact, else None.
        """
        objects_that_lost_contact = contact_points.get_objects_that_got_removed(self.latest_contact_points)
        if self.with_object is not None:
            objects_that_lost_contact = [obj for obj in objects_that_lost_contact if obj == self.with_object]
        if len(objects_that_lost_contact) == 0:
            return []
        event_type = AgentLossOfContactEvent if self.obj_type in self.agent_types else LossOfContactEvent
        return [event_type(contact_points, self.latest_contact_points, of_object=self.object_to_track, with_object=obj)
                for obj in objects_that_lost_contact]


class EventDetector(PrimitiveEventDetector, ABC):

    def __init__(self, logger: EventLogger, starter_event: Event, wait_time: Optional[float] = None):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the Event class that represents the event to start the event detector.
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """
        super().__init__(logger, wait_time)
        self.starter_event: Event = starter_event

    @classmethod
    @abstractmethod
    def start_condition_checker(cls, event: Event) -> bool:
        """
        Check if the event is a starter event.
        :param event: The Event instance that represents the event.
        """
        pass


class AgentPickUpDetector(EventDetector):
    """
    A thread that detects if the object was picked up by the hand.
    """

    def __init__(self, logger: EventLogger, starter_event: AgentContactEvent, trans_threshold: Optional[float] = 0.08,
                 rot_threshold: Optional[float] = 0.4):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the AgentContactEvent class that represents the event to start the
        event detector, this is a contact between the agent and the object.
        :param trans_threshold: An optional float value that represents the translation threshold.
        :param rot_threshold: An optional float value that represents the rotation threshold.
        """
        super().__init__(logger, starter_event)
        self.agent = starter_event.agent
        self.agent_link = starter_event.agent_link
        self.object_link = self.get_object_link_from_event(starter_event)
        self.object = self.object_link.object
        self.trans_threshold = trans_threshold
        self.rot_threshold = rot_threshold
        self.run_once = True

    @property
    def thread_prefix(self) -> str:
        """
        A string that is used as a prefix for the thread ID.
        """
        return "pick_up_"

    @classmethod
    def filter_event(cls, event: AgentContactEvent) -> AgentContactEvent:
        """
        Filters the contact event by removing the contact points that are not in the list of objects to track.
        :param event: An object that represents the event.
        :return: An object that represents the filtered event.
        """
        event.with_object = cls.get_object_link_from_event(event).object
        return event

    @classmethod
    def get_object_link_from_event(cls, event: AgentContactEvent) -> Link:
        """
        Get the object link from the event.
        """
        pickable_objects = cls.find_pickable_objects_from_contact_event(event)
        links_in_contact = event.links
        return [link for link in links_in_contact if link.object in pickable_objects][0]

    @classmethod
    def start_condition_checker(cls, event: Event) -> bool:
        """
        Check if an agent is in contact with the object.

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

    @staticmethod
    def select_pickable_objects(objects: List[Object]) -> List[Object]:
        """
        Selects the objects that can be picked up.

        :param objects: A list of Object instances.
        """
        return [obj for obj in objects
                if obj.obj_type not in [ObjectType.HUMAN, ObjectType.ROBOT, ObjectType.ENVIRONMENT]]

    @property
    def thread_id(self) -> str:
        return self.thread_prefix + str(self.object.id)

    def detect_events(self) -> List[PickUpEvent]:
        """
        Detects if the object was picked up by the hand.
        Used Features are:
        1. The hand should still be in contact with the object.
        2. While the object that is picked should lose contact with the surface.
        Other features that can be used: Grasping Type, Object Type, and Object Motion.
        :return: An instance of the PickUpEvent class that represents the event if the object was picked up, else None.
        """

        # detect all their contacts at the time of contact with each other.
        initial_contact_event = self.get_latest_contact_event(self.object)
        initial_contact_points = initial_contact_event.contact_points
        if not initial_contact_points.is_object_in_the_list(self.agent):
            print(f"Agent not in contact with object: {self.object.name}")
            return []

        pick_up_event = PickUpEvent(self.object, self.agent, initial_contact_event.timestamp)
        latest_stamp = pick_up_event.timestamp

        supporting_surface_found = False
        while not supporting_surface_found:
            time.sleep(0.01)
            loss_of_contact_event = self.get_latest_event_after_timestamp(latest_stamp)
            loss_of_contact_points = loss_of_contact_event.contact_points

            objects_that_lost_contact = loss_of_contact_points.get_objects_that_got_removed(initial_contact_points)
            print(f"object_in_contact: {self.object.name}")
            if len(objects_that_lost_contact) == 0:
                print(f"continue, object: {self.object.name}")
                continue
            if self.agent in objects_that_lost_contact:
                print(f"Agent lost contact with object: {self.object.name}")
                return []

            supporting_surface_found = self.check_for_supporting_surface(objects_that_lost_contact,
                                                                         initial_contact_points)
            if supporting_surface_found:
                pick_up_event.record_end_timestamp()
                break
            else:
                print(f"Supporting surface not found, object: {self.object.name}")
                continue

        print(f"Object picked up: {self.object.name}")

        return [pick_up_event]

    @staticmethod
    def check_for_supporting_surface(objects: List[Object], contact_points: ContactPointsList) -> bool:
        """
        Check if any of the objects in the list are supporting surfaces.

        :param objects: An instance of the Object class that represents the object to check.
        :param contact_points: A list of ContactPoint instances that represent the contact points of the object.
        :return: A boolean value that represents the condition for the object to be considered as a supporting surface.
        """
        supporting_surface = None
        opposite_gravity = [0, 0, 1]
        smallest_angle = np.pi / 4
        for obj in objects:
            normals = contact_points.get_normals_of_object(obj)
            for normal in normals:
                # check if normal is pointing upwards opposite to gravity by finding the angle between the normal
                # and gravity vector.
                angle = get_angle_between_vectors(normal, opposite_gravity)
                if angle < smallest_angle:
                    smallest_angle = angle
                    supporting_surface = obj
        return supporting_surface is not None
