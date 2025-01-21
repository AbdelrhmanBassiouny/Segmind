import threading
import time
from abc import ABC, abstractmethod
from datetime import timedelta
from functools import cached_property
from math import ceil
from queue import Queue

import numpy as np
import rospy

from .mixins import HasTrackedObjects, HasOneTrackedObject, HasTwoTrackedObjects

try:
    from matplotlib import pyplot as plt
except ImportError:
    plt = None
from tf.transformations import euler_from_quaternion
from typing_extensions import Optional, List, Union, Type, Tuple

import pycrap
from pycram import World
from pycram.datastructures.dataclasses import ContactPointsList
from pycram.datastructures.pose import Pose
from pycram.datastructures.world import UseProspectionWorld
from pycram.datastructures.world_entity import PhysicalBody
from pycram.ros.logging import logdebug
from pycram.world_concepts.world_object import Object
from pycrap import PhysicalObject
from .event_logger import EventLogger
from .events import Event, ContactEvent, LossOfContactEvent, PickUpEvent, AgentContactEvent, \
    AgentLossOfContactEvent, EventUnion, LossOfSurfaceEvent, TranslationEvent, StopTranslationEvent, NewObjectEvent, \
    RotationEvent, StopRotationEvent, PlacingEvent, MotionEvent, StopMotionEvent
from .motion_detection_helpers import ConsistentGradient, MotionDetectionMethod, DataFilter
from .object_tracker import ObjectTracker, ObjectTrackerFactory
from .utils import get_angle_between_vectors, calculate_quaternion_difference, \
    check_if_in_contact_with_support, calculate_translation


class PrimitiveEventDetector(threading.Thread, ABC):
    """
    A thread that detects events in another thread and logs them. The event detector is a function that has no arguments
    and returns an object that represents the event. The event detector is called in a loop until the thread is stopped
    by setting the exit_thread attribute to True.
    """

    def __init__(self, logger: EventLogger, wait_time: Optional[float] = None, *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """

        super().__init__()

        self.logger = logger
        self.wait_time = wait_time
        self.kill_event = threading.Event()
        self.queues: List[Queue] = []

        self.run_once = False
        self._pause: bool = False

    def stop(self):
        """
        Stop the event detector.
        """
        self.kill_event.set()

    @property
    def thread_id(self) -> str:
        return f"{self.__class__.__name__}_{self.ident}"

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
        while True:

            if self.kill_event.is_set() and self.all_queues_empty:
                break

            self._wait_if_paused()

            last_processing_time = time.time()
            self.detect_and_log_events()

            if self.run_once:
                break
            else:
                self._wait_to_maintain_loop_rate(last_processing_time)

    @property
    def all_queues_empty(self) -> bool:
        """
        Check if all the queues are empty.

        :return: A boolean value that represents if all the queues are empty.
        """
        return all([queue.empty() for queue in self.queues])

    def detect_and_log_events(self):
        """
        Detect and log the events.
        """
        events = self.detect_events()
        if events:
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
        if self.wait_time is not None:
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

    @abstractmethod
    def __str__(self):
        ...

    def __repr__(self):
        return self.__str__()


class NewObjectDetector(PrimitiveEventDetector):
    """
    A thread that detects if a new object is added to the scene and logs the NewObjectEvent.
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
        self.queues.append(self.new_object_queue)
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

    def stop(self):
        """
        Remove the callback on the add object event and resume the thread to be able to join.
        """
        World.current_world.remove_callback_on_add_object(self.on_add_object)
        super().stop()

    def __str__(self):
        return self.thread_id


class DetectorWithOneTrackedObject(PrimitiveEventDetector, HasOneTrackedObject, ABC):
    """
    A mixin class that provides one tracked object for the event detector.
    """
    def __init__(self, logger: EventLogger, tracked_object: Object, wait_time: Optional[float] = None, *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param tracked_object: An Object instance that represents the object to track.
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """
        HasOneTrackedObject.__init__(self, tracked_object)
        PrimitiveEventDetector.__init__(self, logger, wait_time, *args, **kwargs)

    def __str__(self):
        return f"{self.thread_id} - {self.tracked_object.name}"


class DetectorWithTwoTrackedObjects(PrimitiveEventDetector, HasTwoTrackedObjects, ABC):
    """
    A mixin class that provides two tracked objects for the event detector.
    """
    def __init__(self, logger: EventLogger, tracked_object: Object, with_object: Optional[Object] = None,
                 wait_time: Optional[float] = None, *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param tracked_object: An Object instance that represents the object to track.
        :param with_object: An optional Object instance that represents the object to track.
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """
        HasTwoTrackedObjects.__init__(self, tracked_object, with_object)
        PrimitiveEventDetector.__init__(self, logger, wait_time, *args, **kwargs)

    def __str__(self):
        return f"{self.thread_id} - {self.tracked_object.name} - {self.with_object.name}"


class AbstractContactDetector(DetectorWithTwoTrackedObjects, ABC):
    def __init__(self, logger: EventLogger, tracked_object: Object,
                 with_object: Optional[Object] = None,
                 max_closeness_distance: Optional[float] = 0.05, wait_time: Optional[float] = 0.01,
                 *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the Event class that represents the event to start the event detector.
        :param max_closeness_distance: An optional float value that represents the maximum distance between the object
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """
        super().__init__(logger, wait_time, *args, **kwargs)
        DetectorWithTwoTrackedObjects.__init__(self, logger, tracked_object, with_object, wait_time, *args, **kwargs)
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

    def trigger_events(self, contact_points: ContactPointsList) -> Union[List[LossOfContactEvent],
                                                                         List[AgentLossOfContactEvent]]:
        """
        Check if the object lost contact with another object.

        :param contact_points: The current contact points.
        :return: An instance of the LossOfContactEvent/AgentLossOfContactEvent class that represents the event if the
         object lost contact, else None.
        """
        bodies_that_lost_contact = self.get_bodies_that_lost_contact(contact_points)
        if len(bodies_that_lost_contact) == 0:
            return []
        event_type = AgentLossOfContactEvent if issubclass(self.obj_type, pycrap.Agent) else LossOfContactEvent
        return [event_type(contact_points, self.latest_contact_points, of_object=self.tracked_object,
                           with_object=body.parent_entity)
                for body in bodies_that_lost_contact]

    def get_bodies_that_lost_contact(self, contact_points: ContactPointsList) -> List[PhysicalBody]:
        """
        Get the objects that lost contact with the object to track.

        :param contact_points: The current contact points.
        :return: A list of Object instances that represent the objects that lost contact with the object to track.
        """
        bodies_that_lost_contact = contact_points.get_bodies_that_got_removed(self.latest_contact_points)
        if self.with_object is not None:
            bodies_that_lost_contact = [body for body in bodies_that_lost_contact
                                        if body.parent_entity == self.with_object]
        return bodies_that_lost_contact


class LossOfSurfaceDetector(LossOfContactDetector):

    def trigger_events(self, contact_points: ContactPointsList) -> List[LossOfSurfaceEvent]:
        """
        Check if the object lost contact with the surface.

        :param contact_points: The current contact points.
        :return: An instance of the LossOfSurfaceEvent class that represents the event if the object lost contact with
        the surface, else None.
        """
        bodies_that_lost_contact = self.get_bodies_that_lost_contact(contact_points)
        if len(bodies_that_lost_contact) == 0:
            return []
        supporting_surface = check_if_in_contact_with_support(self.tracked_object,
                                                              bodies_that_lost_contact)
        if supporting_surface is None:
            return []
        return [LossOfSurfaceEvent(contact_points, self.latest_contact_points, of_object=self.tracked_object,
                                   surface=supporting_surface)]


class MotionDetector(DetectorWithOneTrackedObject, ABC):
    """
    A thread that detects if the object starts or stops moving and logs the TranslationEvent or StopTranslationEvent.
    """

    latest_pose: Optional[Pose]
    """
    The latest pose of the object.
    """
    latest_time: Optional[float]
    """
    The latest time where the latest pose was recorded.
    """
    event_time: Optional[float]
    """
    The time when the event occurred.
    """
    start_pose: Optional[Pose]
    """
    The start pose of the object at start of detection.
    """
    filtered_distances: Optional[np.ndarray]
    """
    The filtered distances during the window timeframe.
    """

    def __init__(self, logger: EventLogger, tracked_object: Object,
                 detection_method: MotionDetectionMethod = ConsistentGradient(),
                 measure_timestep: timedelta = timedelta(milliseconds=100),
                 time_between_frames: timedelta = timedelta(milliseconds=50),
                 window_timeframe: timedelta = timedelta(milliseconds=700),
                 distance_filter_method: Optional[DataFilter] = None,
                 *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the NewObjectEvent class that represents the event to start the event.
        :param tracked_object: An optional Object instance that represents the object to track.
        :param detection_method: The motion detection method that is used to detect if the object is moving.
        :param measure_timestep: The time between calls to the event detector.
        :param time_between_frames: The time between frames of episode player.
        :param window_timeframe: The timeframe of the window that is used to calculate the distances.
        :param distance_filter_method: An optional DataFilter instance that is used to filter the distances.
        """
        DetectorWithOneTrackedObject.__init__(self, logger, tracked_object,
                                              wait_time=measure_timestep.total_seconds(), *args, **kwargs)
        self.time_between_frames: timedelta = time_between_frames
        self.measure_timestep: timedelta = measure_timestep
        self.window_timeframe: timedelta = window_timeframe
        self.filter: Optional[DataFilter] = distance_filter_method
        self.detection_method: MotionDetectionMethod = detection_method

        self._init_event_data()

        self._update_measure_timestep_and_wait_time()

        self.window_size: int = ceil(self.window_timeframe.total_seconds()
                                     / self.measure_timestep.total_seconds())

        self._init_data_holders()

        self.was_moving: bool = False

        self.plot_distances: bool = False
        self.plot_distance_windows: bool = False
        self.plot_frequencies: bool = False

    def _init_event_data(self):
        """
        Initialize the event time and start pose of the object/event.
        """
        self.latest_pose, self.latest_time = self.get_current_pose_and_time()
        self.event_time: float = self.latest_time
        self.start_pose: Pose = self.latest_pose

    def _update_measure_timestep_and_wait_time(self):
        """
        Update the measure timestep and the wait time between calls to the event detector.
        """
        # frames per measure timestep
        self.measure_frame_rate: float = ceil(self.measure_timestep.total_seconds() /
                                              self.time_between_frames.total_seconds())
        self.measure_timestep = self.time_between_frames * self.measure_frame_rate
        self.wait_time = self.measure_timestep.total_seconds()

    def _init_data_holders(self):
        """
        Initialize the pose, time, and distance data holders.
        """
        # Data
        self.poses: List[Pose] = []
        self.times: List[float] = []

        # Window data
        self.latest_distances: List[List[float]] = []
        self.filtered_distances: Optional[np.ndarray] = None
        self.latest_times: List[float] = []

        # Plotting data
        self.original_distances: List[List[List[float]]] = []
        self.all_filtered_distances: List[np.ndarray] = []
        self.all_times: List[List[float]] = []

    def update_with_latest_motion_data(self):
        """
        Update the latest pose and time of the object.
        """
        self.latest_pose, self.latest_time = self.get_current_pose_and_time()
        self.poses.append(self.latest_pose)
        self.times.append(self.latest_time)
        if len(self.poses) > 1:
            self.calculate_and_update_latest_distance()
            self._crop_distances_and_times_to_window_size()

    def get_current_pose_and_time(self) -> Tuple[Pose, float]:
        """
        Get the current pose and time of the object.
        """
        return self.tracked_object.pose, time.time()

    def detect_events(self) -> Optional[List[MotionEvent]]:
        """
        Detect if the object starts or stops moving.

        :return: An instance of the TranslationEvent class that represents the event if the object is moving, else None.
        """

        if not self.measure_timestep_passed:
            time.sleep(max(0.0, self.measure_timestep.total_seconds() - self.time_since_last_event))

        self.update_with_latest_motion_data()

        if not self.window_size_reached:
            return

        events: Optional[List[MotionEvent]] = None
        if self.motion_sate_changed:
            events = [self.update_motion_state_and_create_event()]

        if self.plot_distances:
            self.keep_track_of_history()

        return events

    def update_motion_state_and_create_event(self) -> MotionEvent:
        """
        Update the motion state of the object and create an event.
        :return: An instance of the MotionEvent class that represents the event.
        """
        self.was_moving = not self.was_moving
        self.update_object_motion_state(self.was_moving)
        self.update_start_pose_and_event_time()
        return self.create_event()

    @property
    def motion_sate_changed(self):
        """
        Check if the motion state of the object has changed.
        """
        return self.is_moving != self.was_moving

    def keep_track_of_history(self):
        """
        Keep track of the history of the object.
        """
        self.original_distances.append(self.latest_distances)
        if self.filtered_distances:
            self.all_filtered_distances.append(self.filtered_distances)
        self.all_times.append(self.latest_times)

    @property
    def time_since_last_event(self) -> float:
        return time.time() - self.latest_time

    @abstractmethod
    def update_object_motion_state(self, is_moving: bool) -> None:
        """
        Update the object motion state.

        :param is_moving: A boolean value that represents if the object is moving.
        """
        pass

    @property
    def is_moving(self) -> bool:
        """
        Check if the object is moving by using the motion detection method.

        :return: A boolean value that represents if the object is moving.
        """
        distances = self.filter_data() if self.filter else self.latest_distances
        return self.detection_method.is_moving(distances)

    def calculate_and_update_latest_distance(self):
        """
        Calculate the latest distance and time between the current pose and the previous pose.
        """
        distance = self.calculate_distance()
        self.latest_distances.append(distance)

    @property
    def measure_timestep_passed(self) -> bool:
        """
        :return: True if the measure timestep has passed since the last event.
        """
        return self.time_since_last_event >= self.measure_timestep.total_seconds()

    @property
    def window_size_reached(self) -> bool:
        return len(self.latest_distances) >= self.window_size

    def _crop_distances_and_times_to_window_size(self):
        self.latest_distances = self.latest_distances[-self.window_size:]
        self.latest_times = self.times[-self.window_size:]

    def _reset_distances_and_times(self):
        self.latest_distances = []
        self.latest_times = []

    def update_start_pose_and_event_time(self, index: int = 0):
        """
        Update the start pose and event time.

        :param index: The index of the latest pose, and time.
        """
        self.start_pose = self.poses[index]
        self.event_time = self.latest_times[index]

    def filter_data(self) -> np.ndarray:
        """
        Apply a preprocessing filter to the distances.
        """
        self.filtered_distances = self.filter.filter_data(np.array(self.latest_distances))
        return self.filtered_distances

    def create_event(self) -> MotionEvent:
        """
        Create a motion event.

        :return: An instance of the TranslationEvent class that represents the event.
        """
        current_pose, current_time = self.get_current_pose_and_time()
        event_type = self.get_event_type()
        event = event_type(self.tracked_object, self.start_pose, current_pose, timestamp=self.event_time)
        return event

    @abstractmethod
    def calculate_distance(self):
        pass

    @abstractmethod
    def get_event_type(self):
        pass

    def stop(self):
        """
        Stop the event detector.
        """
        # plot the distances
        if self.plot_distances and plt:
            self.plot_and_show_distances()

        if self.plot_distance_windows and plt:
            self.plot_and_show_distance_windows()

        super().stop()

    def plot_and_show_distances(self, plot_filtered: bool = True) -> None:
        """
        Plot the average distances.
        """
        plt.plot([t - self.times[0] for t in self.times], self.original_distances[:len(self.times)])
        if plot_filtered and self.all_filtered_distances:
            plt.plot([t - self.times[0] for t in self.times], self.all_filtered_distances[:len(self.times)])
        plt.title(f"Results of {self.__class__.__name__} for {self.tracked_object.name}")
        plt.show()

    def plot_and_show_distance_windows(self, plot_freq: bool = False) -> None:
        """
        Plot the distances and the frequencies of the distances.

        :param plot_freq: If True, plot the frequencies of the distances as well.
        """
        plot_cols: int = 2 if plot_freq else 1
        for i, window_time in enumerate(self.all_times):
            orig_distances = np.array(self.original_distances[i])
            times = np.array(window_time) - window_time[0]
            fig, axes = plt.subplots(3, plot_cols, figsize=(10, 10))
            self._add_distance_vs_filtered_to_plot(orig_distances, self.all_filtered_distances[i], times,
                                                   axes[:, 0] if plot_freq else axes)
            if plot_freq:
                self._add_frequencies_plot(orig_distances, axes[:, 1])
            plt.show()

    @staticmethod
    def _add_distance_vs_filtered_to_plot(distances: np.ndarray, filtered_distances: np.ndarray, times: np.ndarray,
                                          axes: np.ndarray) -> None:
        """
        Add the distances and the filtered distances to the figure.

        :param distances: The original distances.
        :param filtered_distances: The filtered distances.
        :param times: The times.
        :param axes: The axes to plot on.
        """
        ax_labels: List[str] = ["x", "y", "z"]
        for j, ax in enumerate(axes):
            original = distances[:, j]
            if np.mean(original) <= 1e-3:
                continue
            filtered = filtered_distances[:, j]
            ax.plot(times, original, label=f"original_{ax_labels[j]}")
            ax.plot(times[-len(filtered):], filtered, label=f"filtered_{ax_labels[j]}")
            ax.legend()

    def _add_frequencies_plot(self, distances: np.ndarray, axes: np.ndarray) -> None:
        """
        Add the frequencies plot to the figure.

        :param distances: The distances.
        :param axes: The axes to plot on.
        """
        for j, ax in enumerate(axes):
            xmag = np.fft.fft(distances[:, j])
            freqs = np.fft.fftfreq(len(xmag), d=self.measure_timestep.total_seconds())
            ax.bar(freqs[:len(xmag) // 2], np.abs(xmag)[:len(xmag) // 2], width=0.1)
            ax.legend()


class TranslationDetector(MotionDetector):

    def update_object_motion_state(self, is_moving: bool) -> None:
        """
        Update the object motion state.
        """
        self.tracked_object.is_translating = is_moving

    def calculate_distance(self):
        """
        Calculate the Euclidean distance between the latest and current positions of the object.
        """
        # return calculate_euclidean_distance(self.latest_pose.position_as_list(), current_pose.position_as_list())
        return calculate_translation(self.poses[-2].position_as_list(), self.poses[-1].position_as_list())

    def get_event_type(self):
        return TranslationEvent if self.was_moving else StopTranslationEvent


class RotationDetector(MotionDetector):

    def update_object_motion_state(self, moving: bool) -> None:
        """
        Update the object motion state.
        """
        self.tracked_object.is_rotating = moving

    def calculate_distance(self):
        """
        Calculate the angle between the latest and current quaternions of the object
        """
        quat_diff = calculate_quaternion_difference(self.poses[-2].orientation_as_list(),
                                                    self.poses[-1].orientation_as_list())
        # angle = 2 * np.arccos(quat_diff[0])
        euler_diff = list(euler_from_quaternion(quat_diff))
        euler_diff[2] = 0
        return euler_diff

    def get_event_type(self):
        return RotationEvent if self.was_moving else StopRotationEvent


class DetectorWithStarterEvent(PrimitiveEventDetector, ABC):
    """
    A type of event detector that requires an event to occur as a start condition.
    """

    def __init__(self, logger: EventLogger, starter_event: EventUnion, wait_time: Optional[float] = None,
                 *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the Event class that represents the event to start the event detector.
        :param wait_time: An optional float value that introduces a delay between calls to the event detector.
        """
        super().__init__(logger, wait_time, *args, **kwargs)
        self.starter_event: EventUnion = starter_event
        self._start_timestamp = self.starter_event.timestamp

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
        return self._start_timestamp

    @start_timestamp.setter
    def start_timestamp(self, timestamp: float):
        self._start_timestamp = timestamp

    def check_for_event_pre_starter_event(self, event_type: Type[Event],
                                          time_tolerance: timedelta) -> Optional[EventUnion]:
        """
        Check if the tracked_object was involved in an event before the starter event.

        :param event_type: The event type to check for.
        :param time_tolerance: The time tolerance to consider the event as before the starter event.
        """
        event = self.object_tracker.get_first_event_of_type_before_event(event_type,
                                                                         self.starter_event)
        if event is not None and self.start_timestamp - event.timestamp <= time_tolerance.total_seconds():
            return event
        else:
            self._no_event_found_log(event_type)

    def check_for_event_post_starter_event(self, event_type: Type[Event]) -> Optional[EventUnion]:
        """
        Check if the tracked_object was involved in an event after the starter event.

        :param event_type: The event type to check for.
        :return: The event if the tracked_object was involved in an event, else None.
        """
        event = self.object_tracker.get_first_event_of_type_after_event(event_type, self.starter_event)
        if event is None:
            self._no_event_found_log(event_type)
        return event

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
            self._no_event_found_log(event_type)
        return event

    def _no_event_found_log(self, event_type: Type[Event]):
        logdebug(f"{self} with starter event: {self.starter_event} found no event of type: {event_type}")


class AbstractAgentObjectInteractionDetector(DetectorWithStarterEvent, ABC):
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
        event = None
        while not self.kill_event.is_set():

            if not self.interaction_checks():
                time.sleep(0.01)
                continue

            self.interaction_event.timestamp = self.start_timestamp
            self.interaction_event.end_timestamp = self.end_timestamp
            event = self.interaction_event
            break

        if event:
            rospy.loginfo(f"{self.__class__.__name__} detected an interaction with: {self.tracked_object.name}")
            return [event]

        return []

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
                and any(select_transportable_objects_from_loss_of_contact_event(event))
                and check_if_in_contact_with_support(event.tracked_object, event.links))

    def interaction_checks(self) -> bool:
        """
        Check for upward motion after the object lost contact with the surface.
        """
        print(f"checking if {self.tracked_object.name} was picked up")
        # wait for the object to be lifted TODO: Should be replaced with a wait on a lifting event
        dt = timedelta(milliseconds=1000)
        time.sleep(dt.total_seconds())
        print(f"checking for translation event for {self.tracked_object.name}")
        latest_event = self.check_for_event_near_starter_event(TranslationEvent, dt)

        if latest_event:
            self.start_timestamp = min(latest_event.timestamp, self.start_timestamp)
            self.end_timestamp = max(latest_event.timestamp, self.start_timestamp)
            return True

        self.kill_event.set()
        return False


class PlacingDetector(AbstractAgentObjectInteractionDetector):
    """
    An abstract detector that detects if the tracked_object was placed by the agent.
    """

    thread_prefix = "placing_"

    def _init_interaction_event(self) -> EventUnion:
        return PlacingEvent(self.tracked_object, timestamp=self.start_timestamp)

    def interaction_checks(self) -> bool:
        dt = timedelta(milliseconds=1000)
        event = self.check_for_event_near_starter_event(StopMotionEvent, dt)
        if event is not None:
            # start_motion_event_type = TranslationEvent if isinstance(event, StopTranslationEvent) else RotationEvent
            start_motion_event = self.object_tracker.get_first_event_of_type_before_event(MotionEvent, event)
            self.start_timestamp = min(start_motion_event.timestamp, self.start_timestamp)
            self.end_timestamp = max(event.timestamp, self.starter_event.timestamp)
            return True
        elif time.time() - self.start_timestamp > dt.total_seconds():
            self.kill_event.set()
        return False

    @classmethod
    def get_object_to_track_from_starter_event(cls, starter_event: ContactEvent) -> Object:
        return starter_event.tracked_object

    @classmethod
    def start_condition_checker(cls, event: Event) -> bool:
        """
        Check if an agent is in contact with the tracked_object.

        :param event: The ContactEvent instance that represents the contact event.
        """
        logdebug(f"checking if {event} with object {event.tracked_object.name} is a starter event")
        if (isinstance(event, ContactEvent) and any(select_transportable_objects([event.tracked_object]))
                and check_if_in_contact_with_support(event.tracked_object, event.links)):
            return True
        return False


def check_for_supporting_surface(tracked_object: Object,
                                 possible_surfaces: Optional[List[Object]] = None) -> Optional[Object]:
    """
    Check if any of the possible surfaces are supporting the tracked_object.

    :param tracked_object: An instance of the Object class that represents the tracked_object to check.
    :param possible_surfaces: A list of Object instances that represent the possible surfaces.
    :return: An instance of the Object class that represents the supporting surface if found, else None.
    """
    with UseProspectionWorld():
        dt = 0.1
        World.current_world.simulate(dt)
        prospection_obj = World.current_world.get_prospection_object_for_object(tracked_object)
        contact_points = prospection_obj.contact_points
        contacted_bodies = contact_points.get_objects_that_have_points()
        contacted_body_names = [body.name for body in contacted_bodies]
        contacted_bodies = dict(zip(contacted_body_names, contacted_bodies))
        if possible_surfaces is None:
            possible_surface_names = contacted_body_names
        else:
            possible_surface_names = [obj.name for obj in possible_surfaces]
            possible_surface_names = list(set(contacted_body_names).intersection(possible_surface_names))
    supporting_surface = None
    opposite_gravity = [0, 0, 1]
    smallest_angle = np.pi / 8
    for obj_name in possible_surface_names:
        obj = World.current_world.get_object_by_name(obj_name)
        normals = contact_points.get_normals_of_object(contacted_bodies[obj_name])
        normal = np.mean(np.array(normals), axis=0)
        angle = get_angle_between_vectors(normal, opposite_gravity)
        if 0 <= angle <= smallest_angle:
            smallest_angle = angle
            supporting_surface = obj
    if supporting_surface is not None:
        print("found surface ", supporting_surface.name)
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
    return select_transportable_objects([event.tracked_object])


def select_transportable_objects(objects: List[Object]) -> List[Object]:
    """
    Select the objects that can be transported

    :param objects: A list of Object instances.
    """
    transportable_objects = [obj for obj in objects
                             if not issubclass(obj.obj_type, (pycrap.Agent, pycrap.Location, pycrap.Supporter))]
    return transportable_objects


EventDetectorUnion = Union[ContactDetector, LossOfContactDetector, LossOfSurfaceDetector, MotionDetector,
                           TranslationDetector, RotationDetector, NewObjectDetector, AgentPickUpDetector,
                           MotionPickUpDetector, DetectorWithStarterEvent]
TypeEventDetectorUnion = Union[Type[ContactDetector], Type[LossOfContactDetector], Type[LossOfSurfaceDetector],
                               Type[MotionDetector], Type[TranslationDetector], Type[RotationDetector],
                               Type[NewObjectDetector], Type[AgentPickUpDetector], Type[MotionPickUpDetector],
                               Type[DetectorWithStarterEvent]]
