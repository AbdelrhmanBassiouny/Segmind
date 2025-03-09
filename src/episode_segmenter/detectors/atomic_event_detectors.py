import queue
import threading
import time
from abc import ABC, abstractmethod
from datetime import timedelta
from math import ceil
from queue import Queue

import numpy as np

from ..datastructures.mixins import HasPrimaryTrackedObject, HasSecondaryTrackedObject

try:
    from matplotlib import pyplot as plt
except ImportError:
    plt = None
from tf.transformations import euler_from_quaternion
from typing_extensions import Optional, List, Union, Type, Tuple, Callable

from pycram import World
from pycram.datastructures.dataclasses import ContactPointsList
from pycram.datastructures.pose import Pose
from pycram.datastructures.world_entity import PhysicalBody
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import PhysicalObject, Agent
from ..event_logger import EventLogger
from ..datastructures.events import Event, ContactEvent, LossOfContactEvent, AgentContactEvent, \
    AgentLossOfContactEvent, LossOfSurfaceEvent, TranslationEvent, StopTranslationEvent, NewObjectEvent, \
    RotationEvent, StopRotationEvent, MotionEvent
from .motion_detection_helpers import ConsistentGradient, MotionDetectionMethod, DataFilter
from ..utils import calculate_quaternion_difference, \
    get_support, calculate_translation


class AtomicEventDetector(threading.Thread, ABC):
    """
    A thread that detects events in another thread and logs them. The event detector is a function that has no arguments
    and returns an object that represents the event. The event detector is called in a loop until the thread is stopped
    by setting the exit_thread attribute to True.
    """

    def __init__(self, logger: Optional[EventLogger] = None, wait_time: Optional[timedelta] = None,
                 world: Optional[World] = None, *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param wait_time: An optional timedelta value that introduces a delay between calls to the event detector.
        :param world: An optional World instance that represents the world.
        """

        super().__init__()

        self.logger = logger if logger else EventLogger.current_logger
        self.world = world if world else World.current_world
        self.wait_time = wait_time if wait_time is not None else timedelta(seconds=0.1)

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
        return all([q.empty() for q in self.queues])

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
        time_diff = time.time() - last_processing_time
        if time_diff < self.wait_time.total_seconds():
            time.sleep(self.wait_time.total_seconds() - time_diff)

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


class NewObjectDetector(AtomicEventDetector):
    """
    A thread that detects if a new object is added to the scene and logs the NewObjectEvent.
    """

    def __init__(self, logger: EventLogger, wait_time: Optional[timedelta] = None,
                 avoid_objects: Optional[Callable[[Object], bool]] = None, *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param wait_time: An optional timedelta value that introduces a delay between calls to the event detector.
        :param avoid_objects: An optional list of strings that represent the names of the objects to avoid.
        """
        super().__init__(logger, wait_time, *args, **kwargs)
        self.new_object_queue: Queue[Object] = Queue()
        self.queues.append(self.new_object_queue)
        self.avoid_objects = avoid_objects if avoid_objects else lambda obj: False
        self.world.add_callback_on_add_object(self.on_add_object)

    def on_add_object(self, obj: Object):
        """
        Callback function that is called when a new object is added to the scene.
        """
        if not self.avoid_objects(obj):
            self.new_object_queue.put(obj)

    def detect_events(self) -> List[Event]:
        """
        Detect if a new object is added to the scene and invoke the NewObjectEvent.

        :return: A NewObjectEvent that represents the addition of a new object to the scene.
        """
        events = []
        try:
            while True:
                event = NewObjectEvent(self.new_object_queue.get_nowait())
                self.new_object_queue.task_done()
                events.append(event)
        except queue.Empty:
            return events

    def stop(self):
        """
        Remove the callback on the add object event and resume the thread to be able to join.
        """
        World.current_world.remove_callback_on_add_object(self.on_add_object)
        super().stop()

    def __str__(self):
        return self.thread_id


class DetectorWithTrackedObject(AtomicEventDetector, HasPrimaryTrackedObject, ABC):
    """
    A mixin class that provides one tracked object for the event detector.
    """

    def __init__(self, logger: EventLogger, tracked_object: Object, wait_time: Optional[timedelta] = None,
                 *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param tracked_object: An Object instance that represents the object to track.
        :param wait_time: An optional timedelta value that introduces a delay between calls to the event detector.
        """
        HasPrimaryTrackedObject.__init__(self, tracked_object)
        AtomicEventDetector.__init__(self, logger, wait_time, *args, **kwargs)

    def __str__(self):
        return f"{self.thread_id} - {self.tracked_object.name}"


class DetectorWithTwoTrackedObjects(DetectorWithTrackedObject, HasSecondaryTrackedObject, ABC):
    """
    A mixin class that provides two tracked objects for the event detector.
    """

    def __init__(self, logger: EventLogger, tracked_object: Object, with_object: Optional[Object] = None,
                 wait_time: Optional[timedelta] = None, *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param tracked_object: An Object instance that represents the object to track.
        :param with_object: An optional Object instance that represents the object to track.
        :param wait_time: An optional timedelta value that introduces a delay between calls to the event detector.
        """
        DetectorWithTrackedObject.__init__(self, logger, tracked_object, wait_time, *args, **kwargs)
        HasSecondaryTrackedObject.__init__(self, with_object)

    def __str__(self):
        with_object_name = f" - {self.with_object.name}" if self.with_object is not None else ""
        return super().__str__() + with_object_name


class AbstractContactDetector(DetectorWithTwoTrackedObjects, ABC):
    def __init__(self, logger: EventLogger, tracked_object: Object,
                 with_object: Optional[Object] = None,
                 max_closeness_distance: Optional[float] = 0.05,
                 wait_time: Optional[timedelta] = timedelta(seconds=0.01),
                 *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the Event class that represents the event to start the event detector.
        :param max_closeness_distance: An optional float value that represents the maximum distance between the object
        :param wait_time: An optional timedelta value that introduces a delay between calls to the event detector.
        """
        DetectorWithTwoTrackedObjects.__init__(self, logger, tracked_object, with_object, wait_time,
                                               *args, **kwargs)
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
        event_type = AgentContactEvent if issubclass(self.obj_type, Agent) else ContactEvent
        return [event_type(contact_points.get_points_of_object(new_obj),
                           of_object=self.tracked_object, with_object=new_obj)
                for new_obj in new_objects_in_contact]


class LossOfContactDetector(AbstractContactDetector):
    """
    A thread that detects if the object lost contact with another object.
    """

    def trigger_events(self, contact_points: ContactPointsList) -> List[LossOfContactEvent]:
        """
        Check if the object lost contact with another object.

        :param contact_points: The current contact points.
        :return: An instance of the LossOfContactEvent/AgentLossOfContactEvent class that represents the event if the
         object lost contact, else None.
        """
        bodies_that_lost_contact = self.get_bodies_that_lost_contact(contact_points)
        if len(bodies_that_lost_contact) == 0:
            return []
        event_type = AgentLossOfContactEvent if issubclass(self.obj_type, Agent) else LossOfContactEvent
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
        supporting_surface = get_support(self.tracked_object,
                                         bodies_that_lost_contact)
        if supporting_surface is None:
            return []
        return [LossOfSurfaceEvent(contact_points, self.latest_contact_points, of_object=self.tracked_object,
                                   surface=supporting_surface)]


class MotionDetector(DetectorWithTrackedObject, ABC):
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
                 time_between_frames: timedelta = timedelta(milliseconds=50),
                 window_size: int = 7,
                 distance_filter_method: Optional[DataFilter] = None,
                 *args, **kwargs):
        """
        :param logger: An instance of the EventLogger class that is used to log the events.
        :param starter_event: An instance of the NewObjectEvent class that represents the event to start the event.
        :param tracked_object: An optional Object instance that represents the object to track.
        :param detection_method: The motion detection method that is used to detect if the object is moving.
        :param time_between_frames: The time between frames of episode player.
        :param window_size: The size of the window that is used to calculate the distances (must be > 1).
        :param distance_filter_method: An optional DataFilter instance that is used to filter the distances.
        """
        DetectorWithTrackedObject.__init__(self, logger, tracked_object, *args, **kwargs)
        self.time_between_frames: timedelta = time_between_frames
        self.measure_timestep: timedelta = 2 * time_between_frames
        self.window_size: int = window_size
        self.filter: Optional[DataFilter] = distance_filter_method
        self.detection_method: MotionDetectionMethod = detection_method

        self._init_event_data()

        self._init_data_holders()

        self.was_moving: bool = False

        self.plot_distances: bool = False
        self.plot_distance_windows: bool = False
        self.plot_frequencies: bool = False

    @property
    def window_size(self) -> int:
        return self._window_size

    @window_size.setter
    def window_size(self, window_size: int):
        if window_size < 2:
            raise ValueError("The window size must be greater than 1.")
        self._window_size = window_size

    def get_n_changes_wait_time(self, n: int) -> float:
        """
        :param n: The number of successive changes in motion state.
        :return: The minimum wait time for detecting n successive changes in motion state.
        """
        return self.window_timeframe.total_seconds() * n + self.wait_time.total_seconds()

    def _init_event_data(self):
        """
        Initialize the event time and start pose of the object/event.
        """
        self.latest_pose, self.latest_time = self.get_current_pose_and_time()
        self.event_time: float = self.latest_time
        self.start_pose: Pose = self.latest_pose

    @property
    def window_timeframe(self) -> timedelta:
        return self.measure_timestep * self.window_size

    @property
    def measure_timestep(self) -> timedelta:
        return self._measure_timestep

    @measure_timestep.setter
    def measure_timestep(self, measure_timestep: timedelta):
        """
        Update the measure timestep and the wait time between calls to the event detector.
        """
        # frames per measure timestep
        self.measure_frame_rate: float = ceil(measure_timestep.total_seconds() /
                                              self.time_between_frames.total_seconds())
        self._measure_timestep = self.time_between_frames * self.measure_frame_rate
        self.wait_time = self._measure_timestep

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
