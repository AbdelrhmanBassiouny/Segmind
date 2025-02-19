from __future__ import annotations

import queue
import threading
from collections import UserDict
from threading import RLock
import time
from datetime import timedelta

from typing_extensions import List, Optional, Dict, Type, TYPE_CHECKING, Callable

from pycram.datastructures.dataclasses import TextAnnotation
from pycram.datastructures.world import World
from pycram.ros import loginfo, logdebug
from pycram.world_concepts.world_object import Object, Link
from .datastructures.events import Event, EventUnion, EventWithTrackedObjects
from .datastructures.object_tracker import ObjectTrackerFactory

if TYPE_CHECKING:
    from .detectors.coarse_event_detectors import DetectorWithStarterEvent


class EventCallbacks(UserDict):
    """
    A dictionary that maps event types to a list of callbacks that should be called when the event occurs.
    This modifies the setitem such that if a class or its subclass is added, the callback is also added to the subclass.
    """

    def __setitem__(self, key: Type[Event], value: List[Callable[[Event], None]]):
        if key not in self:
            super().__setitem__(key, value)
        else:
            self[key].extend(value)
        for subclass in key.__subclasses__():
            self.__setitem__(subclass, value)


class EventLogger:
    """
    A class that logs events that are happening in the simulation.
    """

    current_logger: Optional['EventLogger'] = None
    """
    A singleton instance of the event logger.
    """
    event_callbacks: EventCallbacks = EventCallbacks()
    """
    A dictionary that maps event types to a list of callbacks that should be called when the event occurs.
    """

    def __init__(self, annotate_events: bool = False, events_to_annotate: List[Type[Event]] = None):
        self.timeline_per_thread = {}
        self.timeline = []
        self.event_queue = queue.Queue()
        self.timeline_lock: RLock = RLock()
        self.event_callbacks_lock: RLock = RLock()
        self.annotate_events = annotate_events
        self.events_to_annotate = events_to_annotate
        if annotate_events:
            self.annotation_queue = queue.Queue()
            self.annotation_thread = EventAnnotationThread(self)
            self.annotation_thread.start()
        if EventLogger.current_logger is None:
            EventLogger.current_logger = self

    def add_callback(self, event_type: Type[Event], callback: Callable[[Event], None]) -> None:
        """
        Add a callback for an event type.

        :param event_type: The type of the event.
        :param callback: The callback to add.
        """
        with self.event_callbacks_lock:
            self.event_callbacks[event_type] = [callback]

    def log_event(self, event: Event):
        if self.is_event_in_timeline(event):
            logdebug(f"Event {event} already logged.")
            return
        self.update_object_trackers_with_event(event)
        self.event_queue.put(event)
        self.annotate_scene_with_event(event)
        self.add_event_to_timeline_of_thread(event)
        self.call_event_callbacks(event)

    def call_event_callbacks(self, event: Event) -> None:
        """
        Call the callbacks that are registered for the event type.

        :param event: The event to call the callbacks for.
        """
        with self.event_callbacks_lock:
            if type(event) in self.event_callbacks:
                for callback in self.event_callbacks[type(event)]:
                    callback(event)

    def annotate_scene_with_event(self, event: Event) -> None:
        """
        Annotate the scene with the event.

        :param event: The event to annotate the scene with.
        """
        if self.annotate_events and (self.events_to_annotate is None or (type(event) in self.events_to_annotate)):
            self.annotation_queue.put(event)

    @staticmethod
    def update_object_trackers_with_event(event: Event) -> None:
        """
        Update the event object trackers with the event.

        :param event: The event to update the object trackers with.
        """
        if isinstance(event, EventWithTrackedObjects):
            event.update_object_trackers_with_event()

    def add_event_to_timeline_of_thread(self, event: Event) -> None:
        """
        Add an event to the timeline of the detector thread.
        :param event: The event to add.
        """
        thread_id = event.detector_thread_id
        with self.timeline_lock:
            if thread_id not in self.timeline_per_thread:
                self.timeline_per_thread[thread_id] = []
            self.timeline_per_thread[thread_id].append(event)
            self.timeline.append(event)

    def is_event_in_timeline(self, event: Event) -> bool:
        """
        Check if an event is already in the timeline.

        :param event: The event to check.
        :return: True if the event is in the timeline, False otherwise.
        """
        with self.timeline_lock:
            return event in self.timeline

    def plot_events(self):
        """
        Plot all events that have been logged in a timeline.
        """
        loginfo("Plotting events:")
        # construct a dataframe with the events
        import pandas as pd
        import plotly.express as px
        import plotly.graph_objects as go

        data_dict = {'start': [], 'end': [], 'event': [], 'object': [], 'obj_type': []}
        for tracker in ObjectTrackerFactory.get_all_trackers():
            for event in tracker.get_event_history():
                end_timestamp = event.timestamp + timedelta(seconds=0.1).total_seconds()
                if hasattr(event, 'end_timestamp') and event.end_timestamp is not None:
                    end_timestamp = max(event.end_timestamp, end_timestamp)
                data_dict['end'].append(end_timestamp)
                data_dict['start'].append(event.timestamp)
                data_dict['event'].append(event.__class__.__name__)
                data_dict['object'].append(tracker.obj.name)
                if isinstance(tracker.obj, Object):
                    data_dict['obj_type'].append(tracker.obj.obj_type.name)
                elif isinstance(tracker.obj, Link):
                    data_dict['obj_type'].append(f'Link of {tracker.obj.parent_entity.obj_type}')
        # subtract the start time from all timestamps
        min_start = min(data_dict['start'])
        data_dict['start'] = [x - min_start for x in data_dict['start']]
        data_dict['end'] = [x - min_start for x in data_dict['end']]
        df = pd.DataFrame(data_dict)

        fig = go.Figure()

        fig = px.timeline(df, x_start=pd.to_datetime(df[f'start'], unit='s'),
                          x_end=pd.to_datetime(df[f'end'], unit='s'),
                          y=f'event',
                          color=f'event',
                          hover_data={'object': True, 'obj_type': True},
                          # text=f'object',
                          title=f"Events Timeline")
        fig.update_xaxes(tickvals=pd.to_datetime(df[f'start'], unit='s'), tickformat='%S')
        fig.update_yaxes(showgrid=True, gridwidth=1, gridcolor='LightPink')
        fig.update_layout(
            font_family="Courier New",
            font_color="black",
            font_size=20,
            title_font_family="Times New Roman",
            title_font_color="black",
            title_font_size=30,
            legend_title_font_color="black",
            legend_title_font_size=24,
        )
        fig.show()

    def print_events(self):
        """
        Print all events that have been logged.
        """
        loginfo("Events:")
        loginfo(self.__str__())

    def get_events_per_thread(self) -> Dict[str, List[Event]]:
        """
        Get all events that have been logged.
        """
        with self.timeline_lock:
            events = self.timeline_per_thread.copy()
        return events

    def get_events(self) -> List[Event]:
        """
        Get all events that have been logged.
        """
        with self.timeline_lock:
            events = self.timeline.copy()
        return events

    def get_latest_event_of_detector_for_object(self, detector_prefix: str, obj: Object) -> Optional[Event]:
        """
        Get the latest of event of the thread that has the given prefix and object name in its id.

        :param detector_prefix: The prefix of the thread id.
        :param obj: The object that should have its name in the thread id.
        """
        thread_id = self.find_thread_with_prefix_and_object(detector_prefix, obj.name)
        return self.get_latest_event_of_thread(thread_id)

    def get_nearest_event_of_detector_for_object(self, detector_prefix: str, obj: Object,
                                                 timestamp: float) -> Optional[EventUnion]:
        """
        Get the nearest event of the thread that has the given prefix and object name in its id.

        :param detector_prefix: The prefix of the thread id.
        :param obj: The object that should have its name in the thread id.
        :param timestamp: The timestamp of the event.
        """
        thread_id = self.find_thread_with_prefix_and_object(detector_prefix, obj.name)
        return self.get_nearest_event_of_thread(thread_id, timestamp)

    def find_thread_with_prefix_and_object(self, prefix: str, object_name: str) -> Optional[str]:
        """
        Find the thread id that has the given prefix and object name in its id.

        :param prefix: The prefix of the thread id.
        :param object_name: The object name that should be in the thread id.
        :return: The id of the thread or None if no such thread
        """
        with self.timeline_lock:
            thread_id = [thread_id for thread_id in self.timeline_per_thread.keys() if thread_id.startswith(prefix) and
                         object_name in thread_id]
        return None if len(thread_id) == 0 else thread_id[0]

    def get_nearest_event_of_thread(self, thread_id: str, timestamp: float) -> Optional[EventUnion]:
        """
        Get the nearest event of the thread with the given id.

        :param thread_id: The id of the thread.
        :param timestamp: The timestamp of the event.
        :return: The nearest event of the thread or None if no such thread.
        """
        with self.timeline_lock:
            if thread_id not in self.timeline_per_thread:
                return None
            all_event_timestamps = [(event, event.timestamp) for event in self.timeline_per_thread[thread_id]]
            return min(all_event_timestamps, key=lambda x: abs(x[1] - timestamp))[0]

    def get_latest_event_of_thread(self, thread_id: str) -> Optional[Event]:
        """
        Get the latest event of the thread with the given id.

        :param thread_id: The id of the thread.
        :return: The latest event of the thread or None if no such thread.
        """
        with self.timeline_lock:
            if thread_id not in self.timeline_per_thread:
                return None
            return self.timeline_per_thread[thread_id][-1]

    def get_next_event(self):
        """
        Get the next event from the event queue.
        """
        try:
            event = self.event_queue.get(block=False)
            self.event_queue.task_done()
            return event
        except queue.Empty:
            return None

    def join(self):
        """
        Wait for all events to be processed and all annotations to be added.
        """
        if self.annotate_events:
            self.annotation_thread.stop()
            self.annotation_thread.join()
            self.annotation_queue.join()
        self.event_queue.join()

    def __str__(self):
        return '\n'.join([str(event) for event in self.get_events()])


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
        self.kill_event = threading.Event()

    def get_next_z_offset(self):
        return self.initial_z_offset - self.step_z_offset * len(self.current_annotations)

    def run(self):
        while not self.kill_event.is_set():
            try:
                event = self.logger.annotation_queue.get(block=False)
            except queue.Empty:
                time.sleep(0.01)
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
                                                               color=text_ann.color,
                                                               size=text_ann.size)
            z_offset = self.get_next_z_offset()
            text_ann = event.annotate([1.5, 1, z_offset])
            self.current_annotations.append(text_ann)
            time.sleep(0.01)

    def stop(self):
        self.kill_event.set()
