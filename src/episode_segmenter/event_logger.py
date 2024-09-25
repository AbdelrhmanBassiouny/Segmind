import queue
import threading
import time

from typing_extensions import List, Optional, Dict, Type

from pycram.datastructures.dataclasses import TextAnnotation
from pycram.datastructures.world import World
from pycram.world_concepts.world_object import Object

from .events import Event


class EventLogger:
    """
    A class that logs events that are happening in the simulation.
    """

    current_logger: Optional['EventLogger'] = None

    def __init__(self, annotate_events: bool = False, events_to_annotate: List[Type[Event]] = None):
        self.timeline = {}
        self.event_queue = queue.Queue()
        self.lock = threading.Lock()
        self.annotate_events = annotate_events
        self.events_to_annotate = events_to_annotate
        if annotate_events:
            self.annotation_queue = queue.Queue()
            self.annotation_thread = EventAnnotationThread(self)
            self.annotation_thread.start()
        if EventLogger.current_logger is None:
            EventLogger.current_logger = self

    def log_event(self, thread_id, event: Event):
        self.event_queue.put((thread_id, event))
        if self.annotate_events and (self.events_to_annotate is None or (type(event) in self.events_to_annotate)):
            self.annotation_queue.put(event)
        with self.lock:
            if thread_id not in self.timeline:
                self.timeline[thread_id] = []
            self.timeline[thread_id].append(event)

    def print_events(self):
        """
        Print all events that have been logged.
        """
        print("Events:")
        print(self)

    def get_events(self) -> Dict[str, List[Event]]:
        """
        Get all events that have been logged.
        """
        with self.lock:
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

    def find_thread_with_prefix_and_object(self, prefix: str, object_name: str) -> Optional[str]:
        """
        Find the thread id that has the given prefix and object name in its id.

        :param prefix: The prefix of the thread id.
        :param object_name: The object name that should be in the thread id.
        :return: The id of the thread or None if no such thread
        """
        with self.lock:
            thread_id = [thread_id for thread_id in self.timeline.keys() if thread_id.startswith(prefix) and
                         object_name in thread_id]
        return None if len(thread_id) == 0 else thread_id[0]

    def get_latest_event_of_thread(self, thread_id: str) -> Optional[Event]:
        """
        Get the latest event of the thread with the given id.

        :param thread_id: The id of the thread.
        :return: The latest event of the thread or None if no such thread.
        """
        with self.lock:
            if thread_id not in self.timeline:
                return None
            return self.timeline[thread_id][-1]

    def get_next_event(self):
        """
        Get the next event from the event queue.
        """
        try:
            thread_id, event = self.event_queue.get(block=False)
            self.event_queue.task_done()
            return thread_id, event
        except queue.Empty:
            return None, None

    def join(self):
        """
        Wait for all events to be processed and all annotations to be added.
        """
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
                                                               color=text_ann.color,
                                                               size=text_ann.size)
            z_offset = self.get_next_z_offset()
            text_ann = event.annotate([1.5, 1, z_offset])
            self.current_annotations.append(text_ann)
            time.sleep(0.1)

    def stop(self):
        self.exit = True
        self.join()
