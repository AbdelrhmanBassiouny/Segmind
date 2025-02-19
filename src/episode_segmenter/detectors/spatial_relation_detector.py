from queue import Queue, Empty

from typing_extensions import List, Optional

from pycram.datastructures.enums import AdjacentBodyMethod as ABM
from pycram.datastructures.world_entity import PhysicalBody
from pycram.ros import logdebug, loginfo
from .atomic_event_detectors import AtomicEventDetector
from ..datastructures.events import MotionEvent, TranslationEvent, EventUnion, StopMotionEvent


class SpatialRelationDetector(AtomicEventDetector):
    """
    A class that detects spatial relations between objects.
    """

    def __init__(self, check_on_events: Optional[List[EventUnion]] = None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.event_queue: Queue[MotionEvent] = Queue()
        self.queues.append(self.event_queue)
        self.check_on_events = check_on_events if check_on_events is not None else [StopMotionEvent]
        for event in self.check_on_events:
            self.logger.add_callback(event, self.on_motion_event)

    def on_motion_event(self, event: MotionEvent):
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
