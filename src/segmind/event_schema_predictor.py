import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import timedelta

from build.lib.ripple_down_rules.rdr_decorators import RDRDecorator
from .event_logger import EventLogger
from pycram.datastructures.world import World


@dataclass
class EventSchemaPredictor(ABC):
    event_logger: EventLogger = field(default_factory=EventLogger)
    world: World = field(default=World.current_world)
    thread: Thread = field(init=False)
    kill_event: threading.Event = threading.Event()
    step_time: timedelta = field(default=timedelta(milliseconds=100))
    min_step_time: timedelta = field(default=timedelta(milliseconds=50))

    def __post_init__(self):
        self.thread = Thread(target=self.prediction_loop)

    def start(self):
        self.thread.start()

    def prediction_loop(self):
        while not self.kill_event.is_set():
            start_time = time.time()
            self.predict_next_event()
            current_time = time.time()
            time_diff = current_time - start_time
            time.sleep(max(self.min_step_time, self.step_time - time_diff))

    @abstractmethod
    def predict_next_event(self) -> Event:
        pass


event_predictor_rdr: RDRDecorator = RDRDecorator()

@dataclass
class RDREventSchemaPredictor(ABC):
    pass
