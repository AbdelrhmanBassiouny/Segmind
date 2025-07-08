import os.path
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import timedelta
from os.path import dirname

from ripple_down_rules.rdr_decorators import RDRDecorator
from typing_extensions import Optional

from .datastructures.enums import TaskType
from .datastructures.events import EventUnion, Event
from .episode_player import EpisodePlayer
from .event_logger import EventLogger
from pycram.datastructures.world import World


@dataclass
class EventSchemaPredictor(ABC):
    task_type: Optional[TaskType] = field(default=None)
    task_description: Optional[str] = field(default=None)
    event_logger: EventLogger = field(default_factory=EventLogger)
    world: World = field(default=World.current_world)
    thread: threading.Thread = field(init=False)
    kill_event: threading.Event = threading.Event()
    step_time: timedelta = field(default=timedelta(milliseconds=100))
    min_step_time: timedelta = field(default=timedelta(milliseconds=50))

    def __post_init__(self):
        self.thread = threading.Thread(target=self.prediction_loop)
        if self.task_description is None and self.task_type is not None:
            self.task_description = self.task_type.value

    def start(self):
        self.thread.start()

    def prediction_loop(self):
        while not self.kill_event.is_set():
            start_time = time.time()
            predicted_event = self.predict_next_event()
            if self.is_prediction_wrong(predicted_event):
                if self.is_task_correct():
                    current_task.fit_case()
            current_time = time.time()
            time_diff = current_time - start_time
            time.sleep(max(self.min_step_time.total_seconds(), self.step_time.total_seconds() - time_diff))

    @abstractmethod
    def predict_next_event(self) -> EventUnion:
        pass


    def is_prediction_wrong(self, predicted_event: EventUnion) -> bool:
        """
        Check if the predicted event does not match the actual event in the world.
        This method should be overridden by subclasses to implement specific logic.
        """
        raise NotImplementedError("Subclasses must implement this method.")

model_dir = os.path.join(dirname(__file__), "rdrs")
event_predictor_rdr: RDRDecorator = RDRDecorator(model_dir, (Event,), False,
                                                 fit=True, fitting_decorator=EpisodePlayer.pause_resume)

@dataclass
class RDREventSchemaPredictor(ABC):
    pass
