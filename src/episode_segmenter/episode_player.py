import datetime
import threading
import time
from abc import ABC, abstractmethod

try:
    from pycram.worlds.multiverse import Multiverse
except ImportError:
    Multiverse = None


class EpisodePlayer(threading.Thread, ABC):
    def __init__(self, time_between_frames: datetime.timedelta = datetime.timedelta(milliseconds=10)):
        super().__init__()
        self._ready = False
        self._pause: bool = False
        self.time_between_frames: datetime.timedelta = time_between_frames

    @property
    def ready(self):
        return self._ready

    @ready.setter
    def ready(self, value: bool):
        self._ready = value

    @abstractmethod
    def run(self):
        """
        The run method that is called when the thread is started. This should start the episode player thread.
        """
        pass

    def pause(self):
        """
        Pause the episode player frame processing.
        """
        self._pause: bool = True

    def resume(self):
        """
        Resume the episode player frame processing.
        """
        self._pause: bool = False

    def _wait_if_paused(self):
        """
        Wait if the episode player is paused.
        """
        while self._pause:
            time.sleep(0.1)

    def _wait_to_maintain_frame_rate(self, last_processing_time: float):
        """
        Wait to maintain the frame rate of the episode player.

        :param last_processing_time: The time of the last processing.
        """
        time_diff = time.time() - last_processing_time
        if time_diff < self.time_between_frames.total_seconds():
            time.sleep(self.time_between_frames.total_seconds() - time_diff)
