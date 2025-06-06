from __future__ import annotations

import datetime
import threading
from threading import RLock
import time
from abc import ABC, abstractmethod
from typing_extensions import Callable, Any, Optional
from pycram.ros import logdebug

try:
    from pycram.worlds.multiverse import Multiverse
except ImportError:
    Multiverse = None


from .utils import singleton
from .datastructures.enums import PlayerStatus


class EpisodePlayer(threading.Thread, ABC):
    """
    A class that represents the thread that steps the world.
    """

    _instance: Optional[EpisodePlayer] = None
    pause_resume_lock: RLock = RLock()
    
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            EpisodePlayer._instance = cls._instance
            # Initialize only once when instance is first created
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, time_between_frames: datetime.timedelta = datetime.timedelta(milliseconds=10)):
        if not self._initialized:
            super().__init__()
            self._ready: bool = False
            self._status = PlayerStatus.CREATED
            self.time_between_frames: datetime.timedelta = time_between_frames
            self._initialized = True

    @property
    def status(self):
        """
        :return: The current status of the episode player.
        :rtype: PlayerStatus
        """
        return self._status

    @property
    def ready(self):
        return self._ready

    @ready.setter
    def ready(self, value: bool):
        self._ready = value

    def run(self):
        self._status = PlayerStatus.PLAYING
        self._run()

    @abstractmethod
    def _run(self):
        """
        The run method that is called when the thread is started. This should start the episode player thread.
        """
        pass
    
    def pause(self):
        """
        Pause the episode player frame processing.
        """
        self._status = PlayerStatus.PAUSED
        self._pause()

    @abstractmethod
    def _pause(self):
        """
        Perform extra functionalities when the episode player is paused
        """
        pass

    def resume(self):
        """
        Resume the episode player frame processing.
        """
        self._status = PlayerStatus.PLAYING
        self._resume()
    
    @abstractmethod
    def _resume(self):
        """
        Perform extra functionalities when the episode player is resumed
        """
        pass

    def _wait_if_paused(self):
        """
        Wait if the episode player is paused.
        """
        while self.status == PlayerStatus.PAUSED:
            time.sleep(0.1)

    def _wait_to_maintain_frame_rate(self, last_processing_time: float):
        """
        Wait to maintain the frame rate of the episode player.

        :param last_processing_time: The time of the last processing.
        """
        time_diff = time.time() - last_processing_time
        if time_diff < self.time_between_frames.total_seconds():
            time.sleep(self.time_between_frames.total_seconds() - time_diff)

    @classmethod
    def pause_resume(cls, func: Callable) -> Callable:
        """
        A decorator for pausing the player before a function call and then resuming it after the call ends.

        :param func: The callable to wrap with the decorator.
        :return: The wrapped callable
        """
        def wrapper(*args, **kwargs) -> Any:
            with cls.pause_resume_lock:
                if cls._instance.status == PlayerStatus.PLAYING:
                    logdebug("Pausing player")
                    cls._instance.pause()
                    result = func(*args, **kwargs)
                    cls._instance.resume()
                    logdebug("Resuming player")
                    return result
                else:
                    return func(*args, **kwargs)
        return wrapper
