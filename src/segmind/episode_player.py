from __future__ import annotations

import datetime
import threading
from threading import RLock
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass

from pycram.datastructures.dataclasses import FrozenWorldState
from sqlalchemy import Select
from typing_extensions import Callable, Any, Optional, Dict, Generator, List
from pycram.ros import logdebug
from pycram.datastructures.world import World

from .datastructures.events import EventWithOneTrackedObject, Event
from .datastructures.mixins import HasPrimaryTrackedObject

try:
    from pycram.worlds.multiverse import Multiverse
except ImportError:
    Multiverse = None

try:
    from ripple_down_rules.user_interface.gui import RDRCaseViewer
except ImportError:
    RDRCaseViewer = None

from .utils import PropagatingThread
from .datastructures.enums import PlayerStatus
from .event_logger import EventLogger


class EpisodePlayer(PropagatingThread, ABC):
    """
    A class that represents the thread that steps the world.
    """

    current_player: Optional[EpisodePlayer] = None
    pause_resume_lock: RLock = RLock()
    
    def __new__(cls, *args, **kwargs):
        if cls.current_player is None:
            cls.current_player = super().__new__(cls)
            EpisodePlayer.current_player = cls.current_player
            # Initialize only once when instance is first created
            cls.current_player._initialized = False
        return cls.current_player

    def __init__(self, time_between_frames: Optional[datetime.timedelta] = None, use_realtime: bool = False,
                 stop_after_ready: bool = False, world: Optional[World] = None,
                 rdr_viewer: Optional[RDRCaseViewer] = None):
        if not self._initialized:
            super().__init__()
            self.rdr_viewer: Optional[RDRCaseViewer] = rdr_viewer
            self.stop_after_ready: bool = stop_after_ready
            self.world: World = world if world is not None else World.current_world
            self._ready: bool = False
            self._status = PlayerStatus.CREATED
            self.time_between_frames: datetime.timedelta = time_between_frames if time_between_frames is not None else datetime.timedelta(seconds=0.01)
            self.use_realtime: bool = use_realtime
            self._initialized = True
            self.original_state: Optional[FrozenWorldState] = None

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
        if value and self.stop_after_ready:
            self._status = PlayerStatus.STOPPED

    def run(self):
        self._status = PlayerStatus.PLAYING
        super().run()

    def go_to_moment_of_event(self, event: HasPrimaryTrackedObject, event_history: Optional[List[Event]] = None):
        """
        Set the world state to be the one at which the given event occurred.

        :param event: The event to set the world state to.
        :param event_history: (Optional) The event history that replaces the current timeline.
        """
        self.pause()
        self.original_state = self.world.frozen_copy()
        self.world.set_state_from_frozen_cp(event.world_frozen_cp)
        if event_history is not None:
            EventLogger.current_logger.set_timeline(event_history)

    def go_to_original_world_state(self, resume: bool = False):
        """
        Set the world state to be the one saved in the original world state.

        :param resume: (Optional) Whether to resume the episode player frame processing.
        """
        if self.original_state is not None:
            self.world.set_state_from_frozen_cp(self.original_state)
        if resume and self.status == PlayerStatus.PAUSED:
            self.resume()

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
        while self.status == PlayerStatus.PAUSED and not self.kill_event.is_set():
            time.sleep(0.1)

    def _wait_to_maintain_frame_rate(self, last_processing_time: float, delta_time: Optional[datetime.timedelta] = None):
        """
        Wait to maintain the frame rate of the episode player.

        :param last_processing_time: The time of the last processing.
        """
        if delta_time is None:
            delta_time = self.time_between_frames
        time_to_wait = datetime.timedelta(seconds=time.time() - last_processing_time)
        if delta_time < time_to_wait:
            time.sleep((time_to_wait - delta_time).total_seconds())

    @classmethod
    def pause_resume(cls, func: Callable) -> Callable:
        """
        A decorator for pausing the player before a function call and then resuming it after the call ends.

        :param func: The callable to wrap with the decorator.
        :return: The wrapped callable
        """
        def wrapper(*args, **kwargs) -> Any:
            with cls.pause_resume_lock:
                if cls.current_player.status == PlayerStatus.PLAYING:
                    logdebug("Pausing player")
                    cls.current_player.pause()
                    result = func(*args, **kwargs)
                    cls.current_player.resume()
                    logdebug("Resuming player")
                    return result
                else:
                    return func(*args, **kwargs)
        return wrapper
    
    def _join(self, timeout=None):
        self.current_player = None
