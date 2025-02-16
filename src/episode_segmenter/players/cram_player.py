import threading

from typing_extensions import Optional

from pycram.datastructures.world import World
from pycram.worlds.bullet_world import BulletWorld
from ..episode_player import EpisodePlayer


class CRAMPlayer(EpisodePlayer):
    world: World

    def __init__(self, world: Optional[World] = None):
        super().__init__()
        self._init_world(world)
        self.kill_event = threading.Event()

    def _init_world(self, world: Optional[World] = None):
        """
        Initialize the world for the CRAM player.
        """
        self.world = world if world else World.current_world
        if not self.world:
            self.world = BulletWorld()

    def run(self):
        self.ready = True
        while not self.kill_event.is_set():
            self._wait_to_maintain_frame_rate(self.time_between_frames.total_seconds())
