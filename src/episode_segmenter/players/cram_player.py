import threading
from datetime import timedelta

from typing_extensions import Optional

from pycram.datastructures.pose import Pose
from pycram.datastructures.world import World
from pycram.process_module import ProcessModule
from pycram.worlds.bullet_world import BulletWorld
from pycrap.ontologies import Robot, Kitchen
from pycram.world_concepts.world_object import Object
from ..episode_player import EpisodePlayer


class CRAMPlayer(EpisodePlayer):
    world: World

    def __init__(self, world: Optional[World] = None,
                 time_between_frames: timedelta = timedelta(seconds=0.5)):
        """
        Initialize the CRAM player.

        :param world: The pycram world to use.
        :param time_between_frames: The time between frames.
        """
        super().__init__(time_between_frames)
        self._init_world(world)
        self.kill_event = threading.Event()
        ProcessModule.execution_delay = time_between_frames

    def _init_world(self, world: Optional[World] = None):
        """
        Initialize the world for the CRAM player.
        """
        self.world = world if world else World.current_world
        if not self.world:
            self.world = BulletWorld()

    def run(self):
        self.ready = True
