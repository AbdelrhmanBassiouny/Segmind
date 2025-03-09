import threading
from datetime import timedelta

from typing_extensions import Optional, List, Type

from pycram.datastructures.world import World
from pycram.orm.action_designator import Action
from pycram.process_module import ProcessModule
from pycram.tasktree import task_tree, TaskTreeNode
from pycram.worlds.bullet_world import BulletWorld
from ..episode_player import EpisodePlayer


class CRAMPlayer(EpisodePlayer):
    world: World

    def __init__(self, action_types: Optional[List[Type[Action]]] = None, world: Optional[World] = None,
                 time_between_frames: timedelta = timedelta(seconds=0.5)):
        """
        Initialize the CRAM player.

        :param action_types: A list of action types to listen for, if None, all action types are listened for.
        :param world: The pycram world to use.
        :param time_between_frames: The time between frames.
        """
        super().__init__(time_between_frames)
        self.action_types: Optional[List[Type[Action]]] = action_types
        if self.action_types:
            for action_type in self.action_types:
                task_tree.add_callback(action_type)
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

    def action_callback(self, action_node: TaskTreeNode):
        """
        The action callback method that is called when an action is performed.

        :param action_node: The node in the task tree representing the action that was performed.
        """
        # Maybe create an Event for the given action.
        # One could use that for supervising the RDRs of action detection.
        # Maybe fit_rdr_case here :D.

    def run(self):
        self.ready = True
