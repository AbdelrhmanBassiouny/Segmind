import threading
import time
from datetime import timedelta

from typing_extensions import Optional
from enum import Enum
from ..episode_player import EpisodePlayer
import logging

# logging.basicConfig(level=logging.INFO)
# logdebug = logging.debug
# loginfo = logging.info


from pycram.datastructures.enums import Grasp, TorsoState, TaskStatus
from pycram.datastructures.grasp import GraspDescription
from pycram.robot_plans import (
    PickUpActionDescription,
    MoveTorsoActionDescription,
)
from pycram.language import SequentialPlan

from pycram.plan import Plan
from pycram.process_module import ProcessModule, simulated_robot

from semantic_digital_twin.world import World
from semantic_digital_twin.robots.abstract_robot import Arm
from semantic_digital_twin.world_description.world_entity import Body


class ExecutionStatus(Enum):
    RUNNING = 1
    PAUSED = 2


class CRAMPlayer(EpisodePlayer):
    world: World

    def __init__(
        self,
        world: Optional[World] = None,
        time_between_frames: timedelta = timedelta(seconds=0.5),
    ):
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
            self.world = World()

    def _pause(self):
        logdebug("Pausing Plan")
        Plan.status = TaskStatus.PAUSED

    def _resume(self):
        logdebug("Resuming Plan")
        Plan.status = TaskStatus.RUNNING

    def _run(self):
        self.ready = True
        object_description = Body(names=["milk"])
        description = PickUpActionDescription(
            object_description, [Arm.LEFT], [GraspDescription(Grasp.FRONT)]
        )
        with simulated_robot:
            plan = SequentialPlan(
                MoveTorsoActionDescription(TorsoState.HIGH), description
            )
            plan.perform()
        plan.plot()
        # while self.world.current_world is not None:
        #     if self.kill_event.is_set():
        #         break
        #     time.sleep(1)
