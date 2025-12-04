import threading
import time
from datetime import timedelta

from typing_extensions import Optional
from enum import Enum
from ..episode_player_SDT import EpisodePlayer
import logging

logging.basicConfig(level=logging.INFO)
logdebug = logging.debug
loginfo = logging.info

from pycram.datastructures.enums import Arms, Grasp, TorsoState, TaskStatus
from pycram.datastructures.grasp import GraspDescription
from pycram.designator import ObjectDesignatorDescription

# from pycram.robot_plans import ActionDescription, ObjectDesignatorDescription
from pycram.robot_plans import (
    PickUpActionDescription,
    PlaceActionDescription,
    PickUpAction,
    PlaceAction,
    MoveTorsoActionDescription,
)
from pycram.language import SequentialPlan
from pycram.datastructures.pose import Pose

# from pycram.datastructures.world import World
from pycram.plan import Plan
from pycram.process_module import ProcessModule, simulated_robot

# from pycram.ros import logdebug
# from pycram.worlds.bullet_world import BulletWorld
# from pycrap.ontologies import Robot, Kitchen
# from pycram.world_concepts.world_object import Object

from semantic_digital_twin.world import World
from semantic_digital_twin.robots.abstract_robot import (
    ArmSelector,
    TorsoState,
    Grasp,
    GraspDescription,
    TaskStatus,
)


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
        object_description = ObjectDesignatorDescription(names=["milk"])
        description = PickUpActionDescription(
            object_description, [ArmSelector.LEFT], [GraspDescription(Grasp.FRONT)]
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
