import datetime
import time
import unittest

from anytree.exporter import DotExporter

from episode_segmenter.players.cram_player import CRAMPlayer
from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import Arms, Grasp, TorsoState, WorldMode
from pycram.datastructures.pose import Pose
from pycram.datastructures.world import UseProspectionWorld
from pycram.designator import ObjectDesignatorDescription
from pycram.designators.action_designator import PickUpActionPerformable, PickUpAction, MoveTorsoAction
from pycram.process_module import simulated_robot, ProcessModule
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.tasktree import task_tree
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld
from pycrap.ontologies import Robot, Milk, Kitchen
from ripple_down_rules.utils import render_tree


class TestCRAMPlayer(unittest.TestCase):
    cram_player: CRAMPlayer
    robot: Object
    milk: Object
    kitchen: Object
    render_mode: WorldMode = WorldMode.DIRECT
    viz_marker_publisher: VizMarkerPublisher
    world: BulletWorld

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld(mode=cls.render_mode)
        ProcessModule.execution_delay = False
        cls.viz_marker_publisher = VizMarkerPublisher()
        cls.cram_player = CRAMPlayer()
        cls.kitchen = Object("kitchen", Kitchen, "kitchen.urdf")
        cls.robot = Object("pr2", Robot, "pr2.urdf", pose=Pose([0.6, 0.4, 0]))
        cls.milk = Object("milk", Milk, "milk.stl", pose=Pose([1.3, 1, 0.9]))

    def tearDown(self):
        task_tree.reset_tree()
        time.sleep(0.05)
        self.world.reset_world(remove_saved_states=True)
        with UseProspectionWorld():
            pass

    @classmethod
    def tearDownClass(cls):
        cls.viz_marker_publisher._stop_publishing()
        cls.world.exit()

    def test_pick_up(self):
        object_description = ObjectDesignatorDescription(names=["milk"])
        description = PickUpAction(object_description, [Arms.LEFT], [Grasp.FRONT])
        with simulated_robot:
            MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
            description.resolve().perform()
        task_tree.render("results/pick_up_tree")
