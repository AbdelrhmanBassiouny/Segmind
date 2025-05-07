import time
import unittest

from pycram.datastructures.grasp import GraspDescription
from pycram.designators.action_designator import MoveTorsoActionDescription, PickUpActionDescription
from pycram.language import SequentialPlan
from pycram.datastructures.pose import PoseStamped, Vector3
from episode_segmenter.segmenters.cram_segmenter import CRAMSegmenter
from episode_segmenter.detectors.coarse_event_detectors import MotionPickUpDetector, GeneralPickUpDetector
from pycram.datastructures.enums import Arms, Grasp, TorsoState, WorldMode
from pycram.datastructures.pose import Pose
from pycram.datastructures.world import UseProspectionWorld
from pycram.designator import ObjectDesignatorDescription
from pycram.designators.action_designator import PickUpAction, MoveTorsoAction
from pycram.process_module import simulated_robot, ProcessModule
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.plan import Plan
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld
from pycrap.ontologies import Robot, Milk, Kitchen
from pycram.robot_description import RobotDescriptionManager


class TestCRAMPlayer(unittest.TestCase):
    cram_segmenter: CRAMSegmenter
    robot: Object
    milk: Object
    kitchen: Object
    render_mode: WorldMode = WorldMode.DIRECT
    viz_marker_publisher: VizMarkerPublisher
    world: BulletWorld

    @classmethod
    def setUpClass(cls):
        RobotDescriptionManager().load_description("pr2")
        cls.world = BulletWorld(mode=cls.render_mode)
        cls.viz_marker_publisher = VizMarkerPublisher()
        cls.cram_segmenter = CRAMSegmenter(cls.world, [GeneralPickUpDetector])
        cls.kitchen = Object("kitchen", Kitchen, "kitchen.urdf")
        cls.robot = Object("pr2", Robot, "pr2.urdf", pose=PoseStamped(Pose(Vector3(0.6, 0.4, 0))))
        cls.milk = Object("milk", Milk, "milk.stl", pose=PoseStamped(Pose(Vector3(1.3, 1, 0.9))))

    def tearDown(self):
        if Plan.current_plan is not None:
            Plan.current_plan.clear()
        time.sleep(0.05)
        self.world.reset_world(remove_saved_states=True)
        with UseProspectionWorld():
            pass

    @classmethod
    def tearDownClass(cls):
        GeneralPickUpDetector.start_condition_rdr.save()
        cls.viz_marker_publisher._stop_publishing()
        cls.world.exit()

    def test_pick_up(self):
        # self.execute_pick_up_plan()
        self.cram_segmenter.start()

    @staticmethod
    def execute_pick_up_plan():
        object_description = ObjectDesignatorDescription(names=["milk"])
        description = PickUpActionDescription(object_description, [Arms.LEFT], [GraspDescription(Grasp.FRONT)])
        with simulated_robot:
            plan = SequentialPlan(MoveTorsoActionDescription(TorsoState.HIGH),
            description)
            plan.perform()
        plan.plot()
