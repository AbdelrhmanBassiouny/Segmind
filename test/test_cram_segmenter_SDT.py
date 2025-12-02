import sys
import time
import unittest
from os.path import dirname
from typing import Optional
import logging
logging.basicConfig(level=logging.DEBUG)
import os

from segmind.segmenters.cram_segmenter import CRAMSegmenter
from segmind.detectors.coarse_event_detectors_SDT import GeneralPickUpDetector


#from pycram.datastructures.grasp import GraspDescription
#from pycram.designators.action_designator import MoveTorsoActionDescription, PickUpActionDescription
from pycram.robot_plans import ActionDescription, ObjectDesignatorDescription
from pycram.robot_plans import PickUpActionDescription, PlaceActionDescription, PickUpAction, \
    PlaceAction, MoveTorsoActionDescription
from pycram.language import SequentialPlan
from pycram.plan import Plan
from pycram.designator import ObjectDesignatorDescription
from pycram.process_module import simulated_robot

#from pycram.datastructures.pose import PoseStamped, Vector3
from pycram.ros import set_logger_level
#from pycram.datastructures.enums import Arms, Grasp, TorsoState, WorldMode, LoggerLevel
from pycram.datastructures.pose import Pose
from pycram.datastructures.world import UseProspectionWorld
#from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld
from pycrap.ontologies import Robot, Milk, Kitchen

from pycram.robot_description import RobotDescriptionManager
try:
    from pyqt6.QtWidgets import QApplication
    from ripple_down_rules.user_interface.gui import RDRCaseViewer
except ImportError as e:
    QApplication = None
    RDRCaseViewer = None

from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.spatial_types import TransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.robots.abstract_robot import WorldMode, ArmSelector, TorsoState, Grasp, GraspDescription, TaskStatus
from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.semantic_annotations.semantic_annotations import Container
from semantic_digital_twin.utils import get_semantic_digital_twin_directory_root
apartment = os.path.join(get_semantic_digital_twin_directory_root(os.getcwd()), "resources", "urdf", "apartment.urdf")

class TestCRAMPlayer(unittest.TestCase):
    cram_segmenter: CRAMSegmenter
    PR2: Body
    Container: Body
    apartment: Body
    render_mode: WorldMode = WorldMode.DIRECT
    viz_marker_publisher: VizMarkerPublisher
    world: World
    app: Optional[QApplication] = None
    viewer: Optional[RDRCaseViewer] = None
    use_gui: bool = False

    @classmethod
    def setUpClass(cls):
        RobotDescriptionManager().load_description("pr2")
        cls.world = World(mode=cls.render_mode)
        logging.basicConfig(level=logging.DEBUG)
        cls.viz_marker_publisher = VizMarkerPublisher()
        cls.cram_segmenter = CRAMSegmenter(cls.world, [GeneralPickUpDetector], plot_timeline=True,
                                           plot_save_path=f"{dirname(__file__)}/test_results/cram_segmenter_test")
        cls.apartment = Body("apartment", apartment, "apartment.urdf")
        cls.PR2 = Body(
            "pr2",
            PR2,
            "pr2.urdf",
            pose=TransformationMatrix.from_translation_rotation(
                translation=Vector3(0.6, 0.4, 0.0),
                quat_wxyz=(1, 0, 0, 0),  # (w, x, y, z)
                child_frame="pr2"
            )
        )

        cls.Container = Body(
            "Container",
            Container,
            "Container.stl",
            pose=TransformationMatrix.from_translation_rotation(
                translation=Vector3(1.3, 1.0, 0.9),
                quat_wxyz=(1, 0, 0, 0),  # (w, x, y, z)
                child_frame="milk"
            )
        )

        if cls.use_gui and QApplication is not None:
            cls.app = QApplication(sys.argv)
            cls.viewer = RDRCaseViewer()


    def tearDown(self):
        if Plan.current_plan is not None:
            Plan.current_plan.clear()
        time.sleep(0.05)
        self.world.reset_world(remove_saved_states=True)
        with World():
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
        description = PickUpActionDescription(object_description, [ArmSelector.LEFT], [GraspDescription(Grasp.FRONT)])
        with simulated_robot:
            plan = SequentialPlan(MoveTorsoActionDescription(TorsoState.HIGH),
            description)
            plan.perform()
        # plan.plot()
