import os

from neem_pycram_interface import PyCRAMNEEMInterface

from episode_segmenter.detectors.coarse_event_detectors import AgentPickUpDetector, PlacingDetector, \
    GeneralPickUpDetector
from episode_segmenter.segmenters.neem_segmenter import NEEMSegmenter
from unittest import TestCase

from pycram.datastructures.enums import WorldMode, LoggerLevel
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.datastructures.world import World
from pycram.worlds.bullet_world import BulletWorld
from pycram.ros import set_logger_level


class TestNEEMSegmentor(TestCase):
    pni: PyCRAMNEEMInterface
    viz_mark_publisher: VizMarkerPublisher

    @classmethod
    def setUpClass(cls):
        BulletWorld(WorldMode.GUI)
        set_logger_level(LoggerLevel.DEBUG)
        cls.pni = PyCRAMNEEMInterface(f'mysql+pymysql://{os.environ["my_maria_uri"]}')
        cls.viz_mark_publisher = VizMarkerPublisher()

    @classmethod
    def tearDownClass(cls):
        cls.viz_mark_publisher._stop_publishing()
        if World.current_world is not None:
            World.current_world.exit()

    def test_event_detector(self):
        ns = NEEMSegmenter(self.pni, detectors_to_start=[AgentPickUpDetector, PlacingDetector], annotate_events=True)
        ns.start([17])

    def test_general_pick_up_detector(self):
        ns = NEEMSegmenter(self.pni, detectors_to_start=[GeneralPickUpDetector], annotate_events=True)
        ns.start([17])
