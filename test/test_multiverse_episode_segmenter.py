import datetime
import os
from datetime import timedelta
from pathlib import Path
from unittest import TestCase
from os.path import dirname

from gitdb.util import dirname

import pycram.ros
from pycram.config.multiverse_conf import SimulatorConfig, MultiverseConfig
from pycram.datastructures.world import World
from pycram.datastructures.enums import WorldMode
from pycram.robot_description import RobotDescriptionManager
from pycram.world_concepts.world_object import Object
from segmind.players.csv_player import CSVEpisodePlayer
from segmind.episode_segmenter import NoAgentEpisodeSegmenter
from segmind.detectors.coarse_event_detectors import GeneralPickUpDetector
from segmind.detectors.spatial_relation_detector import InsertionDetector
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.world import World
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld
from pycrap.ontologies import Container, Bowl, Cup, Location

try:
    from pycram.worlds.multiverse2 import Multiverse
except ImportError:
    Multiverse = None


class TestMultiverseEpisodeSegmenter(TestCase):
    world: World
    file_player: CSVEpisodePlayer
    episode_segmenter: NoAgentEpisodeSegmenter
    viz_marker_publisher: VizMarkerPublisher

    @classmethod
    def setUpClass(cls):
        multiverse_episodes_dir = f"{dirname(__file__)}/../resources/multiverse_episodes"
        selected_episode = "icub_montessori_no_hands"
        episode_dir = os.path.join(multiverse_episodes_dir, selected_episode)
        csv_file = os.path.join(episode_dir, f"data.csv")
        models_dir = os.path.join(episode_dir, "models")
        scene_file_path = os.path.join(models_dir, f"scene.xml")
        simulator_conf = MultiverseConfig.simulator_config
        simulator_conf.step_size = timedelta(milliseconds=4)
        simulator_conf.integrator = "IMPLICITFAST"
        simulator_conf.cone = "ELLIPTIC"
        rdm = RobotDescriptionManager()
        rdm.load_description("iCub3")
        simulator = BulletWorld
        if simulator is Multiverse:
            cls.world: Multiverse = Multiverse(WorldMode.GUI, scene_file_path=scene_file_path,
                                               simulator_config=simulator_conf)
        else:
            cls.world: BulletWorld = BulletWorld(WorldMode.DIRECT)
        pycram.ros.set_logger_level(pycram.datastructures.enums.LoggerLevel.INFO)
        cls.viz_marker_publisher = VizMarkerPublisher()
        cls.file_player = CSVEpisodePlayer(csv_file, world=cls.world, time_between_frames=datetime.timedelta(milliseconds=4))
        # scene = Object("scene", Location, f"{Path(scene_file_path).stem}.urdf")
        cls.episode_segmenter = NoAgentEpisodeSegmenter(cls.file_player, annotate_events=True,
                                                        plot_timeline=True,
                                                        plot_save_path=f'test_results/{Path(dirname(csv_file)).stem}',
                                                        detectors_to_start=[GeneralPickUpDetector],
                                                        initial_detectors=[InsertionDetector])

    @classmethod
    def tearDownClass(cls):
        cls.viz_marker_publisher._stop_publishing()
        cls.world.exit()
        cls.episode_segmenter.join()

    def test_multiverse_replay(self):
        self.episode_segmenter.start()
