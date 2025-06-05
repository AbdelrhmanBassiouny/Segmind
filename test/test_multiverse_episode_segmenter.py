import datetime
from pathlib import Path
from unittest import TestCase
from os.path import dirname

from gitdb.util import dirname

import pycram.ros
from pycram.datastructures.world import World
from pycram.datastructures.enums import WorldMode
from segmind.players.csv_player import CSVpisodePlayer
from segmind.players.json_player import FileEpisodePlayer
from segmind.episode_segmenter import NoAgentEpisodeSegmenter
from segmind.players.json_player import FileEpisodePlayer
from segmind.detectors.coarse_event_detectors import GeneralPickUpDetector
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.world import World
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld
from pycrap.ontologies import Container, Bowl, Cup

Multiverse = None
try:
    from pycram.worlds.multiverse2 import Multiverse
except ImportError:
    pass


class TestFileEpisodeSegmenter(TestCase):
    world: World
    file_player: FileEpisodePlayer
    episode_segmenter: NoAgentEpisodeSegmenter
    viz_marker_publisher: VizMarkerPublisher

    @classmethod
    def setUpClass(cls):
        csv_file = f"{dirname(__file__)}/../resources/multiverse_episodes/icub_montessori_no_hands/data.csv"
        # json_file = "../resources/fame_episodes/alessandro_sliding_bueno/refined_poses.json"
        # simulator = BulletWorld if Multiverse is None else Multiverse
        simulator = Multiverse
        annotate_events = True if simulator == BulletWorld else False
        cls.world = simulator(WorldMode.DIRECT)
        pycram.ros.set_logger_level(pycram.datastructures.enums.LoggerLevel.DEBUG)
        cls.viz_marker_publisher = VizMarkerPublisher()
        cls.file_player = CSVpisodePlayer(csv_file, world=cls.world,
                                           time_between_frames=datetime.timedelta(milliseconds=10))
        cls.episode_segmenter = NoAgentEpisodeSegmenter(cls.file_player, annotate_events=annotate_events,
                                                        plot_timeline=True,
                                                        plot_save_path=f'test_results/{Path(dirname(csv_file)).stem}',
                                                        detectors_to_start=[GeneralPickUpDetector])

    @classmethod
    def tearDownClass(cls):
        cls.viz_marker_publisher._stop_publishing()
        cls.world.exit()
        cls.episode_segmenter.join()

    def test_replay_episode(self):
        self.episode_segmenter.start()
