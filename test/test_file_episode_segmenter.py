import datetime
from unittest import TestCase

import pycrap
from pycram.datastructures.world import World
from pycram.datastructures.enums import WorldMode
from episode_segmenter.episode_player import FileEpisodePlayer
from episode_segmenter.episode_segmenter import NoAgentEpisodeSegmenter
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld

Multiverse = None
try:
    from pycram.worlds.multiverse import Multiverse
except ImportError:
    pass


class TestFileEpisodeSegmenter(TestCase):
    world: World
    file_player: FileEpisodePlayer
    episode_segmenter: NoAgentEpisodeSegmenter
    viz_marker_publisher: VizMarkerPublisher

    @classmethod
    def setUpClass(cls):
        json_file = "../resources/fame_episodes/alessandro_with_ycp_objects_in_max_room_2/refined_poses.json"
        # simulator = BulletWorld if Multiverse is None else Multiverse
        simulator = BulletWorld
        annotate_events = True if simulator == BulletWorld else False
        cls.world = simulator()
        cls.viz_marker_publisher = VizMarkerPublisher()
        obj_id_to_name = {1: "chips", 3: "bowl", 4: "cup"}
        obj_id_to_type = {1: pycrap.Container, 3: pycrap.Bowl, 4: pycrap.Cup}
        cls.file_player = FileEpisodePlayer(json_file, world=cls.world,
                                            time_between_frames=datetime.timedelta(milliseconds=50),
                                            objects_to_ignore=[5],
                                            obj_id_to_name=obj_id_to_name,
                                            obj_id_to_type=obj_id_to_type)
        cls.episode_segmenter = NoAgentEpisodeSegmenter(cls.file_player, annotate_events=annotate_events)

    @classmethod
    def tearDownClass(cls):
        cls.viz_marker_publisher._stop_publishing()
        cls.world.exit()
        cls.episode_segmenter.join()

    def test_replay_episode(self):
        self.episode_segmenter.start()
