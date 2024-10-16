import datetime
from unittest import TestCase

from pycram.datastructures.world import World
from pycram.datastructures.enums import WorldMode
from episode_segmenter.episode_player import FileEpisodePlayer
from episode_segmenter.episode_segmenter import NoAgentEpisodeSegmenter
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

    @classmethod
    def setUpClass(cls):
        json_file = "../resources/fame_episodes/alessandro_with_ycp_objects_in_max_room/refined_poses.json"
        simulator = BulletWorld if Multiverse is None else Multiverse
        # simulator = BulletWorld
        cls.world = simulator(WorldMode.GUI)
        cls.file_player = FileEpisodePlayer(json_file, world=cls.world,
                                            time_between_frames=datetime.timedelta(milliseconds=100),
                                            objects_to_ignore=[5])
        cls.episode_segmenter = NoAgentEpisodeSegmenter(cls.file_player, detectors_to_start=[], annotate_events=True)

    @classmethod
    def tearDownClass(cls):
        cls.world.exit()
        cls.episode_segmenter.join()

    def test_replay_episode(self):
        self.episode_segmenter.start()
