from unittest import TestCase, skipIf
from episode_segmenter.file_episode_segmenter import FileEpisodeSegmenter, FileEpisodePlayer
from pycram.worlds.bullet_world import BulletWorld

Multiverse = None
try:
    from pycram.worlds.multiverse import Multiverse
except ImportError:
    pass


class TestFileEpisodeSegmenter(TestCase):
    file_player: FileEpisodePlayer

    @classmethod
    def setUpClass(cls):
        json_file = "../resources/fame_episodes/alessandro_with_ycp_objects_in_max_room/refined_poses.json"
        simulator = BulletWorld if Multiverse is None else Multiverse
        cls.file_player = FileEpisodePlayer(json_file, world=simulator)

    @classmethod
    def tearDownClass(cls):
        cls.file_player.world.exit()

    def test_replay_episode(self):
        self.file_player.run()
