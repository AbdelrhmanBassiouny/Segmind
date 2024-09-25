from unittest import TestCase
from episode_segmenter.file_episode_segmenter import FileEpisodeSegmenter, FileEpisodePlayer
from pycram.worlds.multiverse import Multiverse


class TestFileEpisodeSegmenter(TestCase):
    file_player: FileEpisodePlayer

    @classmethod
    def setUpClass(cls):
        json_file = "../resources/fame_episodes/alessandro_with_ycp_objects_in_max_room/refined_poses.json"
        cls.file_player = FileEpisodePlayer(json_file, world=Multiverse)

    @classmethod
    def tearDownClass(cls):
        cls.file_player.world.exit()

    def test_replay_episode(self):
        self.file_player.run()
