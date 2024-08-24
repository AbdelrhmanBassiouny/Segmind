from unittest import TestCase
from episode_segmenter.file_episode_segmenter import FAMEEpisodeSegmenter, FAMEEpisodePlayer


class TestFameEpisodeSegmenter(TestCase):
    fame_player: FAMEEpisodePlayer

    @classmethod
    def setUpClass(cls):
        json_file = "../resources/fame_episodes/alessandro_with_ycp_objects_in_max_room/refined_poses.json"
        cls.fame_player = FAMEEpisodePlayer(json_file)

    @classmethod
    def tearDownClass(cls):
        cls.fame_player.world.exit()

    def test_replay_episode(self):
        self.fame_player.replay_episode()
