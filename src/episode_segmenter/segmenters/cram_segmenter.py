from ..episode_segmenter import AgentBasedEpisodeSegmenter
from ..players.cram_player import CRAMPlayer


class CRAMSegmenter(AgentBasedEpisodeSegmenter):
    """
    The CRAMSegmenter class is used to segment the CRAMs motion replay data by using event detectors, such as contact,
    loss of contact, and pick up events.
    """

    def __init__(self, world, detectors_to_start=None, annotate_events=False):
        """
        Initializes the CRAMSegmenter class.

        :param world: The world object used to query the CRAMs motion replay data.
        :param detectors_to_start: An optional list of event detectors to start.
        :param annotate_events: A boolean value that indicates whether the events should be annotated.
        """
        self.cram_player_thread = CRAMPlayer(world)
        super().__init__(self.cram_player_thread,
                         detectors_to_start=detectors_to_start,
                         annotate_events=annotate_events)

    def start(self) -> None:
        """
        Query CRAMs for motion replay data and run the event detectors on the CRAMs motion replay data.
        """
        super().start()
