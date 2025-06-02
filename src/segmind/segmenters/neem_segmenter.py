from typing_extensions import Optional, List, Type

from neem_pycram_interface import PyCRAMNEEMInterface

from ..detectors.coarse_event_detectors import DetectorWithStarterEvent
from ..episode_segmenter import AgentEpisodeSegmenter
from ..players.neem_player import NEEMPlayer


class NEEMSegmenter(AgentEpisodeSegmenter):
    """
    The NEEMSegmenter class is used to segment the NEEMs motion replay data by using event detectors, such as contact,
    loss of contact, and pick up events.
    """

    def __init__(self, pycram_neem_interface: PyCRAMNEEMInterface,
                 detectors_to_start: Optional[List[Type[DetectorWithStarterEvent]]] = None,
                 annotate_events: bool = False, **kwargs):
        """
        Initializes the NEEMSegmenter class.

        :param pycram_neem_interface: The neem pycram interface object used to query the NEEMs motion replay data.
        :param detectors_to_start: An optional list of event detectors to start.
        :param annotate_events: A boolean value that indicates whether the events should be annotated.
        """
        self.neem_player_thread = NEEMPlayer(pycram_neem_interface)
        super().__init__(self.neem_player_thread, detectors_to_start=detectors_to_start,
                         annotate_events=annotate_events, **kwargs)

    def start(self, sql_neem_ids: Optional[List[int]] = None) -> None:
        """
        Query NEEMs for motion replay data and run the event detectors on the NEEMs motion replay data.

        :param sql_neem_ids: An optional list of integer values that represent the SQL NEEM IDs.
        """
        if sql_neem_ids is None:
            sql_neem_ids = [17]

        self.neem_player_thread.query_neems_motion_replay_data(sql_neem_ids)

        super().start()
