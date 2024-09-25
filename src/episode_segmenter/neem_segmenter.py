import datetime
from typing_extensions import Optional, List, Type

from neem_pycram_interface import PyCRAMNEEMInterface

from .event_detectors import EventDetector
from .events import Event
from .episode_segmenter import EpisodePlayer, AgentBasedEpisodeSegmenter


class NEEMSegmenter(AgentBasedEpisodeSegmenter):
    """
    The NEEMSegmenter class is used to segment the NEEMs motion replay data by using event detectors, such as contact,
    loss of contact, and pick up events.
    """

    def __init__(self, pycram_neem_interface: PyCRAMNEEMInterface,
                 detectors_to_start: List[Type[EventDetector]],
                 annotate_events: bool = False):
        """
        Initializes the NEEMSegmenter class.
        :param pycram_neem_interface: The neem pycram interface object used to query the NEEMs motion replay data.
        """
        self.neem_player_thread = NEEMPlayer(pycram_neem_interface)
        super().__init__(self.neem_player_thread, detectors_to_start=detectors_to_start,
                         annotate_events=annotate_events)

    def run_event_detectors_on_neem(self, sql_neem_ids: Optional[List[int]] = None) -> None:
        """
        Runs the event detectors on the NEEMs motion replay data.
        :param sql_neem_ids: An optional list of integer values that represent the SQL NEEM IDs.
        """
        if sql_neem_ids is None:
            sql_neem_ids = [17]

        self.neem_player_thread.query_neems_motion_replay_data(sql_neem_ids)

        self.start_motion_generator_thread_and_wait_till_ready()

        self.run_event_detectors()


class NEEMPlayer(EpisodePlayer):
    def __init__(self, pycram_neem_interface: PyCRAMNEEMInterface):
        super().__init__()
        self.pni = pycram_neem_interface

    def query_neems_motion_replay_data(self, sql_neem_ids: List[int]):
        self.pni.query_neems_motion_replay_data(sql_neem_ids=sql_neem_ids)

    @property
    def ready(self):
        return self.pni.replay_environment_initialized

    def run(self):
        self.pni.replay_motions_in_query(real_time=False,
                                         step_time=datetime.timedelta(milliseconds=1))
