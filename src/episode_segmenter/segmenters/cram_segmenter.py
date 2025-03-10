from typing_extensions import Type, List
from queue import Queue

from pycram.orm.action_designator import Action
from pycram.tasktree import TaskTreeNode, task_tree
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
        self.action_types: List[Type[Action]] = [detector.action_type for detector in self.detectors_to_start]
        self.start_action_queue: Queue = Queue()
        self.end_action_queue: Queue = Queue()
        if self.action_types:
            for action_type in self.action_types:
                self.add_callback(action_type)
        else:
            self.add_callback(Action)

    def add_callback(self, action_type: Type[Action]):
        """
        Add a callback for the given action type.

        :param action_type: The action type to add the callback for.
        """
        task_tree.add_callback(action_type, self.start_action_callback)
        task_tree.add_callback(action_type, self.end_action_callback, on_start=False)

    def start_action_callback(self, action_node: TaskTreeNode):
        """
        The action callback method that is called when an action is performed.

        :param action_node: The node in the task tree representing the action that was performed.
        """
        # Maybe create an Event for the given action.
        # One could use that for supervising the RDRs of action detection.
        # Maybe fit_rdr_case here :D.
        print(f"Action Started: {action_node}")
        self.start_action_queue.put(action_node)

    def end_action_callback(self, action_node: TaskTreeNode):
        """
        The action callback method that is called when an action is performed.

        :param action_node: The node in the task tree representing the action that was performed.
        """
        print(f"Action Ended: {action_node}")
        self.end_action_queue.put(action_node)

    def start(self) -> None:
        """
        Query CRAMs for motion replay data and run the event detectors on the CRAMs motion replay data.
        """
        super().start()
