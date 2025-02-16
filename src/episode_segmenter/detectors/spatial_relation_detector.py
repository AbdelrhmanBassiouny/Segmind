from typing_extensions import Optional, List

from coarse_event_detectors import DetectorWithStarterEvent
from episode_segmenter.event_logger import EventLogger
from pycram import World
from pycram.world_concepts.world_object import Object
from ..datastructures.events import Event, ContactEvent, StopMotionEvent, MotionEvent, NewObjectEvent
from ..utils import calculate_translation_difference_and_check


class SpatialRelationDetector(DetectorWithStarterEvent):
    """
    A class that detects spatial relations between objects.
    """
    near_distance: float = 1.0
    """
    The distance threshold for nearness in meters.
    """

    @classmethod
    def start_condition_checker(cls, event: Event) -> bool:
        if isinstance(event, (NewObjectEvent, ContactEvent, MotionEvent)):
            return True

    def detect_events(self) -> List[Event]:
        involved_objects = self.starter_event.involved_objects

    def get_near_objects(self, obj: Object) -> List[Object]:
        """
        Get the objects that are near the given object.

        :param obj: The object.
        :return: The objects that are near the given object.
        """
        near_objects = []
        for other_obj in World.current_world.objects:
            if obj is not other_obj:
                near = calculate_translation_difference_and_check(obj.pose_as_list, other_obj.pose_as_list,
                                                                  self.near_distance)
                if near:
                    near_objects.append(other_obj)
        return near_objects

    def get_container_objects(self, obj: Object) -> List[Object]:
        """
        Get the objects that are containers of the given object.

        :param obj: The object.
        :return: The objects that are containers of the given object.
        """
        container_objects = []
        obj_bbox = obj.get_axis_aligned_bounding_box()
        for possible_container in World.current_world.objects:
            if obj is not possible_container:
                pc_bbox = possible_container.get_axis_aligned_bounding_box()
                if obj_bbox in pc_bbox:
                    container_objects.append(possible_container)
        return container_objects



    def __str__(self):
        pass

    def detect(self) -> Optional[Event]:
        pass
