import itertools
import time

from random_events.product_algebra import SimpleEvent
from ripple_down_rules.datastructures.case import Case

from pycram.datastructures.dataclasses import AxisAlignedBoundingBox
from ....datastructures.events import InsertionEvent, InterferenceEvent, PickUpEvent
from typing_extensions import Dict, Optional, Union
from types import NoneType
from ....detectors.spatial_relation_detector import InsertionDetector
from pycram.datastructures.world_entity import PhysicalBody
from semantic_world.views.views import Container

from ....utils import is_object_supported_by_container_body


def conditions_38355037295796650033371896063976531277(case) -> bool:
    def conditions_for_insertion_detector_hole_insertion_verifier(self_: InsertionDetector, hole: PhysicalBody, event: InterferenceEvent, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for InsertionDetector_hole_insertion_verifier.output_  of type ."""
        hole_bbox = hole.get_axis_aligned_bounding_box()
        hole_depth, hole_width, hole_height = hole_bbox.dimensions
        hole_max_z = hole_bbox.max_z
        # time.sleep(0.1)
        obj_bbox = event.tracked_object.get_axis_aligned_bounding_box()
        obj_max = obj_bbox.max_z
        # obj_event = SimpleEvent({obj_bbox.x_variable: obj_bbox.x_interval,
        #                     obj_bbox.y_variable: obj_bbox.y_interval})
        # hole_event = SimpleEvent({hole_bbox.x_variable: hole_bbox.x_interval,
        #                     hole_bbox.y_variable: hole_bbox.y_interval})
        # intersection = hole_event.intersection_with(obj_event)
        # result = []
        # for x, y in itertools.product(intersection[AxisAlignedBoundingBox.x_variable].simple_sets,
        #                                  intersection[AxisAlignedBoundingBox.y_variable].simple_sets):
        #     result.append(AxisAlignedBoundingBox(x.lower, y.lower, y.lower, x.upper, y.upper, y.upper))
        # if len(result) == 0:
        #     return False
        # i_width, i_depth, i_height = result[0].width, result[0].depth, result[0].height
        if obj_max <= hole_max_z + 1e-3: # and i_width >= hole_width*0.3 and i_depth >= hole_depth*0.3:
            return True
        else:
            return False
        # intersection = hole_bbox.intersection_with(event.bounding_box)
        # i_width, i_depth, i_height = intersection.width, intersection.depth, intersection.height
        # if (hole_bbox.width >= (i_width - 1e-3)
        #     and hole_bbox.depth >= (i_depth - 1e-3)
        #     and hole_bbox.height >= (i_height - 1e-3)):
        #     return True
        # return False
        # return True
    return conditions_for_insertion_detector_hole_insertion_verifier(**case)


def conditions_21738774625860220488991060484462427733(case) -> bool:
    def conditions_for_insertion_detector_hole_insertion_verifier(self_: InsertionDetector, hole: PhysicalBody, event: InterferenceEvent, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for InsertionDetector_hole_insertion_verifier.output_  of type ."""
        if event.tracked_object in self_.bodies_states:
            prev_containers = self_.bodies_states[event.tracked_object]
            event.tracked_object.update_containment(intersection_ratio=0.7)
            current_containers = event.tracked_object.contained_in_bodies
            new_containers = [body for body in current_containers if body not in prev_containers]
            return len(new_containers) > 0
        else:
            self_.update_body_state(event.tracked_object)
            contained_in = self_.bodies_states[event.tracked_object]
            if len(contained_in) > 0:
                if any("drawer" in b.name and "handle" not in b.name for b in contained_in):
                    return True
                elif any("box" in b.name for b in contained_in):
                    return True
                else:
                    return False
            else:
                return False
    return conditions_for_insertion_detector_hole_insertion_verifier(**case)


def conditions_313966059252436144481394373657043070884(case) -> bool:
    def conditions_for_insertion_detector_hole_insertion_verifier(self_: InsertionDetector, hole: PhysicalBody, event: InterferenceEvent, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for InsertionDetector_hole_insertion_verifier.output_  of type ."""
        latest_insertion = event.object_tracker.get_latest_event_of_type(InsertionEvent)
        if latest_insertion is None:
            return False
        latest_pickup = event.object_tracker.get_first_event_of_type_after_event(PickUpEvent, latest_insertion)
        no_pick_up = latest_pickup is None
        latest_insertion_before_this_insertion = event.object_tracker.get_first_event_of_type_before_event(InsertionEvent, latest_insertion)
        if latest_insertion_before_this_insertion is None:
            return no_pick_up
        elif abs(latest_insertion.timestamp - latest_insertion_before_this_insertion.timestamp) < 0.5:
            return True
        else:
            return no_pick_up
    return conditions_for_insertion_detector_hole_insertion_verifier(**case)


def conclusion_313966059252436144481394373657043070884(case) -> bool:
    def insertion_detector_hole_insertion_verifier(self_: InsertionDetector, hole: PhysicalBody, event: InterferenceEvent, output_: bool) -> bool:
        """Get possible value(s) for InsertionDetector_hole_insertion_verifier.output_  of type ."""
        return False
    return insertion_detector_hole_insertion_verifier(**case)


def conclusion_21738774625860220488991060484462427733(case) -> bool:
    def insertion_detector_hole_insertion_verifier(self_: InsertionDetector, hole: PhysicalBody, event: InterferenceEvent, output_: bool) -> bool:
        """Get possible value(s) for InsertionDetector_hole_insertion_verifier.output_  of type ."""
        return True
    return insertion_detector_hole_insertion_verifier(**case)


def conclusion_38355037295796650033371896063976531277(case) -> bool:
    def insertion_detector_hole_insertion_verifier(self_: InsertionDetector, hole: PhysicalBody, event: InterferenceEvent, output_: bool) -> bool:
        """Get possible value(s) for InsertionDetector_hole_insertion_verifier.output_  of type ."""
        return True
    return insertion_detector_hole_insertion_verifier(**case)


