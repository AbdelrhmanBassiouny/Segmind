from ripple_down_rules.datastructures.case import Case
from segmind.datastructures.events import InsertionEvent, InterferenceEvent, PickUpEvent
from typing_extensions import Dict, Optional, Union
from types import NoneType
from segmind.detectors.spatial_relation_detector import InsertionDetector
from pycram.datastructures.world_entity import PhysicalBody
from semantic_world.views.views import Container


def conditions_38355037295796650033371896063976531277(case) -> bool:
    def conditions_for_insertion_detector_hole_insertion_verifier(self_: InsertionDetector, hole: PhysicalBody, event: InterferenceEvent, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for InsertionDetector_hole_insertion_verifier.output_  of type ."""
        return hole not in event.tracked_object.contained_in_bodies
    return conditions_for_insertion_detector_hole_insertion_verifier(**case)


def conditions_21738774625860220488991060484462427733(case) -> bool:
    def conditions_for_insertion_detector_hole_insertion_verifier(self_: InsertionDetector, hole: PhysicalBody, event: InterferenceEvent, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for InsertionDetector_hole_insertion_verifier.output_  of type ."""
        rays_results = event.tracked_object.cast_rays_in_all_directions(0.07)
        directions_results = [rr.intersected for rr in rays_results]
        directions_results_bottom = directions_results[4]
        if not directions_results_bottom:
            return False
        link_id = rays_results[4].link_id
        obj_id = rays_results[4].obj_id
        link = event.tracked_object.world.get_object_by_id(obj_id).get_link_by_id(link_id)
        containers = [v for v in event.tracked_object.world.views['views'] if isinstance(v, Container)]
        container_bodies = [c.body for c in containers]
        container_body_names = [c.name.name for c in container_bodies]
        return directions_results_bottom and link.name in container_body_names
    return conditions_for_insertion_detector_hole_insertion_verifier(**case)


def conditions_313966059252436144481394373657043070884(case) -> bool:
    def conditions_for_insertion_detector_hole_insertion_verifier(self_: InsertionDetector, hole: PhysicalBody, event: InterferenceEvent, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for InsertionDetector_hole_insertion_verifier.output_  of type ."""
        latest_insertion = event.object_tracker.get_latest_event_of_type(InsertionEvent)
        if latest_insertion is None:
            return False
        latest_pickup = event.object_tracker.get_first_event_of_type_after_event(PickUpEvent, latest_insertion)
        return latest_pickup is None
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
        return False
    return insertion_detector_hole_insertion_verifier(**case)


