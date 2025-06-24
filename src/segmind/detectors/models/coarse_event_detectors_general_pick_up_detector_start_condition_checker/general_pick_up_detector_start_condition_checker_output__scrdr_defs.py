from segmind.datastructures.events import Event, LossOfSupportEvent, StopTranslationEvent, TranslationEvent
from segmind.detectors.coarse_event_detectors import GeneralPickUpDetector
from segmind.episode_player import EpisodePlayer
from segmind.utils import get_support
from typing_extensions import Type

from pycram.ros import logerr


def conditions_313968436519149281932112885344716145224(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event,
                                                                        output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return isinstance(event, LossOfSupportEvent)

    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conditions_185374571278911955422500089733244334209(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event,
                                                                        output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        support = event.with_object
        return support.contains_body(event.tracked_object, update=True, intersection_ratio=0.9)

    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conclusion_185374571278911955422500089733244334209(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event,
                                                         output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return False

    return general_pick_up_detector_start_condition_checker(**case)


def conditions_259334668447470890319338447268411809573(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event,
                                                                        output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return "hole" in event.with_object.name

    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conclusion_259334668447470890319338447268411809573(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event,
                                                         output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return False

    return general_pick_up_detector_start_condition_checker(**case)


def conditions_1084517150588802996811823834978763787(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event,
                                                                        output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return any(["hole" in b.name for b in event.tracked_object.contact_points.get_all_bodies()])

    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conclusion_1084517150588802996811823834978763787(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event,
                                                         output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return False

    return general_pick_up_detector_start_condition_checker(**case)


def conditions_257426441376249650265480473188288026956(case) -> bool:
    def conditions_for_general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event,
                                                                        output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        translation_event = event.object_tracker.get_first_event_of_type_before_event(TranslationEvent, event)
        if translation_event is None:
            # logerr("No translation event found for GeneralPickUpDetector_start_condition_checker")
            return True
        if (translation_event.current_pose.position.z - translation_event.start_pose.position.z) >= (3 * 1e-3):
            return False
        else:
            return True
    return conditions_for_general_pick_up_detector_start_condition_checker(**case)


def conclusion_257426441376249650265480473188288026956(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event,
                                                         output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return False

    return general_pick_up_detector_start_condition_checker(**case)


def conclusion_313968436519149281932112885344716145224(case) -> bool:
    def general_pick_up_detector_start_condition_checker(cls_: Type[GeneralPickUpDetector], event: Event,
                                                         output_: bool) -> bool:
        """Get possible value(s) for GeneralPickUpDetector_start_condition_checker.output_  of type ."""
        return True

    return general_pick_up_detector_start_condition_checker(**case)
