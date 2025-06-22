from ripple_down_rules.datastructures.case import Case
from typing_extensions import Dict, Optional, Union
from segmind.detectors.atomic_event_detectors import RotationDetector
from types import NoneType


def conditions_105596190107074995602739244811495447398(case) -> bool:
    def conditions_for_motion_detector_is_moving(self_: RotationDetector, output_: bool) -> bool:
        """Get conditions on whether it's possible to conclude a value for MotionDetector_is_moving.output_  of type ."""
        return True
    return conditions_for_motion_detector_is_moving(**case)


def conclusion_105596190107074995602739244811495447398(case) -> bool:
    def motion_detector_is_moving(self_: RotationDetector, output_: bool) -> bool:
        """Get possible value(s) for MotionDetector_is_moving.output_  of type ."""
        return output_
    return motion_detector_is_moving(**case)


