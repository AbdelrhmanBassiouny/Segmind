from types import NoneType
from ripple_down_rules.datastructures.case import Case, create_case
from typing_extensions import Optional
from .motion_detector_is_moving_output__scrdr_defs import *


attribute_name = 'output_'
conclusion_type = (bool,)
mutually_exclusive = True


def classify(case: Dict, **kwargs) -> Optional[bool]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)

    if conditions_105596190107074995602739244811495447398(case):
        return conclusion_105596190107074995602739244811495447398(case)
    else:
        return None
