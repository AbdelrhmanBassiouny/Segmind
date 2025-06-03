from types import NoneType
from typing_extensions import Optional
from ripple_down_rules.datastructures.case import create_case
from .general_pick_up_detector_start_condition_checker_output__scrdr_defs import *


attribute_name = 'output_'
conclusion_type = (bool,)
mutually_exclusive = True


def classify(case: Dict, **kwargs) -> Optional[bool]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)

    if conditions_79409294830217498801042528243955850723(case):
        return conclusion_79409294830217498801042528243955850723(case)
    else:
        return None
