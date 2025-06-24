from ripple_down_rules.datastructures.case import Case, create_case
from typing_extensions import Optional
from types import NoneType
from .general_pick_up_detector_start_condition_checker_output__scrdr_defs import *


attribute_name = 'output_'
conclusion_type = (bool,)
mutually_exclusive = True


def classify(case: Dict, **kwargs) -> Optional[bool]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)

    if conditions_313968436519149281932112885344716145224(case):
        return conclusion_313968436519149281932112885344716145224(case)
    else:
        return None
