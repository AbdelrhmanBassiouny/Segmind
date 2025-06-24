from ripple_down_rules.datastructures.case import create_case
from types import NoneType
from typing_extensions import Optional
from .general_pick_up_detector_start_condition_checker_output__scrdr_defs import *


attribute_name = 'output_'
conclusion_type = (bool,)
mutually_exclusive = True


def classify(case: Dict, **kwargs) -> Optional[bool]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)

    if conditions_204802191293408881662554642895906550581(case):
        return conclusion_204802191293408881662554642895906550581(case)

    elif conditions_161580757619499966579994522098509194470(case):
        return conclusion_161580757619499966579994522098509194470(case)

    elif conditions_38026252761470320656686246787772237017(case):
        return conclusion_38026252761470320656686246787772237017(case)
    else:
        return None
