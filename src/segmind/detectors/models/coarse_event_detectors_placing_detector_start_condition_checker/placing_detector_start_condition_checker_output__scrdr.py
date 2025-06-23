from types import NoneType
from ripple_down_rules.datastructures.case import create_case
from typing_extensions import Optional
from .placing_detector_start_condition_checker_output__scrdr_defs import *


attribute_name = 'output_'
conclusion_type = (bool,)
mutually_exclusive = True


def classify(case: Dict, **kwargs) -> Optional[bool]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)

    if conditions_213633301887999429662908232055793196406(case):
        return conclusion_213633301887999429662908232055793196406(case)

    elif conditions_324552974989252417774318641412101963311(case):

        if conditions_200496988155257780428139539183300226414(case):
            return conclusion_200496988155257780428139539183300226414(case)

        elif conditions_308324129151228864976597656067893575149(case):
            return conclusion_308324129151228864976597656067893575149(case)
        return conclusion_324552974989252417774318641412101963311(case)
    else:
        return None
