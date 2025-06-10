from types import NoneType
from ripple_down_rules.datastructures.case import Case, create_case
from typing_extensions import Optional
from .episode_segmenter_is_detector_redundant_output__scrdr_defs import *


attribute_name = 'output_'
conclusion_type = (bool,)
mutually_exclusive = True


def classify(case: Dict, **kwargs) -> Optional[bool]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)

    if conditions_318535409151373315477142163500790537263(case):
        return conclusion_318535409151373315477142163500790537263(case)
    else:
        return None
