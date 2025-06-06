from ripple_down_rules.datastructures.case import Case, create_case
from types import NoneType
from typing_extensions import Optional
from .general_pick_up_detector_get_interaction_event_output__scrdr_defs import *


attribute_name = 'output_'
conclusion_type = (NoneType, PickUpEvent,)
mutually_exclusive = True


def classify(case: Dict, **kwargs) -> Optional[PickUpEvent]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)

    if conditions_87074858769394720739688305292375760638(case):
        return conclusion_87074858769394720739688305292375760638(case)

    elif conditions_121320552838186729138877771657303489240(case):

        if conditions_320416996501934194262144719758568379805(case):
            return conclusion_320416996501934194262144719758568379805(case)
        return conclusion_121320552838186729138877771657303489240(case)
    else:
        return None
