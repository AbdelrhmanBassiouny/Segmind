from types import NoneType
from ripple_down_rules.datastructures.case import Case, create_case
from typing_extensions import Optional
from .general_pick_up_detector_get_object_to_track_from_starter_event_output__scrdr_defs import *


attribute_name = 'output_'
conclusion_type = (NoneType, Object,)
mutually_exclusive = True


def classify(case: Dict, **kwargs) -> Optional[Object]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)

    if conditions_257779009639527613634091190332753064691(case):
        return conclusion_257779009639527613634091190332753064691(case)

    elif conditions_306600839998818187425923425930024243551(case):
        return conclusion_306600839998818187425923425930024243551(case)
    else:
        return None
