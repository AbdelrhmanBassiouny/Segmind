from ripple_down_rules.datastructures.case import Case, create_case
from .general_pick_up_detector_get_object_to_track_from_starter_event_output__scrdr_defs import *


attribute_name = 'output_'
conclusion_type = (Object,)
mutually_exclusive = True


def classify(case: Dict, **kwargs) -> Object:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)

    if conditions_257779009639527613634091190332753064691(case):
        return conclusion_257779009639527613634091190332753064691(case)
