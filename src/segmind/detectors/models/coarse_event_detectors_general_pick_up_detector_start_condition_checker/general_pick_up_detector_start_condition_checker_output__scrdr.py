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

        if conditions_8274239634455277150877461420723135533(case):

            if conditions_175987223108804549769623056194939396888(case):
                return conclusion_175987223108804549769623056194939396888(case)

            elif conditions_137657377818990651652995783903795984105(case):
                return conclusion_137657377818990651652995783903795984105(case)

            elif conditions_127762420515884983148248561408247495267(case):
                return conclusion_127762420515884983148248561408247495267(case)

            elif conditions_199058987582084612752317307773823561830(case):
                return conclusion_199058987582084612752317307773823561830(case)

            elif conditions_85208169816383195582254003928161949871(case):
                return conclusion_85208169816383195582254003928161949871(case)

            elif conditions_181643586107175766966334460719476171055(case):
                return conclusion_181643586107175766966334460719476171055(case)
            return conclusion_8274239634455277150877461420723135533(case)
        return conclusion_79409294830217498801042528243955850723(case)
    else:
        return None
