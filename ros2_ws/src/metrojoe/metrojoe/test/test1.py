from typing import Literal
from FastDebugger import fd


direction_map = {
    ('left', 'forward'): 1,
    ('left', 'reverse'): 0,
    ('right', 'forward'): 0,
    ('right', 'reverse'): 1,
}

def get_direction_value(side, direction:Literal['forward', 'reverse']):
    return direction_map.get((side, direction), None) # type: ignore

fd(get_direction_value('left', 'forward'))