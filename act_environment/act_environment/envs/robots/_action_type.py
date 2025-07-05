from enum import Enum, auto

class ActionType(Enum):
    POSITION = auto()
    VELOCITY = auto()
    TORQUE = auto()