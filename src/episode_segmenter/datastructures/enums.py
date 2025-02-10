from enum import Enum, auto


class DistanceFilter(Enum):
    MOVING_AVERAGE = auto()
    LOW_PASS = auto()


class MotionDetectionMethod(Enum):
    CONSISTENT_GRADIENT = auto()
    DISTANCE = auto()
