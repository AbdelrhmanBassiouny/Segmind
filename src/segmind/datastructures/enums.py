from enum import Enum, auto


class TaskType(str, Enum):
    """
    A class that represents the type of task being performed.
    """
    TRANSPORTATION = "The task involves transporting objects, by picking them up and placing them somewhere else."
    ASSEMBLY = "The task involves assembling objects together, such as building a structure or completing a puzzle."
    TABLE_SETTING = "Setting a table with objects, such as plates, cutlery, and glasses."
    CLEANING = "The task involves cleaning objects or surfaces, such as wiping a table or washing dishes."
    MONTESSORI_BOX = "The task involves interacting with a Montessori box, which is a type of educational toy that encourages exploration and learning."


class PlayerStatus(Enum):
    """
    A class that represents the state of the episode player.
    """
    CREATED = auto()
    """
    The episode player is created.
    """
    PLAYING = auto()
    """
    The episode player is playing.
    """
    PAUSED = auto()
    """
    The episode player is paused.
    """
    STOPPED = auto()
    """
    The episode player is stopped.
    """


class DistanceFilter(Enum):
    MOVING_AVERAGE = auto()
    LOW_PASS = auto()


class MotionDetectionMethod(Enum):
    CONSISTENT_GRADIENT = auto()
    DISTANCE = auto()
