from abc import abstractmethod, ABC
from math import ceil, floor

import numpy as np
from typing_extensions import List, Tuple


class MotionDetectionMethod(ABC):
    """
    Interface for motion detection methods.
    """

    @abstractmethod
    def is_moving(self, latest_distances: List[List[float]]) -> Tuple[bool, int]:
        """
        Check if the object is moving.

        :param latest_distances: List of the latest distances.
        :return: True if the object is moving, False if it is not moving,
        and return the index in the given list of distances where the object started moving.
        """
        pass


class ConsistentGradient(MotionDetectionMethod):

    def __init__(self, threshold: float = 1e-4):
        self.threshold = threshold

    def is_moving(self, latest_distances: List[List[float]]) -> Tuple[bool, int]:
        """
        Check if the object is moving by checking if the distance between the current and the previous position is
        consistently positive or negative in at least one axis during the latest steps (the number of latest distances).
        """
        distance_arr = np.array(latest_distances)
        n_axes = distance_arr.shape[1]
        return any(np.all(distance_arr[:, i] > self.threshold) or np.all(distance_arr[:, i] < -self.threshold)
                   for i in range(n_axes)), 0


class Displacement(MotionDetectionMethod):

    def __init__(self, threshold: float):
        self.threshold = threshold

    def is_moving(self, latest_distances: List[List[float]]) -> Tuple[bool, int]:
        """
        Check if the object is moving by checking if the displacement between latest position and the start position is
        above a certain threshold.
        """
        avg_distance = np.linalg.norm(np.sum(np.array(latest_distances)))
        return avg_distance > self.threshold, floor((len(latest_distances) / 2) - 1)
