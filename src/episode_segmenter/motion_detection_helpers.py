from abc import abstractmethod, ABC
from datetime import timedelta
from math import ceil, floor

import numpy as np
from typing_extensions import List, Tuple
from scipy.signal import butter, lfilter, sosfilt


class MotionDetectionMethod(ABC):
    """
    Interface for motion detection methods.
    """

    @abstractmethod
    def is_moving(self, latest_distances: List[List[float]]) -> bool:
        """
        Check if the object is moving.

        :param latest_distances: List of the latest distances.
        :return: True if the object is moving, False if it is not moving.
        """
        pass


class ConsistentGradient(MotionDetectionMethod):

    def __init__(self, threshold: float = 1e-4):
        self.threshold = threshold

    def is_moving(self, latest_distances: List[List[float]]) -> bool:
        """
        Check if the object is moving by checking if the distance between the current and the previous position is
        consistently positive or negative in at least one axis during the latest steps (the number of latest distances).
        """
        distance_arr = np.array(latest_distances)
        n_axes = distance_arr.shape[1]
        return any(np.all(distance_arr[:, i] > self.threshold) or np.all(distance_arr[:, i] < -self.threshold)
                   for i in range(n_axes))


class Displacement(MotionDetectionMethod):

    def __init__(self, threshold: float = 0.05):
        self.threshold = threshold

    def is_moving(self, latest_distances: List[List[float]]) -> bool:
        """
        Check if the object is moving by checking if the displacement between latest position and the start position is
        above a certain threshold.
        """
        avg_distance = np.linalg.norm(np.sum(np.array(latest_distances)))
        return avg_distance > self.threshold


class DataFilter(ABC):
    """
    Interface for data filters.
    """

    @abstractmethod
    def filter_data(self, data: np.ndarray) -> np.ndarray:
        """
        Filter the given data.

        :param data: The data to filter.
        :return: The filtered data.
        """
        pass


class LowPassFilter(DataFilter):

    def __init__(self, sampling_timestep: timedelta, cut_off_frequency: float = 2, order: int = 5):
        """
        A low-pass filter that filters the data with a given cut-off frequency.

        :param sampling_timestep: The time between each measurement/sample.
        :param cut_off_frequency: The cut-off frequency.
        :param order: The order of the filter.
        """
        self.sampling_timestep: timedelta = sampling_timestep
        self.cut_off_frequency: float = cut_off_frequency
        self.order: int = order

    def filter_data(self, data: np.ndarray) -> np.ndarray:
        """
        Apply a low-pass filter to the given data.
        """
        sos = butter(self.order, self.cut_off_frequency, fs=self.sampling_frequency, btype='low', output='sos',
                     analog=False)
        return sosfilt(sos, data, axis=0)

    @property
    def sampling_frequency(self) -> float:
        return 1 / self.sampling_timestep.total_seconds()


class ExponentialMovingAverage(DataFilter):

    def __init__(self, gamma: float):
        """
        An exponential moving average filter that filters the data with a given gamma (decay) value.

        :param gamma: The gamma value (i.e. decay coefficient).
        """
        self.gamma: float = gamma

    def filter_data(self, data: np.ndarray) -> np.ndarray:
        """
        Apply an exponential moving average filter to the given data.
        """
        return self.gamma * data
