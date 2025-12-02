from abc import abstractmethod, ABC
from datetime import timedelta

import numpy as np
from scipy.signal import butter, sosfilt
from typing_extensions import List, Callable
from typing import Dict


Distances = List[List[float]]
"""
Type for lists of distances.
"""
MotionDetectionMethod = Callable[[Distances, float], bool]
"""
Interface for motion detection methods, it takes a list of distances and a threshold and returns a boolean value.
"""


def has_consistent_direction(
    latest_distances: Distances, threshold: float = 1e-4
) -> bool:
    """
    Check if the object is moving by checking if the distance between the current and the previous position is
    consistently positive or negative in at least one axis during the latest steps (the number of latest distances).
    """
    distance_arr = np.array(latest_distances)
    n_axes = distance_arr.shape[1]
    return any(
        np.all(distance_arr[:, i] > threshold)
        or np.all(distance_arr[:, i] < -threshold)
        for i in range(n_axes)
    )


def is_displaced(
    latest_distances: Distances,
    velocity_threshold: float = 0.05,
    window_size_seconds: float = 1.0,
) -> bool:
    """
    Determine if the object is moving based on average step velocity.
    Handles back-and-forth motion correctly.
    """
    if len(latest_distances) < 2:
        return False

    distance_arr = np.array(latest_distances)
    step_magnitudes = np.linalg.norm(distance_arr, axis=1)
    timestep_seconds = window_size_seconds / len(step_magnitudes)
    avg_velocity = step_magnitudes.mean() / timestep_seconds

    return avg_velocity >= velocity_threshold


def is_stopped(
    latest_distances: Distances,
    velocity_threshold: float = 0.01,
    window_size_seconds: float = 1.0,
) -> bool:
    """
    Determine if the object is stopped based on average step velocity.
    Uses same metric as is_displaced for consistency.
    """
    if len(latest_distances) < 2:
        return True

    distance_arr = np.array(latest_distances)
    step_magnitudes = np.linalg.norm(distance_arr, axis=1)
    timestep_seconds = window_size_seconds / len(step_magnitudes)
    avg_velocity = step_magnitudes.mean() / timestep_seconds

    return avg_velocity <= velocity_threshold


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

    def __init__(
        self, sampling_timestep: timedelta, cut_off_frequency: float = 2, order: int = 5
    ):
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
        sos = butter(
            self.order,
            self.cut_off_frequency,
            fs=self.sampling_frequency,
            btype="low",
            output="sos",
            analog=False,
        )
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
