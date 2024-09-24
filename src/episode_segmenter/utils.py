import numpy as np
from tf.transformations import quaternion_inverse, quaternion_multiply
from typing_extensions import List

from pycram.datastructures.pose import Transform


def get_angle_between_vectors(vector_1: List[float], vector_2: List[float]) -> float:
    """
    Get the angle between two vectors.

    :param vector_1: A list of float values that represent the first vector.
    :param vector_2: A list of float values that represent the second vector.
    :return: A float value that represents the angle between the two vectors.
    """
    return np.arccos(np.dot(vector_1, vector_2) / (np.linalg.norm(vector_1) * np.linalg.norm(vector_2)))


def calculate_transform_difference_and_check_if_small(transform_1: Transform, transform_2: Transform,
                                                      translation_threshold: float, angle_threshold: float) -> bool:
    """
    Calculate the translation and rotation of the object with respect to the hand to check if it was picked up,
     uses the translation and rotation thresholds to determine if the object was picked up.

    :param transform_1: The transform of the object at the first time step.
    :param transform_2: The transform of the object at the second time step.
    :param translation_threshold: The threshold for the translation difference to be considered as small.
    :param angle_threshold: The threshold for the angle between the two quaternions to be considered as small.
    :return: A tuple of two boolean values that represent the conditions for the translation and rotation of the
    object to be considered as picked up.
    """
    trans_1, quat_1 = transform_1.translation_as_list(), transform_1.rotation_as_list()
    trans_2, quat_2 = transform_2.translation_as_list(), transform_2.rotation_as_list()
    trans_diff_cond = calculate_translation_difference_and_check(trans_1, trans_2, translation_threshold)
    rot_diff_cond = calculate_angle_between_quaternions_and_check(quat_1, quat_2, angle_threshold)
    return trans_diff_cond and rot_diff_cond


def calculate_translation_difference_and_check(trans_1: List[float], trans_2: List[float],
                                               threshold: float) -> bool:
    """
    Calculate the translation difference and checks if it is small.

    :param trans_1: The translation of the object at the first time step.
    :param trans_2: The translation of the object at the second time step.
    :param threshold: The threshold for the translation difference to be considered as small.
    :return: A boolean value that represents the condition for the translation of the object to be considered as
    picked up.
    """
    translation_diff = calculate_translation_difference(trans_1, trans_2)
    return is_translation_difference_small(translation_diff, threshold)


def is_translation_difference_small(trans_diff: List[float], threshold: float) -> bool:
    """
    Check if the translation difference is small by comparing it to the translation threshold.

    :param trans_diff: The translation difference.
    :param threshold: The threshold for the translation difference to be considered as small.
    :return: A boolean value that represents the condition for the translation difference to be considered as small.
    """
    return all([diff <= threshold for diff in trans_diff])


def calculate_translation_difference(trans_1: List[float], trans_2: List[float]) -> List[float]:
    """
    Calculate the translation difference.

    :param trans_1: The translation of the object at the first time step.
    :param trans_2: The translation of the object at the second time step.
    :return: A list of float values that represent the translation difference.
    """
    return [abs(t1 - t2) for t1, t2 in zip(trans_1, trans_2)]


def calculate_angle_between_quaternions_and_check(quat_1: List[float], quat_2: List[float], threshold: float) -> bool:
    """
    Calculate the angle between two quaternions and checks if it is small.

    :param quat_1: The first quaternion.
    :param quat_2: The second quaternion.
    :param threshold: The threshold for the angle between the two quaternions to be considered as small.
    :return: A boolean value that represents the condition for the angle between the two quaternions
     to be considered as small.
    """
    quat_diff_angle = calculate_angle_between_quaternions(quat_1, quat_2)
    return quat_diff_angle <= threshold


def calculate_angle_between_quaternions(quat_1: List[float], quat_2: List[float]) -> float:
    """
    Calculate the angle between two quaternions.

    :param quat_1: The first quaternion.
    :param quat_2: The second quaternion.
    :return: A float value that represents the angle between the two quaternions.
    """
    quat_diff = calculate_quaternion_difference(quat_1, quat_2)
    quat_diff_angle = 2 * np.arctan2(np.linalg.norm(quat_diff[0:3]), quat_diff[3])
    return quat_diff_angle


def calculate_quaternion_difference(quat_1: List[float], quat_2: List[float]) -> List[float]:
    """
    Calculate the quaternion difference.

    :param quat_1: The quaternion of the object at the first time step.
    :param quat_2: The quaternion of the object at the second time step.
    :return: A list of float values that represent the quaternion difference.
    """
    quat_diff = quaternion_multiply(quaternion_inverse(quat_1), quat_2)
    return quat_diff
