from unittest import TestCase

import numpy as np

from episode_segmenter.motion_detection_helpers import ConsistentGradient, Displacement
from episode_segmenter.event_detectors import *


class TestEventDetectors(TestCase):

    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_consistent_gradient_motion_detection_method(self):
        for i in range(3):
            a = np.zeros((3, 3))
            a[:, i] = 1
            cg = ConsistentGradient()
            self.assertTrue(cg.is_moving(a.tolist()))
            a = np.zeros((3, 3))
            a[:, i] = -1
            self.assertTrue(cg.is_moving(a.tolist()))
            a = np.zeros((3, 3))
            a[:, i] = -1
            a[1, i] = 1
            self.assertFalse(cg.is_moving(a.tolist()))

    def test_displacement_motion_detection_method(self):
        for i in range(3):
            a = np.zeros((3, 3))
            a[:, i] = 1
            disp = Displacement(1.5)
            self.assertTrue(disp.is_moving(a.tolist()))
            a = np.zeros((3, 3))
            a[:, i] = -1
            self.assertTrue(disp.is_moving(a.tolist()))
            a = np.zeros((3, 3))
            a[:, i] = -1
            a[1, i] = 1
            self.assertFalse(disp.is_moving(a.tolist()))
