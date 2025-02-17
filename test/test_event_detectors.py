import time
from unittest import TestCase

import numpy as np

from episode_segmenter.datastructures.events import TranslationEvent
from episode_segmenter.datastructures.object_tracker import ObjectTrackerFactory
from episode_segmenter.detectors.atomic_event_detectors import MotionDetector, TranslationDetector
from episode_segmenter.detectors.motion_detection_helpers import ConsistentGradient, Displacement
from episode_segmenter.detectors.spatial_relation_detector import SpatialRelationDetector
from episode_segmenter.event_logger import EventLogger
from pycram.testing import EmptyBulletWorldTestCase, BulletWorldTestCase
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import Milk


class TestEventDetectors(BulletWorldTestCase):

    def test_spatial_relation_detector(self):
        milk_tracker = ObjectTrackerFactory.get_tracker(self.milk)
        logger = EventLogger()
        translation_detector = TranslationDetector(logger, self.milk, Displacement(0.05))
        translation_detector.start()
        sr_detector = SpatialRelationDetector()
        sr_detector.start()
        try:
            self.assertFalse(self.kitchen.links["iai_fridge_main"].contains_body(self.milk))
            fridge_position = self.kitchen.links["iai_fridge_main"].position_as_list
            self.milk.set_position(fridge_position)
            time.sleep(0.2)
            self.assertTrue(milk_tracker.get_latest_event_of_type(TranslationEvent) is not None)
            self.assertTrue(self.kitchen.links["iai_fridge_main"].contains_body(self.milk))
        finally:
            translation_detector.stop()
            sr_detector.stop()
            translation_detector.join()
            sr_detector.join()

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
