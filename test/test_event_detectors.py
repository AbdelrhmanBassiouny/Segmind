import time
from datetime import timedelta

import numpy as np
from typing_extensions import List

from segmind.datastructures.events import TranslationEvent, StopMotionEvent, StopTranslationEvent, \
    ContactEvent
from segmind.datastructures.object_tracker import ObjectTrackerFactory
from segmind.detectors.atomic_event_detectors import TranslationDetector
from segmind.detectors.coarse_event_detectors import GeneralPickUpDetector
from segmind.detectors.spatial_relation_detector import InsertionDetector
from segmind.detectors.motion_detection_helpers import has_consistent_direction, is_displaced
from segmind.event_logger import EventLogger
from pycram.testing import BulletWorldTestCase
from pycram.ros import set_logger_level
from pycram.datastructures.enums import LoggerLevel
from pycram.world_concepts.world_object import Object


# set_logger_level(LoggerLevel.DEBUG)

class TestEventDetectors(BulletWorldTestCase):

    def test_general_pick_up_start_condition_checker(self):
        event = ContactEvent(self.milk, self.robot, 0.1)
        GeneralPickUpDetector.start_condition_checker(event)

    def test_translation_detector(self):
        milk_tracker = ObjectTrackerFactory.get_tracker(self.milk)
        translation_detector = self.run_and_get_translation_detector(self.milk,
                                                                     time_between_frames=timedelta(seconds=0.05))

        # Target position: fridge
        fridge_position = self.kitchen.links["iai_fridge_main"].position.to_list()
        print("Fridge position for translation test:", fridge_position)

        # Gradually move milk to fridge to simulate motion
        current = self.milk.get_position().to_list()
        steps = 3
        for i in range(steps):
            interp = [current[j] + (fridge_position[j] - current[j]) * (i + 1) / steps for j in range(3)]
            self.milk.set_position(interp)
            translation_detector.update_with_latest_motion_data()
            time.sleep(translation_detector.get_n_changes_wait_time(1) * 2)
            print(f"Step {i + 1} pose history:", [p.to_list() for p in translation_detector.poses])
            event = milk_tracker.get_latest_event_of_type(TranslationEvent)
            print(f"Step {i + 1}, latest TranslationEvent:", event)

        # Extra updates at final pose for stop detection
        for _ in range(10):
            translation_detector.update_with_latest_motion_data()
            time.sleep(translation_detector.get_n_changes_wait_time(1) * 2)

        translation_event = milk_tracker.get_latest_event_of_type(TranslationEvent)
        stop_translation_event = milk_tracker.get_latest_event_of_type(StopTranslationEvent)
        print("Final TranslationEvent:", translation_event)
        print("StopTranslationEvent after motion:", stop_translation_event)

        self.assertIsNotNone(translation_event, "TranslationEvent was not detected!")
        self.assertIsNotNone(stop_translation_event, "StopTranslationEvent was not detected!")

    def test_insertion_detector(self):
        milk_tracker = ObjectTrackerFactory.get_tracker(self.milk)
        time_between_frames = timedelta(seconds=0.05)
        translation_detector = self.run_and_get_translation_detector(self.milk, time_between_frames)
        sr_detector = InsertionDetector(wait_time=time_between_frames)
        sr_detector.start()

        fridge_position = self.kitchen.links["iai_fridge_main"].position.to_list()
        print("Fridge position for insertion test:", fridge_position)

        # Gradually move milk into fridge
        current = self.milk.get_position().to_list()
        steps = 3
        for i in range(steps):
            interp = [current[j] + (fridge_position[j] - current[j]) * (i + 1) / steps for j in range(3)]
            self.milk.set_position(interp)
            translation_detector.update_with_latest_motion_data()
            time.sleep(translation_detector.get_n_changes_wait_time(1) * 2)
            print(f"Step {i + 1} pose history:", [p.to_list() for p in translation_detector.poses])
            event = milk_tracker.get_latest_event_of_type(StopMotionEvent)
            print(f"Step {i + 1}, latest StopMotionEvent:", event)

        # Extra updates at final pose for stop detection
        for _ in range(10):
            translation_detector.update_with_latest_motion_data()
            time.sleep(translation_detector.get_n_changes_wait_time(1) * 2)

        stop_motion_event = milk_tracker.get_latest_event_of_type(StopMotionEvent)
        print("Final StopMotionEvent before assert:", stop_motion_event)
        self.assertIsNotNone(stop_motion_event, "StopMotionEvent was not detected!")

    @staticmethod
    def run_and_get_translation_detector(obj: Object, time_between_frames: timedelta = timedelta(
        seconds=0.01)) -> TranslationDetector:
        logger = EventLogger()
        translation_detector = TranslationDetector(logger, obj,
                                                   time_between_frames=time_between_frames,
                                                   window_size=2)
        translation_detector.start()
        time.sleep(translation_detector.get_n_changes_wait_time(1))
        return translation_detector

    def test_consistent_gradient_motion_detection_method(self):
        for i in range(3):
            a = np.zeros((3, 3))
            a[:, i] = 1
            self.assertTrue(has_consistent_direction(a.tolist()))
            a = np.zeros((3, 3))
            a[:, i] = -1
            self.assertTrue(has_consistent_direction(a.tolist()))
            a = np.zeros((3, 3))
            a[:, i] = -1
            a[1, i] = 1
            self.assertFalse(has_consistent_direction(a.tolist()))

    def test_displacement_motion_detection_method(self):
        for i in range(3):
            a = np.zeros((3, 3))
            a[:, i] = 1
            self.assertTrue(is_displaced(a.tolist(), 1.5))
            a = np.zeros((3, 3))
            a[:, i] = -1
            self.assertTrue(is_displaced(a.tolist(), 1.5))
            a = np.zeros((3, 3))
            a[:, i] = -1
            a[1, i] = 1
            self.assertFalse(is_displaced(a.tolist(), 1.5))