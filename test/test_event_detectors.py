import time
from datetime import timedelta

import numpy as np
from typing_extensions import List, Type

from segmind.datastructures.events import TranslationEvent, StopMotionEvent, StopTranslationEvent, \
    ContactEvent, Event, EventUnion
from segmind.datastructures.object_tracker import ObjectTrackerFactory
from segmind.detectors.atomic_event_detectors import TranslationDetector, AtomicEventDetector
from segmind.detectors.coarse_event_detectors import GeneralPickUpDetector
from segmind.detectors.motion_detection_helpers import ConsistentGradient, Displacement
from segmind.detectors.spatial_relation_detector import InsertionDetector
from segmind.event_logger import EventLogger
from pycram.testing import BulletWorldTestCase
from pycram.datastructures.enums import LoggerLevel
from pycram.ros import set_logger_level
from pycram.world_concepts.world_object import Object

# set_logger_level(LoggerLevel.DEBUG)


class TestEventDetectors(BulletWorldTestCase):

    def test_general_pick_up_start_condition_checker(self):
        event = ContactEvent(self.milk, self.robot, 0.1)
        GeneralPickUpDetector.start_condition_checker(event)

    def test_translation_detector(self):
        milk_tracker = ObjectTrackerFactory.get_tracker(self.milk)
        translation_detector = self.run_and_get_translation_detector(self.milk)

        try:
            fridge_position = self.kitchen.links["iai_fridge_main"].position.to_list()
            self.milk.set_position(fridge_position)

            # wait one timestep to detect that it is moving
            time.sleep(translation_detector.get_n_changes_wait_time(1))
            translation_event = milk_tracker.get_latest_event_of_type(TranslationEvent)
            self.assertTrue(translation_event is not None)

            # wait one timestep to detect that it is not moving
            time.sleep(translation_detector.get_n_changes_wait_time(1))
            self.assertTrue(milk_tracker.get_first_event_of_type_after_event(StopTranslationEvent, translation_event)
                            is not None)
        except Exception as e:
            raise e
        finally:
            translation_detector.stop()
            translation_detector.join()

    def test_insertion_detector(self):
        milk_tracker = ObjectTrackerFactory.get_tracker(self.milk)
        time_between_frames = timedelta(seconds=0.01)
        translation_detector = self.run_and_get_translation_detector(self.milk, time_between_frames)

        sr_detector = InsertionDetector(wait_time=time_between_frames)
        sr_detector.start()

        try:
            self.assertFalse(self.kitchen.links["iai_fridge_main"].contains_body(self.milk))
            fridge_position = self.kitchen.links["iai_fridge_main"].position.to_list()
            self.milk.set_position(fridge_position)
            # because milk goes to moving state then to stop state thus we need to wait for 2 changes
            time.sleep(translation_detector.get_n_changes_wait_time(2))
            self.assertTrue(milk_tracker.get_latest_event_of_type(StopMotionEvent) is not None)
            self.assertTrue(self.kitchen.links["iai_fridge_main"].contains_body(self.milk))
        except Exception as e:
            raise e
        finally:
            translation_detector.stop()
            sr_detector.stop()
            translation_detector.join()
            sr_detector.join()

    @staticmethod
    def run_and_get_translation_detector(obj: Object, time_between_frames: timedelta = timedelta(seconds=0.01))\
            -> TranslationDetector:
        logger = EventLogger()
        translation_detector = TranslationDetector(logger, obj,
                                                   detection_method=Displacement(0.01),
                                                   time_between_frames=time_between_frames,
                                                   window_size=2)
        translation_detector.start()
        # wait one timestep to detect the initial state
        time.sleep(translation_detector.get_n_changes_wait_time(1))
        return translation_detector

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
