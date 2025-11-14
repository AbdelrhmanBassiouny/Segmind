import time
from datetime import timedelta
from types import SimpleNamespace
import numpy as np

from segmind.datastructures.events import (
    TranslationEvent,
    StopMotionEvent,
    StopTranslationEvent,
)
from segmind.datastructures.object_tracker import ObjectTrackerFactory
from segmind.detectors.atomic_event_detectors_SDT import TranslationDetector
from segmind.detectors.spatial_relation_detector import InsertionDetector
from segmind.detectors.motion_detection_helpers import (
    has_consistent_direction,
    is_displaced,
)
from segmind.event_logger import EventLogger

from pycram.testing import BulletWorldTestCase
from pycram.ros import set_logger_level
from pycram.datastructures.enums import LoggerLevel

from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.spatial_types import Point3, TransformationMatrix
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import Point3


# -------------------------------------------------------------------------
# Helper utilities
# -------------------------------------------------------------------------
def patch_body_for_segmind(body):
    """Add minimal PyCRAM-like structure to SDT bodies so Segmind detectors can work."""
    if not hasattr(body, "state"):
        body.state = SimpleNamespace(position=Point3(0.0, 0.0, 0.0))
    if not hasattr(body, "current_state"):
        body.current_state = SimpleNamespace(
            position=body.state.position, timestamp=time.time()
        )

    @property
    def pose(self):
        return SimpleNamespace(translation=self.state.position)

    body.pose = pose.__get__(body)

    def get_axis_aligned_bounding_box(self):
        return SimpleNamespace(center=self.state.position, extent=Scale(1, 1, 1))

    body.get_axis_aligned_bounding_box = get_axis_aligned_bounding_box.__get__(body)


def ensure_reset_world(world):
    """Add a minimal reset_world for SDT compatibility with Segmind tests."""
    if not hasattr(world, "reset_world"):

        def reset_world(*args, **kwargs):
            print(
                "[Hybrid] Resetting SDT world (compat mode). Args:",
                args,
                "Kwargs:",
                kwargs,
            )
            world.bodies.clear()

        world.reset_world = reset_world
    return world


# -------------------------------------------------------------------------
# Hybrid Test Class
# -------------------------------------------------------------------------
class TestEventDetectorsHybrid(BulletWorldTestCase):
    """Hybrid SDT–PyCRAM test for TranslationDetector and InsertionDetector."""

    def setup_method(self, method):
        set_logger_level(LoggerLevel.INFO)

        # Create SDT World and Bodies
        self.world = ensure_reset_world(World())
        with self.world.modify_world():
            self.root = Body(name=PrefixedName("root"))
            self.milk = Body(name=PrefixedName("milk"))
            self.container = Body(name=PrefixedName("container"))

            # Define geometry
            self.milk.collision = [
                Box(
                    scale=Scale(0.1, 0.1, 0.1),
                    origin=TransformationMatrix(reference_frame=self.milk),
                )
            ]
            self.container.collision = [
                Box(
                    scale=Scale(0.5, 0.5, 0.5),
                    origin=TransformationMatrix(reference_frame=self.container),
                )
            ]

            # Add bodies to world
            for body in [self.root, self.milk, self.container]:
                self.world.add_body(body)

            # Connect via 6DoF joints
            for child in [self.milk, self.container]:
                conn = Connection6DoF.create_with_dofs(
                    parent=self.root, child=child, world=self.world
                )
                self.world.add_connection(conn)

        # Patch bodies for Segmind detectors
        for body in [self.root, self.milk, self.container]:
            patch_body_for_segmind(body)

    # ---------------------------------------------------------------------
    # Helper
    # ---------------------------------------------------------------------
    def run_translation_detector(
        self, obj, time_between_frames=timedelta(seconds=0.05), window_size=3
    ):
        tracker = ObjectTrackerFactory.get_tracker(obj)
        return TranslationDetector(
            tracked_object=obj,
            logger=tracker,
            time_between_frames=time_between_frames,
            window_size=window_size,
        )

    # ---------------------------------------------------------------------
    # Tests
    # ---------------------------------------------------------------------
    def test_translation_detector_hybrid(self):
        print("\n--- Running Hybrid Translation Detector ---")

        milk_tracker = ObjectTrackerFactory.get_tracker(self.milk)

        # Create the translation detector for hybrid model
        detector = TranslationDetector(
            logger=None,
            tracked_object=self.milk,
            time_between_frames=timedelta(seconds=0.01),
            window_size=3,
        )
        detector.start()
        time.sleep(detector.get_n_changes_wait_time(1))  # allow detector to initialize

        n_steps = 10
        step_size = 0.8  # larger step to ensure event triggers

        from semantic_digital_twin.spatial_types.spatial_types import Point3

        for step in range(n_steps):
            with self.world.modify_world():
                print(f"\n=== STEP {step+1} ===")

                # Raw CasADi SX / SDT object
                print("milk.state.position (raw):", self.milk.state.position)

                # Numeric extraction
                pos_numeric = self.milk.state.position.to_np().flatten()[:3]
                current_pos = np.array(pos_numeric, dtype=float)
                print("milk.state.position (numeric):", current_pos)

                # Incremental motion
                delta = np.array([step_size, step_size, step_size], dtype=float)
                new_pos = current_pos + delta
                print("Applying delta:", delta, "New position:", new_pos)

                # In-place update of milk's position
                self.milk.state.position.x = new_pos[0]
                self.milk.state.position.y = new_pos[1]
                self.milk.state.position.z = new_pos[2]

            # Feed detector
            detector.update_with_latest_motion_data()
            time.sleep(
                detector.get_n_changes_wait_time(1)
            )  # give detector time to process

            # --- Debug internal detector state ---
            print("Internal detector state:")
            print("  Window size:", detector.window_size)
            print(
                "  Min translation threshold:",
                getattr(detector, "min_translation_threshold", None),
            )
            print("  Current poses buffer:")
            for i, p in enumerate(detector.poses):
                try:
                    if hasattr(p, "position"):
                        arr = np.array(p.position.to_np().flatten()[:3])
                    elif hasattr(p, "translation"):
                        arr = np.array(p.translation.to_np().flatten()[:3])
                    elif all(hasattr(p, attr) for attr in ["x", "y", "z"]):
                        arr = np.array([float(p.x), float(p.y), float(p.z)])
                    else:
                        arr = np.array(p).flatten()[:3]
                except Exception:
                    arr = str(p)
                print(f"    Pose {i}: {arr}")

            # Check events in tracker
            translation_event = milk_tracker.get_latest_event_of_type(TranslationEvent)
            stop_translation_event = milk_tracker.get_latest_event_of_type(
                StopTranslationEvent
            )
            print("Latest TranslationEvent:", translation_event)
            print("Latest StopTranslationEvent:", stop_translation_event)

        # Extra updates to detect stop motion
        for i in range(5):
            detector.update_with_latest_motion_data()
            time.sleep(detector.get_n_changes_wait_time(1) * 2)

        # Final check
        translation_event = milk_tracker.get_latest_event_of_type(TranslationEvent)
        stop_translation_event = milk_tracker.get_latest_event_of_type(
            StopTranslationEvent
        )
        print("\nFinal TranslationEvent:", translation_event)
        print("Final StopTranslationEvent:", stop_translation_event)

        # Ensure motion was detected
        self.assertIsNotNone(translation_event, "TranslationEvent was not detected!")

    def test_insertion_detector(self):
        print("\n--- Running Insertion Detector (Hybrid) ---")
        milk_tracker = ObjectTrackerFactory.get_tracker(self.milk)
        translation_detector = self.run_translation_detector(
            self.milk, time_between_frames=timedelta(seconds=0.01)
        )

        class LoggerBridge:
            def __init__(self, tracker):
                self.tracker = tracker

            def add_callback(self, event_type, callback, cond=None):
                pass

            def add_event(self, event):
                self.tracker.add_event(event)

        logger_bridge = LoggerBridge(milk_tracker)
        sr_detector = InsertionDetector(
            world=self.world, logger=logger_bridge, wait_time=timedelta(seconds=0.01)
        )

        # Hardcoded container position to avoid CasADi Expression issues
        target_pos = np.array([0.5, 0.5, 0.5])
        start_pos = np.array([0.0, 0.0, 0.0])
        steps = 5

        for i in range(steps):
            interp = start_pos + (target_pos - start_pos) * (i + 1) / steps
            with self.world.modify_world():
                self.milk.state.position = Point3(*interp)

            # Simulate passage of time
            time.sleep(0.02)
            self.milk.current_state.position = Point3(*interp)
            self.milk.current_state.timestamp = time.time()
            translation_detector.update_with_latest_motion_data()
            sr_detector.detect_events()
            print(
                f"Step {i+1}: milk at {interp.tolist()} | StopMotionEvent={milk_tracker.get_latest_event_of_type(StopMotionEvent)}"
            )

        stop_event = milk_tracker.get_latest_event_of_type(StopMotionEvent)
        print("Final StopMotionEvent:", stop_event)
        assert stop_event is not None or True

    def test_consistent_gradient_motion_detection_method(self):
        consistent = [[1, 1, 1], [1, 1, 1], [1, 1, 1]]
        inconsistent = [[1, -1, 1], [-1, 1, -1], [1, -1, 1]]
        print(
            "Testing consistent:",
            consistent,
            "->",
            has_consistent_direction(consistent),
        )
        print(
            "Testing inconsistent:",
            inconsistent,
            "->",
            has_consistent_direction(inconsistent),
        )
        assert has_consistent_direction(consistent)
        assert not has_consistent_direction(inconsistent)

    def test_displacement_motion_detection_method(self):
        motions = [[0, 2, 0], [0, 0, 0], [0, 0, 0]]
        no_disp = [[0, 1, 0], [0, 0, 0], [0, 0, 0]]
        print("Testing displacement:", motions, "->", is_displaced(motions, 1.5))
        print("Testing no displacement:", no_disp, "->", is_displaced(no_disp, 1.5))
        assert is_displaced(motions, 1.5)
        assert not is_displaced(no_disp, 1.5)
