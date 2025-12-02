import time
from datetime import timedelta
import casadi as ca

import numpy as np
import pytest
from typing_extensions import List

from segmind.datastructures.events_SDT import (
    TranslationEvent,
    StopMotionEvent,
    StopTranslationEvent,
    ContactEvent,
)
from segmind.datastructures.object_tracker_SDT import ObjectTrackerFactory
from segmind.detectors.atomic_event_detectors_SDT2 import TranslationDetector
from segmind.detectors.coarse_event_detectors_SDT import GeneralPickUpDetector
from segmind.detectors.spatial_relation_detector_SDT import InsertionDetector
from segmind.detectors.motion_detection_helpers import (
    has_consistent_direction,
    is_displaced,
)
from segmind.event_logger_SDT import EventLogger

from semantic_digital_twin.world import World
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.connections import (
    FixedConnection,
    Connection6DoF,
)
from semantic_digital_twin.spatial_types.spatial_types import (
    TransformationMatrix,
    Point3,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.geometry import Box, Scale, Color


@pytest.fixture
def create_kitchen_world_with_milk_and_robot():
    world = World()

    with world.modify_world():
        # Root
        root = Body(name=PrefixedName("root"))

        # Robot (FixedConnection)
        robot_shape = Box(scale=Scale(1.0, 0.5, 0.7), color=Color(0.7, 0.5, 0.3))
        robot_body = Body(
            name=PrefixedName("robot"),
            visual=ShapeCollection([robot_shape]),
            collision=ShapeCollection([robot_shape]),
        )
        robot_conn = FixedConnection(
            parent=root,
            child=robot_body,
            parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(
                x=0, y=0, z=1.6
            ),
        )
        world.add_connection(robot_conn)

        # Table (FixedConnection)
        table_shape = Box(scale=Scale(1.0, 0.5, 0.7), color=Color(0.7, 0.5, 0.3))
        table_body = Body(
            name=PrefixedName("table"),
            visual=ShapeCollection([table_shape]),
            collision=ShapeCollection([table_shape]),
        )
        table_conn = FixedConnection(
            parent=root,
            child=table_body,
            parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(
                x=0, y=0, z=1.0
            ),
        )
        world.add_connection(table_conn)

        # Milk (6DoF Connection)
        milk_shape = Box(scale=Scale(0.1, 0.1, 0.2), color=Color(1.0, 1.0, 1.0))
        milk_body = Body(
            name=PrefixedName("milk"),
            visual=ShapeCollection([milk_shape]),
            collision=ShapeCollection([milk_shape]),
        )
        milk_conn = Connection6DoF.create_with_dofs(
            world=world,
            parent=table_body,
            child=milk_body,
            parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(
                x=0, y=0, z=0.15
            ),
        )
        world.add_connection(milk_conn)

        # Target position (Point3)
        target_position = Point3.from_iterable([0.0, 0.5, 0.15])

    return world, milk_conn, robot_conn, table_conn, target_position


def test_general_pick_up_start_condition_checker(
    create_kitchen_world_with_milk_and_robot,
):
    world, milk_conn, robot_conn, table_conn, target = (
        create_kitchen_world_with_milk_and_robot
    )
    robot_body = robot_conn.child
    event = ContactEvent(
        milk_conn.child, robot_body, 0.1
    )  # pass the Body, not the connection
    GeneralPickUpDetector.start_condition_checker(event)


def test_translation_detector(create_kitchen_world_with_milk_and_robot):
    world, milk_conn, robot_conn, table_conn, target_position = (
        create_kitchen_world_with_milk_and_robot
    )
    milk_tracker = ObjectTrackerFactory.get_tracker(milk_conn.child)

    translation_detector = run_and_get_translation_detector(
        milk_conn.child, world=world
    )

    # Optionally run only a single loop (useful for some setups)
    # translation_detector.run_once = True

    try:
        steps = 5
        for i in range(steps):
            current_pos = milk_conn.origin.to_position().to_np()[:3]
            target_pos = target_position.to_np()[:3]
            interp_pos = current_pos + (target_pos - current_pos) * (i + 1) / steps

            milk_conn.origin = TransformationMatrix.from_xyz_rpy(
                x=float(interp_pos[0]),
                y=float(interp_pos[1]),
                z=float(interp_pos[2]),
                reference_frame=table_conn.child,
            )
            world.notify_state_change()
            translation_detector.update_with_latest_motion_data()

            print(f"Step {i+1}/{steps} - Milk position: {interp_pos}")
            print(
                f"  Latest TranslationEvent: {milk_tracker.get_latest_event_of_type(TranslationEvent)}"
            )
            print(
                f"  Latest StopTranslationEvent: {milk_tracker.get_latest_event_of_type(StopTranslationEvent)}"
            )

            time.sleep(translation_detector.get_n_changes_wait_time(1) * 2)

        # Simulate no movement to trigger StopTranslationEvent
        for j in range(5):
            translation_detector.update_with_latest_motion_data()
            world.notify_state_change()
            print(
                f"Wait {j+1}/5 - Latest StopTranslationEvent: {milk_tracker.get_latest_event_of_type(StopTranslationEvent)}"
            )
            time.sleep(translation_detector.get_n_changes_wait_time(1) * 2)

        assert milk_tracker.get_latest_event_of_type(TranslationEvent) is not None
        assert milk_tracker.get_latest_event_of_type(StopTranslationEvent) is not None

    finally:
        # Always stop and join the detector thread
        try:
            translation_detector.stop()
        except Exception:
            pass
        try:
            translation_detector.join(timeout=2.0)
        except Exception:
            pass


def test_insertion_detector(create_kitchen_world_with_milk_and_robot):
    world, milk_conn, robot_conn, table_conn, target_position = (
        create_kitchen_world_with_milk_and_robot
    )
    milk_tracker = ObjectTrackerFactory.get_tracker(milk_conn.child)

    # Start translation detector with small thresholds for testing
    translation_detector = run_and_get_translation_detector(
        milk_conn.child, time_between_frames=timedelta(seconds=0.01), world=world
    )

    # Start spatial relation detector (InsertionDetector)
    sr_detector = InsertionDetector(wait_time=timedelta(seconds=0.01), world=world)

    # Optionally run only a single loop for SR detector as well:
    # sr_detector.run_once = True

    sr_detector.start()

    try:
        steps = 5
        for i in range(steps):
            # Linear interpolation between start and target positions
            current_pos = milk_conn.origin.to_position().to_np()[:3].flatten()
            target_pos_np = target_position.to_np()[:3].flatten()
            interp_pos = current_pos + (target_pos_np - current_pos) * (i + 1) / steps

            # Update the origin (mutable) instead of global_pose
            milk_conn.origin = TransformationMatrix.from_xyz_rpy(
                x=float(interp_pos[0]),
                y=float(interp_pos[1]),
                z=float(interp_pos[2]),
                reference_frame=table_conn.child,
            )
            world.notify_state_change()

            # Let translation detector process current frame
            translation_detector.update_with_latest_motion_data()

            # Debug prints
            print(f"Step {i+1}/{steps}")
            print(f"  Milk position: {interp_pos}")
            latest_translation_event = milk_tracker.get_latest_event_of_type(
                TranslationEvent
            )
            print(f"  Latest TranslationEvent: {latest_translation_event}")

            # Wait enough time for detector to register motion
            time.sleep(translation_detector.get_n_changes_wait_time(1) * 2)

        # Simulate multiple frames with NO movement to trigger StopTranslationEvent
        for j in range(10):
            # Keep object still
            translation_detector.update_with_latest_motion_data()
            world.notify_state_change()

            latest_translation_event = milk_tracker.get_latest_event_of_type(
                TranslationEvent
            )
            latest_stop_event = milk_tracker.get_latest_event_of_type(
                StopTranslationEvent
            )

            print(f"Wait {j+1}/10: Latest TranslationEvent: {latest_translation_event}")
            print(f"Wait {j+1}/10: Latest StopTranslationEvent: {latest_stop_event}")

            # Wait for detector to process
            time.sleep(translation_detector.get_n_changes_wait_time(1) * 2)

        stop_motion_event = milk_tracker.get_latest_event_of_type(StopTranslationEvent)
        print(f"Final StopTranslationEvent: {stop_motion_event}")
        assert stop_motion_event is not None, "StopTranslationEvent was not detected!"

    finally:
        # Always stop/join both detectors
        for d in (translation_detector, sr_detector):
            try:
                d.stop()
            except Exception:
                pass
            try:
                d.join(timeout=2.0)
            except Exception:
                pass


def run_and_get_translation_detector(
    obj: Body,
    time_between_frames: timedelta = timedelta(seconds=0.01),
    world: World = None,
) -> TranslationDetector:
    """
    Create and start a TranslationDetector for the given object, with lowered
    thresholds for testing so even small movements trigger events.
    """
    logger = EventLogger()
    translation_detector = TranslationDetector(
        logger,
        obj,
        time_between_frames=time_between_frames,
        window_size_in_seconds=0.05,  # smaller window for faster detection in tests
        world=world,
    )

    # Lower thresholds for test purposes
    translation_detector.min_translation_threshold = 1e-3  # 1 mm
    translation_detector.min_velocity_threshold = 1e-3  # 1 mm/s

    translation_detector.start()
    # Wait a bit to let the detector initialize
    time.sleep(translation_detector.get_n_changes_wait_time(1))
    return translation_detector


def test_consistent_gradient_motion_detection_method():
    for i in range(3):
        a = np.zeros((3, 3))
        a[:, i] = 1
        assert has_consistent_direction(a.tolist())
        a = np.zeros((3, 3))
        a[:, i] = -1
        assert has_consistent_direction(a.tolist())
        a = np.zeros((3, 3))
        a[:, i] = -1
        a[1, i] = 1
        assert not has_consistent_direction(a.tolist())


def test_displacement_motion_detection_method():
    for i in range(3):
        a = np.zeros((3, 3))
        a[:, i] = 1
        assert is_displaced(a.tolist(), 1.5)
        a = np.zeros((3, 3))
        a[:, i] = -1
        assert is_displaced(a.tolist(), 1.5)
        a = np.zeros((3, 3))
        a[:, i] = -1
        a[1, i] = 1
        assert is_displaced(a.tolist(), 1.5)
