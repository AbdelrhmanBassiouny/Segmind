import time
from datetime import timedelta
import casadi as ca

import numpy as np
import pytest
from typing_extensions import List

from segmind.datastructures.events import (
    TranslationEvent,
    StopMotionEvent,
    StopTranslationEvent,
    ContactEvent,
)
from segmind.datastructures.object_tracker import ObjectTrackerFactory
from segmind.detectors.atomic_event_detectors import TranslationDetector
from segmind.detectors.coarse_event_detectors import GeneralPickUpDetector
from segmind.detectors.spatial_relation_detector import InsertionDetector
from segmind.detectors.motion_detection_helpers import (
    has_consistent_direction,
    is_displaced,
)
from segmind.event_logger import EventLogger

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

        fridge_shape = Box(scale=Scale(0.6, 0.6, 1.8), color=Color(0.2, 0.8, 0.2))
        fridge_body = Body(
            name=PrefixedName("fridge"),
            visual=ShapeCollection([fridge_shape]),
            collision=ShapeCollection([fridge_shape]),
        )
        fridge_conn = FixedConnection(
            parent=root,
            child=fridge_body,
            parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(
                x=0.5, y=0.0, z=0.9  # place fridge somewhere in the kitchen
            ),
        )
        world.add_connection(fridge_conn)

        # Target position (Point3)
        target_position = Point3.from_iterable([0.0, 0.5, 0.15])

    return world, milk_conn, robot_conn, table_conn, fridge_conn, target_position


def test_general_pick_up_start_condition_checker(
    create_kitchen_world_with_milk_and_robot,
):
    world, milk_conn, robot_conn, table_conn, _, target_position = (
        create_kitchen_world_with_milk_and_robot
    )
    robot_body = robot_conn.child
    event = ContactEvent(
        milk_conn.child, robot_body, 0.1
    )  # pass the Body, not the connection
    GeneralPickUpDetector.start_condition_checker(event)


def test_translation_detector(create_kitchen_world_with_milk_and_robot):
    world, milk_conn, robot_conn, table_conn, _, target_position = (
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
    world, milk_conn, robot_conn, table_conn, fridge_conn, target_position = (
        create_kitchen_world_with_milk_and_robot
    )
    milk_tracker = ObjectTrackerFactory.get_tracker(milk_conn.child)

    # Start translation detector with lowered thresholds
    translation_detector = run_and_get_translation_detector(
        milk_conn.child, time_between_frames=timedelta(seconds=0.01), world=world
    )
    translation_detector.min_translation_threshold = 1e-4
    translation_detector.min_velocity_threshold = 1e-4

    # Start spatial relation detector (InsertionDetector)
    sr_detector = InsertionDetector(wait_time=timedelta(seconds=0.01), world=world)
    sr_detector.start()

    # Proper containment check using bounding boxes
    def fridge_contains(body: Body):
        fridge_box = fridge_conn.child.collision.shapes[0]  # assume single Box
        milk_box = body.collision.shapes[0]

        fridge_pos = fridge_conn.origin.to_position().to_np()[:3]
        milk_pos = milk_conn.origin.to_position().to_np()[:3]

        fridge_min = fridge_pos - np.array(
            [fridge_box.scale.x / 2, fridge_box.scale.y / 2, fridge_box.scale.z / 2]
        )
        fridge_max = fridge_pos + np.array(
            [fridge_box.scale.x / 2, fridge_box.scale.y / 2, fridge_box.scale.z / 2]
        )

        milk_min = milk_pos - np.array(
            [milk_box.scale.x / 2, milk_box.scale.y / 2, milk_box.scale.z / 2]
        )
        milk_max = milk_pos + np.array(
            [milk_box.scale.x / 2, milk_box.scale.y / 2, milk_box.scale.z / 2]
        )

        return np.all(milk_min >= fridge_min) and np.all(milk_max <= fridge_max)

    try:
        # Initially milk is not in fridge
        print(f"Initial milk position: {milk_conn.origin.to_position().to_np()[:3]}")
        print(f"Fridge position: {fridge_conn.origin.to_position().to_np()[:3]}")
        assert not fridge_contains(milk_conn.child)

        # Move milk to fridge gradually in world coordinates
        fridge_pos = fridge_conn.origin.to_position().to_np()[:3]
        current_pos = milk_conn.origin.to_position().to_np()[:3]
        steps = 5  # enough steps to trigger the translation detector
        for i in range(steps):
            interp_pos = current_pos + (fridge_pos - current_pos) * (i + 1) / steps
            milk_conn.origin = TransformationMatrix.from_xyz_rpy(
                x=float(interp_pos[0]),
                y=float(interp_pos[1]),
                z=float(interp_pos[2]),
                reference_frame=table_conn.child,
            )
            world.notify_state_change()
            translation_detector.update_with_latest_motion_data()

            # Debug prints
            print(f"Step {i+1}/{steps} - Milk position: {interp_pos}")
            print(
                f"  Latest TranslationEvent: {milk_tracker.get_latest_event_of_type(TranslationEvent)}"
            )
            print(
                f"  Latest StopTranslationEvent: {milk_tracker.get_latest_event_of_type(StopTranslationEvent)}"
            )
            time.sleep(translation_detector.get_n_changes_wait_time(1) * 2)

        # Wait a bit to allow StopTranslationEvent & StopMotionEvent
        for j in range(5):
            translation_detector.update_with_latest_motion_data()
            world.notify_state_change()
            stop_motion_event = milk_tracker.get_latest_event_of_type(StopMotionEvent)
            print(f"Wait {j+1}/5 - StopMotionEvent: {stop_motion_event}")
            time.sleep(translation_detector.get_n_changes_wait_time(1) * 2)

        # Final assertions
        stop_motion_event = milk_tracker.get_latest_event_of_type(StopMotionEvent)
        assert stop_motion_event is not None, "StopMotionEvent was not detected!"
        assert fridge_contains(milk_conn.child), "Milk was not detected in the fridge!"


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
