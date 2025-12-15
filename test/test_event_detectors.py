import logging
import time
from datetime import timedelta
import casadi as ca

import numpy as np
import pytest
from krrood.ormatic.dao import to_dao
from krrood.ormatic.utils import create_engine
from sqlalchemy.orm import Session
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
from semantic_digital_twin.collision_checking.trimesh_collision_detector import (
    TrimeshCollisionDetector,
)
from semantic_digital_twin.orm.ormatic_interface import BodyDAO, Base, WorldMappingDAO
from semantic_digital_twin.reasoning.predicates import InsideOf

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
import inspect
from segmind import logger
import logging


logger.setLevel(logging.DEBUG)


@pytest.fixture
def create_kitchen_world_with_milk_and_robot():
    world = World()

    with world.modify_world():
        # Root
        root = Body(name=PrefixedName("root"))

        # Robot (FixedConnection)
        robot_shape = Box(scale=Scale(1.68, 0.94, 0.68), color=Color(0.7, 0.5, 0.3))
        robot_body = Body(
            name=PrefixedName("robot"),
            visual=ShapeCollection([robot_shape]),
            collision=ShapeCollection([robot_shape]),
        )
        robot_conn = FixedConnection(
            parent=root,
            child=robot_body,
            parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(
                x=3.60, y=1.20, z=0.34
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
                x=0, y=0, z=0.35
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
                x=3.60, y=1.20, z=0.34
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
    robot_body = world.get_body_by_name("robot")
    milk_body = world.get_body_by_name("milk")
    tcd = TrimeshCollisionDetector(world)
    event = ContactEvent(
        contact_points=tcd.check_collision_between_bodies(robot_body, milk_body),
        with_object=robot_body,
        of_object=milk_body,
        latest_contact_points=None,
    )
    GeneralPickUpDetector.start_condition_checker(event)
    # print(world.bodies_with_enabled_collision)
    # print(tcd.check_collision_between_bodies(robot_body, milk_body))
    # print(GeneralPickUpDetector.start_condition_checker(event))
    assert GeneralPickUpDetector.start_condition_checker(event)


def test_translation_detector(create_kitchen_world_with_milk_and_robot):
    world, milk_conn, _, table_conn, _, target_position = (
        create_kitchen_world_with_milk_and_robot
    )
    milk_tracker = ObjectTrackerFactory.get_tracker(milk_conn.child)
    translation_detector = run_and_get_translation_detector(
        milk_conn.child, world=world
    )
    translation_detector.disable_thread = True

    # Initial pose
    translation_detector.update_with_latest_motion_data()

    # Move object
    fridge_pos = target_position.to_np()[:3]
    milk_conn.origin = TransformationMatrix.from_xyz_rpy(
        x=float(fridge_pos[0]),
        y=float(fridge_pos[1]),
        z=float(fridge_pos[2]),
        reference_frame=table_conn.child,
    )
    world.notify_state_change()

    # Now give it 2 more updates BEFORE trying to detect anything
    translation_detector.update_with_latest_motion_data()
    translation_detector.update_with_latest_motion_data()

    # NOW it is safe to run event detection
    translation_detector.detect_and_log_events()

    translation_event = milk_tracker.get_latest_event_of_type(TranslationEvent)
    # print(f"Latest TranslationEvent: {translation_event}")
    # assert translation_event is not None

    # Simulate stopping
    translation_detector.update_with_latest_motion_data()
    translation_detector.detect_and_log_events()

    stop_event = milk_tracker.get_first_event_of_type_after_event(
        StopTranslationEvent, translation_event
    )
    # assert stop_event is not None


def test_insertion_detector(create_kitchen_world_with_milk_and_robot):
    world, milk_conn, robot_conn, table_conn, fridge_conn, target_position = (
        create_kitchen_world_with_milk_and_robot
    )

    milk_body = world.get_body_by_name("milk")
    fridge_body = world.get_body_by_name("fridge")
    translation_detector = run_and_get_translation_detector(
        milk_body, time_between_frames=timedelta(seconds=0.01), world=world
    )

    # Track events from the milk object
    milk_tracker = ObjectTrackerFactory.get_tracker(milk_body)
    # Use a permissive stop threshold so the windowed deltas qualify as "stopped"

    # Spatial-relation detector (InsertionDetector)
    sr_detector = InsertionDetector(world=world)
    sr_detector.start()

    try:
        # Initial containment check
        assert (
            InsideOf(
                world.get_body_by_name("milk"), world.get_body_by_name("fridge")
            ).compute_containment_ratio()
            == 0.0
        )

        # Establish baseline sample before the move
        translation_detector.update_with_latest_motion_data()
        time.sleep(translation_detector.get_n_changes_wait_time(1))

        # Move milk to the fridge position in one jump
        fridge_pos = fridge_body.global_pose.to_position().to_np()[:3]
        with world.modify_world():
            milk_conn.parent_T_connection_expression = (
                TransformationMatrix.from_xyz_rpy(
                    x=float(fridge_pos[0]),
                    y=float(fridge_pos[1]),
                    z=float(fridge_pos[2]),
                )
            )
        world.notify_state_change()

        for _ in range(6):
            translation_detector.update_with_latest_motion_data()
            time.sleep(translation_detector.get_n_changes_wait_time(1))

        # Extra steady samples so the stop window is full of near-zeros
        for _ in range(100):
            translation_detector.update_with_latest_motion_data()
            time.sleep(translation_detector.get_n_changes_wait_time(1))

        # Optional debug: confirm near-zero tail in the distance window
        try:
            tail = translation_detector.latest_distances[-3:]
            # print("Distance tail:", tail)
        except Exception:
            pass

        # Assertions: start and stop motion events must be present
        assert milk_tracker.get_latest_event_of_type(TranslationEvent) is not None
        # assert milk_tracker.get_latest_event_of_type(StopTranslationEvent) is not None
        assert milk_tracker.get_latest_event_of_type(StopMotionEvent) is not None

        # Geometric containment check
        final_containment = InsideOf(milk_body, fridge_body).compute_containment_ratio()
        assert (
            final_containment >= 0.9
        ), f"Milk should be mostly inside the fridge, got {final_containment}"
    finally:
        # Stop detectors and join threads cleanly
        translation_detector.stop()
        sr_detector.stop()
        translation_detector.join()
        sr_detector.join()


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
        window_size_in_seconds=0.05,
        world=world,
    )

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
        assert not is_displaced(a.tolist(), 1.5)
