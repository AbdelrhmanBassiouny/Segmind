import logging
import os
import threading
import time
from datetime import timedelta
from queue import Queue

import casadi as ca

import numpy as np
import pytest
import rclpy
from krrood.ormatic.dao import to_dao
from krrood.ormatic.utils import create_engine
from pyglet import window
from sqlalchemy.orm import Session
from typing_extensions import List

from conftest import apartment_world
from segmind.datastructures.events import (
    TranslationEvent,
    StopMotionEvent,
    StopTranslationEvent,
    ContactEvent,
    ContainmentEvent,
    SupportEvent,
    LossOfContactEvent,
    LossOfSupportEvent,
    PickUpEvent,
    EventUnion,
)
from segmind.datastructures.object_tracker import ObjectTrackerFactory
from segmind.detectors.atomic_event_detectors import (
    TranslationDetector,
    ContactDetector,
    LossOfContactDetector,
)
from segmind.detectors.coarse_event_detectors import (
    GeneralPickUpDetector,
    PlacingDetector,
)
from segmind.detectors.spatial_relation_detector import (
    InsertionDetector,
    ContainmentDetector,
    SupportDetector,
)
from segmind.detectors.motion_detection_helpers import (
    has_consistent_direction,
    is_displaced,
)
from segmind.event_logger import EventLogger
from segmind.utils import get_support
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
from semantic_digital_twin.collision_checking.trimesh_collision_detector import (
    TrimeshCollisionDetector,
)
from semantic_digital_twin.orm.ormatic_interface import BodyDAO, Base, WorldMappingDAO
from semantic_digital_twin.reasoning.predicates import InsideOf
from semantic_digital_twin.reasoning.world_reasoner import WorldReasoner
from semantic_digital_twin.robots.abstract_robot import AbstractRobot
from semantic_digital_twin.semantic_annotations.semantic_annotations import Fridge

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
from segmind import set_logger_level, LogLevel, logger

set_logger_level(LogLevel.DEBUG)


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
        robot_body.collision.reference_frame = robot_body

        robot_conn = FixedConnection(
            parent=root,
            child=robot_body,
            parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(
                x=3.60, y=0, z=0.35
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
        table_body.collision.reference_frame = table_body

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
        milk_body.collision.reference_frame = milk_body

        milk_conn = Connection6DoF.create_with_dofs(
            world=world,
            parent=table_body,
            child=milk_body,
            parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(
                x=0, y=0, z=0.40
            ),
        )
        world.add_connection(milk_conn)

        target_position = robot_conn.parent_T_connection_expression

    return world, milk_conn, robot_conn, table_conn, target_position


@pytest.fixture
def kitchen_with_milk(kitchen_world):

    world = kitchen_world

    with world.modify_world():

        # Milk (6DoF Connection)
        milk_shape = Box(scale=Scale(0.1, 0.1, 0.2), color=Color(1.0, 1.0, 1.0))
        milk_body = Body(
            name=PrefixedName("milk"),
            visual=ShapeCollection([milk_shape]),
            collision=ShapeCollection([milk_shape]),
        )
        milk_conn = Connection6DoF.create_with_dofs(
            world=world,
            parent=world.root,
            child=milk_body,
            parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(
                -1.02, 1.61, 0.864 + milk_body.combined_mesh.bounding_box.extents[2] / 2
            ),
        )
        world.add_connection(milk_conn)

        wr = WorldReasoner(world=world)
        wr.reason()
        fridge = world.get_semantic_annotations_by_type(Fridge)[0]

        # Target position (Point3)
        target_position = Point3.from_iterable([1.56, -0.972, 0.535])

    return world, milk_conn, fridge, target_position


@pytest.fixture
def visualize_kitchen_world(kitchen_with_milk):
    world, _, _, _ = kitchen_with_milk
    rclpy.init()
    node = rclpy.create_node("demo_node")
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    VizMarkerPublisher(world=world, node=node)
    # # wait for key press
    # input("Press Enter to continue...")
    # rclpy.shutdown()
    return kitchen_with_milk


@pytest.fixture
def visualize_abstract_world(create_kitchen_world_with_milk_and_robot):
    world, milk_conn, robot_conn, table_conn, target_position = (
        create_kitchen_world_with_milk_and_robot
    )
    rclpy.init()
    node = rclpy.create_node("demo2_node")
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    VizMarkerPublisher(world=world, node=node)
    # # wait for key press
    # input("Press Enter to continue...")
    # rclpy.shutdown()
    return create_kitchen_world_with_milk_and_robot


def test_pick_place_and_support_detectors(visualize_abstract_world):
    world, milk_conn, robot_conn, table_conn, target_position = visualize_abstract_world

    # Bodies
    robot_body = world.get_body_by_name("robot")
    milk_body = world.get_body_by_name("milk")
    table_body = world.get_body_by_name("table")

    # Real Event Logger

    event_logger = EventLogger()

    # Real Support Detector
    support_detector = SupportDetector(world=world, event_logger=event_logger)
    support_detector.start()

    # 1. Milk on table----------------------

    # Check support directly
    support = get_support(milk_body, contact_bodies=[table_body])
    assert support == table_body, "get_support should detect table as support"

    event_logger.log_event(
        StopTranslationEvent(
            tracked_object=milk_body,
            start_pose=TransformationMatrix(),
            current_pose=TransformationMatrix(),
        )
    )

    time.sleep(1)

    # Update detector state
    # support_detector.update_body_state(milk_body, with_bodies=[table_body])
    assert (
        support_detector.bodies_states[milk_body] == table_body
    ), "Milk should be supported by table"

    # Check logged SupportEvent
    events = logger.get_events()
    support_events = [e for e in events if isinstance(e, SupportEvent)]
    assert any(
        e.tracked_object == milk_body and e.with_object == table_body
        for e in support_events
    ), "Support event should be logged for milk on table"

    # 2. Robot picks up milk-------------------

    with world.modify_world():
        milk_conn.parent_T_connection_expression = TransformationMatrix.from_xyz_rpy(
            x=3.6, y=0, z=0.35
        )
    world.notify_state_change()
    time.sleep(2.0)
    tcd = TrimeshCollisionDetector(world)
    pickup_event = ContactEvent(
        contact_points=tcd.check_collision_between_bodies(robot_body, milk_body),
        with_object=robot_body,
        of_object=milk_body,
        latest_contact_points=None,
    )

    assert GeneralPickUpDetector.start_condition_checker(
        pickup_event
    ), "Pick-up should be detected"
    GeneralPickUpDetector().get_object_to_track_from_starter_event(pickup_event)
    GeneralPickUpDetector().get_interaction_event(pickup_event)

    # support_detector.update_body_state(milk_body, with_bodies=[table_body])
    assert (
        support_detector.bodies_states[milk_body] is None
    ), "Milk should no longer be supported after lift"

    # Check logged LossOfSupportEvent
    events = logger.get_events()
    loss_events = [e for e in events if isinstance(e, LossOfSupportEvent)]
    assert any(
        e.tracked_object == milk_body and e.with_object == table_body
        for e in loss_events
    ), "Loss-of-support event should be logged when milk is lifted"

    # 3. Milk placed back on table--------------------------------

    with world.modify_world():
        milk_conn.parent_T_connection_expression = TransformationMatrix.from_xyz_rpy(
            x=0, y=0, z=0.40
        )
    world.notify_state_change()
    time.sleep(2.0)
    place_event = LossOfContactEvent(
        contact_points=tcd.check_collision_between_bodies(robot_body, milk_body),
        with_object=robot_body,
        of_object=milk_body,
        latest_contact_points=None,
    )

    assert PlacingDetector.start_condition_checker(place_event)

    support_detector.update_body_state(milk_body, with_bodies=[table_body])
    assert (
        support_detector.bodies_states[milk_body] == table_body
    ), "Milk should be supported again"

    events = logger.get_events()
    support_events = [e for e in events if isinstance(e, SupportEvent)]
    assert any(
        e.tracked_object == milk_body and e.with_object == table_body
        for e in support_events
    ), "Support event should be logged again after placing milk back"


def test_contact_detector(visualize_abstract_world):
    world, milk_conn, robot_conn, table_conn, target_position = visualize_abstract_world

    milk_body = world.get_body_by_name("milk")
    table_body = world.get_body_by_name("table")
    robot_body = world.get_body_by_name("robot")
    logger = EventLogger()

    contact_detector = ContactDetector(
        event_logger=logger,
        tracked_object=milk_body,
        with_object=table_body,
        world=world,
    )
    contact_detector2 = ContactDetector(
        event_logger=logger,
        tracked_object=milk_body,
        with_object=robot_body,
        max_closeness_distance=0.9,
        world=world,
    )

    loss_detector = LossOfContactDetector(
        event_logger=logger,
        tracked_object=milk_body,
        with_object=table_body,
        world=world,
    )
    loss_detector2 = LossOfContactDetector(
        event_logger=logger,
        tracked_object=milk_body,
        with_object=robot_body,
        max_closeness_distance=0.9,
        world=world,
    )
    # Detect initial contact
    contact_events = contact_detector.detect_events()
    assert any(
        isinstance(e, ContactEvent) and e.with_object == table_body
        for e in contact_events
    ), "Milk-table contact should be detected"

    loss_detector.latest_contact_points = contact_detector.latest_contact_points.copy()
    time.sleep(2)
    with world.modify_world():
        milk_conn.parent_T_connection_expression = TransformationMatrix.from_xyz_rpy(
            x=3.60, y=0, z=0.35
        )
    world.notify_state_change()

    events = loss_detector.detect_events()
    lost_contact = any(
        isinstance(e, LossOfContactEvent) and e.with_object == table_body
        for e in events
    )

    print("Detected events after moving milk:", events)
    print("Lost contact detected:", lost_contact)
    assert (
        lost_contact
    ), "LossOfContactDetector should detect milk-table loss of contact"

    contact_events = contact_detector2.detect_events()
    assert any(
        isinstance(e, ContactEvent) and e.with_object == robot_body
        for e in contact_events
    ), "Milk-robot contact should be detected"

    loss_detector2.latest_contact_points = (
        contact_detector2.latest_contact_points.copy()
    )
    time.sleep(2)
    with world.modify_world():
        milk_conn.parent_T_connection_expression = TransformationMatrix.from_xyz_rpy(
            x=0, y=0, z=0.40
        )
    world.notify_state_change()

    events = loss_detector2.detect_events()
    lost_contact = any(
        isinstance(e, LossOfContactEvent) and e.with_object == robot_body
        for e in events
    )

    print("Detected events after moving milk:", events)
    print("Lost contact detected:", lost_contact)
    assert (
        lost_contact
    ), "LossOfContactDetector should detect milk-robot loss of contact"


def test_contact_detector2(visualize_abstract_world):
    world, milk_conn, robot_conn, table_conn, target_position = visualize_abstract_world

    milk_body = world.get_body_by_name("milk")
    table_body = world.get_body_by_name("table")
    # robot_body = world.get_body_by_name("robot")
    logger = EventLogger()

    contact_detector = ContactDetector(
        event_logger=logger,
        tracked_object=milk_body,
        with_object=table_body,
        world=world,
    )

    contact_detector.detect_events()

    contact_detector.latest_contact_points = []
    contact_detector.latest_interference_points = None

    time.sleep(2.0)

    events = contact_detector.detect_events()
    milk_table_contact = any(
        isinstance(e, ContactEvent)
        and any(
            cp.body_a == milk_body and cp.body_b == table_body
            for cp in e.contact_points
        )
        for e in events
    )
    assert milk_table_contact, "Milk-table contact should be detected"


def test_translation_and_containment_detector(visualize_kitchen_world):

    world, milk_conn, fridge, target_position = visualize_kitchen_world

    milk_body = milk_conn.child
    fridge_body = fridge.container.body

    logger.debug(
        f"fridge_body: {fridge_body} with name {fridge_body.name} and id {fridge_body.id}"
    )

    translation_detector = run_and_get_translation_detector(
        milk_body, time_between_frames=timedelta(seconds=0.01), world=world
    )

    # Track events from the milk object
    milk_tracker = ObjectTrackerFactory.get_tracker(milk_body)
    logger.debug(
        f"Milk tracker: {milk_tracker} with object {milk_body} with id {milk_body.id}"
    )

    sr_detector = ContainmentDetector(
        check_on_events={StopTranslationEvent: None}, world=world
    )
    sr_detector.start()

    try:
        # Initial containment check
        assert InsideOf(milk_body, fridge_body).compute_containment_ratio() == 0.0

        # Establish baseline sample before the move
        for _ in range(translation_detector.window_size + 1):
            translation_detector.update_with_latest_motion_data()
            time.sleep(translation_detector.get_n_changes_wait_time(1))

        # Move the milk into the fridge
        with world.modify_world():
            milk_conn.parent_T_connection_expression = (
                TransformationMatrix.from_xyz_rpy(
                    x=target_position.x,
                    y=target_position.y,
                    z=target_position.z,
                )
            )
        world.notify_state_change()

        assert all(
            current == expected
            for current, expected in zip(
                milk_body.global_pose.to_position().to_np(), target_position.to_np()
            )
        )

        logger.debug(f"Milk event history: {milk_tracker.get_event_history()}")

        for _ in range((translation_detector.window_size + 1) * 2):
            translation_detector.update_with_latest_motion_data()
            time.sleep(translation_detector.get_n_changes_wait_time(2))
            logger.debug(f"latest_distances: {translation_detector.latest_distances}")

        logger.debug(f"Milk event history: {milk_tracker.get_event_history()}")
        # Assertions: start and stop motion events must be present
        assert milk_tracker.get_latest_event_of_type(TranslationEvent) is not None
        # assert milk_tracker.get_latest_event_of_type(StopTranslationEvent) is not None
        assert milk_tracker.get_latest_event_of_type(StopMotionEvent) is not None

        # Wait for spatial-relation detector to process the stop event
        time.sleep(sr_detector.wait_time.total_seconds() * 20)

        # Geometric containment check
        final_containment = InsideOf(milk_body, fridge_body).compute_containment_ratio()
        assert (
            final_containment >= 0.9
        ), f"Milk should be mostly inside the fridge, got {final_containment}"
        assert milk_tracker.get_latest_event_of_type(ContainmentEvent) is not None
        assert (
            "fridge"
            in milk_tracker.get_latest_event_of_type(
                ContainmentEvent
            ).with_object.name.name
        )
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
    event_logger = EventLogger()
    translation_detector = TranslationDetector(
        event_logger,
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
