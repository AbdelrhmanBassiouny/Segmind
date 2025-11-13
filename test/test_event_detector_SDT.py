import time
from datetime import timedelta
import uuid
from types import SimpleNamespace

# Segmind imports
from segmind.datastructures.events import TranslationEvent, StopMotionEvent, StopTranslationEvent, ContactEvent
from segmind.datastructures.object_tracker import ObjectTrackerFactory
from segmind.detectors.atomic_event_detectors_SDT import TranslationDetector
from segmind.detectors.coarse_event_detectors import GeneralPickUpDetector
from segmind.detectors.spatial_relation_detector import InsertionDetector
from segmind.detectors.motion_detection_helpers import has_consistent_direction, is_displaced
from segmind.event_logger import EventLogger

# SDT imports
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.spatial_types import TransformationMatrix, Point3, Vector3
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_computations.raytracer import RayTracer

def create_simple_world():
    """Create a simple SDT world with a root, a milk body, and a container."""
    world = World()

    with world.modify_world():
        # Root body must be unique
        root = Body(name=PrefixedName("root"))
        world.add_body(root)

        milk = Body(name=PrefixedName("milk"))
        milk.collision = [Box(scale=Scale(0.1, 0.1, 0.1), origin=TransformationMatrix(reference_frame=milk))]
        world.add_body(milk)

        container = Body(name=PrefixedName("container"))
        container.collision = [Box(scale=Scale(0.5, 0.5, 0.5), origin=TransformationMatrix(reference_frame=container))]
        world.add_body(container)

    return world, milk, container

# --- Test Class ---
def attach_dummy_state(body: Body, pos=(0, 0, 0)):
    """Attach a dummy `.state` and `.current_state` with a position."""
    body.state = SimpleNamespace(position=Point3(*pos))
    body.current_state = body.state

def patch_body_for_segmind(body: Body):
    """Patch SDT body to provide Segmind-like interface."""
    attach_dummy_state(body)

    # Provide a .pose property (returns position)
    @property
    def pose(self):
        return self.state.position
    body.pose = pose.__get__(body)

    # Provide dummy get_axis_aligned_bounding_box method
    def get_axis_aligned_bounding_box(self):
        if self.collision and len(self.collision) > 0:
            box = self.collision[0]
            s = getattr(box, "scale", Scale(1,1,1))
            p = getattr(self.state, "position", Point3(0,0,0))
            return SimpleNamespace(center=p, extent=s)
        else:
            return SimpleNamespace(center=Point3(0,0,0), extent=Scale(1,1,1))
    body.get_axis_aligned_bounding_box = get_axis_aligned_bounding_box.__get__(body)


class TestEventDetectors:

    def setup_method(self):
        """Setup a fresh SDT world, one root, patched bodies, and RayTracer."""
        self.world = World()
        with self.world.modify_world():
            # Root (mandatory)
            self.root = Body(name=PrefixedName("root"))
            self.world.add_body(self.root)

            # Other bodies
            self.milk = Body(name=PrefixedName("milk"))
            self.milk.collision = [Box(scale=Scale(0.1, 0.1, 0.1),
                                       origin=TransformationMatrix(reference_frame=self.milk))]
            self.world.add_body(self.milk)

            self.container = Body(name=PrefixedName("container"))
            self.container.collision = [Box(scale=Scale(0.5, 0.5, 0.5),
                                            origin=TransformationMatrix(reference_frame=self.container))]
            self.world.add_body(self.container)

            # Connect all bodies to root to satisfy SDT
            for child in [self.milk, self.container]:
                conn = Connection6DoF.create_with_dofs(parent=self.root, child=child, world=self.world)
                self.world.add_connection(conn)

        # Patch bodies for Segmind compatibility
        for body in [self.root, self.milk, self.container]:
            patch_body_for_segmind(body)

        # Initial positions
        self.root.state.position = Point3(0, 0, 0)
        self.milk.state.position = Point3(0, 0, 0)
        self.container.state.position = Point3(1, 0, 0)

        # Initialize RayTracer
        self.raytracer = RayTracer(self.world)
        self.raytracer.update_scene()

        self.logger = EventLogger()


    def test_general_pick_up_start_condition_checker(self):
        event = ContactEvent(self.milk, self.container, 0.1)
        GeneralPickUpDetector.start_condition_checker(event)


    def test_translation_detector(self):
        milk_tracker = ObjectTrackerFactory.get_tracker(self.milk)
        translation_detector = TranslationDetector(self.logger, self.milk, time_between_frames=timedelta(seconds=0.01))
        translation_detector.start()

        try:
            # Apply a motion large enough to trigger TranslationEvent
            with self.world.modify_world():
                self.milk.state.position = Point3(1.0, 1.0, 0.5)

            translation_detector.update_with_latest_motion_data()
            time.sleep(translation_detector.get_n_changes_wait_time(1))

            # Check that TranslationEvent is detected
            translation_event = milk_tracker.get_latest_event_of_type(TranslationEvent)
            assert translation_event is not None

            # Simulate stopping motion
            translation_detector.update_with_latest_motion_data()
            time.sleep(translation_detector.get_n_changes_wait_time(1))
            assert milk_tracker.get_first_event_of_type_after_event(StopTranslationEvent, translation_event) is not None

        finally:
            translation_detector.stop()
            translation_detector.join()


    def test_insertion_detector(self):
        milk_tracker = ObjectTrackerFactory.get_tracker(self.milk)
        translation_detector = TranslationDetector(self.logger, self.milk, time_between_frames=timedelta(seconds=0.01))
        translation_detector.start()

        sr_detector = InsertionDetector(world=self.world, wait_time=timedelta(seconds=0.01))
        sr_detector.start()

        try:
            # Milk not in container initially
            assert self.milk.state.position != self.container.state.position

            # Move milk to container position
            with self.world.modify_world():
                self.milk.state.position = self.container.state.position

            translation_detector.update_with_latest_motion_data()
            time.sleep(translation_detector.get_n_changes_wait_time(2))

            # Check that StopMotionEvent occurs
            assert milk_tracker.get_latest_event_of_type(StopMotionEvent) is not None

        finally:
            translation_detector.stop()
            sr_detector.stop()
            translation_detector.join()
            sr_detector.join()


    def test_consistent_gradient_motion_detection_method(self):
        for i in range(3):
            a = [[0,0,0],[0,0,0],[0,0,0]]
            for j in range(3): a[j][i] = 1
            assert has_consistent_direction(a)
            a = [[0,0,0],[0,0,0],[0,0,0]]
            for j in range(3): a[j][i] = -1
            assert has_consistent_direction(a)
            a = [[0,0,0],[0,0,0],[0,0,0]]
            for j in range(3): a[j][i] = -1
            a[1][i] = 1
            assert not has_consistent_direction(a)


    def test_displacement_motion_detection_method(self):
        for i in range(3):
            a = [[0,0,0],[0,0,0],[0,0,0]]
            for j in range(3): a[j][i] = 1
            assert is_displaced(a, 1.5)
            a = [[0,0,0],[0,0,0],[0,0,0]]
            for j in range(3): a[j][i] = -1
            assert is_displaced(a, 1.5)
            a = [[0,0,0],[0,0,0],[0,0,0]]
            for j in range(3): a[j][i] = -1
            a[1][i] = 1
            assert not is_displaced(a, 1.5)
