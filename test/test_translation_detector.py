# test_translation_detector.py

import math
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import TransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.geometry import Box, Scale, Color
from semantic_digital_twin.world_description.connections import Connection6DoF

from segmind.detectors.atmoic_event_detectors_SDT import TranslationDetector

# --------------------------
# SDT wrapper for tracked object (uses connection origin)
# --------------------------
class SDTTrackedBody:
    def __init__(self, connection: Connection6DoF):
        self.connection = connection
        self.body = connection.child
        self.name = self.body.name
        self.current_state = self.CurrentState(connection)
        self.pose = connection.origin  # expose pose for AED

    class CurrentState:
        def __init__(self, connection: Connection6DoF):
            self.connection = connection

        @property
        def position(self):
            return self.connection.origin.to_translation().to_np().tolist()

        @property
        def orientation(self):
            return self.connection.origin.to_rotation_matrix().to_quaternion().to_np().tolist()

# --------------------------
# Create the SDT world
# --------------------------
world = World()
root = Body(name=PrefixedName(name="root", prefix="world"))

# Geometry (no reference_frame here)
base_plate_shape = Box(scale=Scale(0.6, 0.6, 0.05), color=Color(0.3, 0.3, 0.3, 1.0))
camera_shape = Box(scale=Scale(0.10, 0.06, 0.05), color=Color(0.1, 0.2, 0.8, 1.0))

# Bodies
base_body = Body(
    name=PrefixedName(name="base", prefix="demo"),
    visual=ShapeCollection([base_plate_shape]),
)

camera_body = Body(
    name=PrefixedName(name="camera", prefix="demo"),
    visual=ShapeCollection([camera_shape]),
)

# Connections
world_C_base = Connection6DoF.create_with_dofs(parent=root, child=base_body, world=world)
base_C_camera = Connection6DoF.create_with_dofs(parent=base_body, child=camera_body, world=world)

# Origins
world_T_base = TransformationMatrix.from_xyz_rpy(z=0.025, reference_frame=root)
base_T_camera = TransformationMatrix.from_xyz_rpy(y=0.25, z=0.10, yaw=math.radians(30), reference_frame=base_body)

with world.modify_world():
    world.add_connection(world_C_base)
    world.add_connection(base_C_camera)
    world_C_base.origin = world_T_base
    base_C_camera.origin = base_T_camera

# --------------------------
# Wrap tracked body
# --------------------------
tracked_body = SDTTrackedBody(world_C_base)

# --------------------------
# Initialize TranslationDetector (AED)
# --------------------------
detector = TranslationDetector(
    logger=None,
    tracked_object=tracked_body,
    velocity_threshold=0.001,       # threshold for detecting translation
    window_size_in_seconds=2.0,     # must be >= 2 frames
    frame_rate=30
)

# --------------------------
# Print initial state
# --------------------------
print("Initial position:", tracked_body.current_state.position)
print("Initial orientation:", tracked_body.current_state.orientation)

# --------------------------
# Apply small translations
# --------------------------
translations = [
    (0.05, 0.0, 0.025),
    (0.10, 0.0, 0.025),
    (0.15, 0.05, 0.05)
]

for t in translations:
    new_origin = TransformationMatrix.from_xyz_rpy(x=t[0], y=t[1], z=t[2], reference_frame=root)
    with world.modify_world():
        world_C_base.origin = new_origin
        tracked_body.pose = world_C_base.origin  # update pose for AED

    # update AED detector
    pose, timestamp = detector.get_current_pose_and_time()
    translation_world = pose.to_translation().to_np().flatten()[:3].tolist()
    translation_detected = detector.is_event()  # check if AED detects movement

    print("\n--- Updated Step ---")
    print("Translation applied:", t)
    print("Updated translation (world):", translation_world)
    print("Timestamp:", timestamp)
    print("Translation detected by AED?", translation_detected)
