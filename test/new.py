#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import TransformationMatrix
from semantic_digital_twin.world_description.world_entity import Body, SemanticAnnotation
from semantic_digital_twin.world_description.geometry import Box, Cylinder, Scale, Color
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world import World

# --- Semantic Annotations ---
class WindmillBase(SemanticAnnotation):
    def __init__(self, body: Body):
        self.body = body

class WindmillTower(SemanticAnnotation):
    def __init__(self, body: Body):
        self.body = body

class WindmillBlade(SemanticAnnotation):
    def __init__(self, body: Body):
        self.body = body

# --- ROS 2 Node ---
class WindmillPublisher(Node):
    def __init__(self):
        super().__init__("sem_dt_windmill_rviz")
        print("✅ Running updated WindmillPublisher — blades fixed on hub")
        self.pub = self.create_publisher(MarkerArray, "windmill_markers", 10)
        self.timer = self.create_timer(0.1, self.publish_windmill)

        # --- Create SemDT world ---
        self.world = World()
        self.root = Body(name=PrefixedName("root", "world"))

        with self.world.modify_world():
            self.world.add_body(self.root)

        # --- Base ---
        base_shape = Box(scale=Scale(0.5, 0.5, 0.05), color=Color(0.6, 0.6, 0.6, 1.0))
        self.base_body = Body(
            name=PrefixedName("windmill_base", "windmill"),
            visual=ShapeCollection([base_shape]),
            collision=ShapeCollection([base_shape])
        )
        _ = WindmillBase(self.base_body)

        # --- Tower ---
        tower_shape = Cylinder(width=0.05, height=1.0, color=Color(0.8, 0.8, 0.8, 1.0))
        self.tower_body = Body(
            name=PrefixedName("windmill_tower", "windmill"),
            visual=ShapeCollection([tower_shape]),
            collision=ShapeCollection([tower_shape])
        )
        _ = WindmillTower(self.tower_body)

        # --- Hub (top of tower) ---
        hub_shape = Box(scale=Scale(0.1, 0.05, 0.02), color=Color(0.3, 0.3, 0.3, 1.0))
        self.hub_body = Body(
            name=PrefixedName("windmill_hub", "windmill"),
            visual=ShapeCollection([hub_shape]),
            collision=ShapeCollection([hub_shape])
        )

        # --- Blades ---
        self.blades = []
        num_blades = 3
        blade_length = 0.6
        blade_width = 0.05
        blade_height = 0.02
        for i in range(num_blades):
            angle = i * 2 * math.pi / num_blades
            blade_shape = Box(scale=Scale(blade_length, blade_width, blade_height),
                              color=Color(1.0, 0.0, 0.0, 1.0))
            blade_body = Body(
                name=PrefixedName(f"blade_{i}", "windmill"),
                visual=ShapeCollection([blade_shape]),
                collision=ShapeCollection([blade_shape])
            )
            self.blades.append((blade_body, angle))
            _ = WindmillBlade(blade_body)

        # --- Connections ---
        with self.world.modify_world():
            # Base -> root
            c_base = Connection6DoF.create_with_dofs(parent=self.root, child=self.base_body, world=self.world)
            self.world.add_connection(c_base)
            c_base.origin = TransformationMatrix.from_xyz_rpy(z=0.025, reference_frame=self.root)

            # Tower -> base
            c_tower = Connection6DoF.create_with_dofs(parent=self.base_body, child=self.tower_body, world=self.world)
            self.world.add_connection(c_tower)
            c_tower.origin = TransformationMatrix.from_xyz_rpy(z=0.525, reference_frame=self.base_body)

            # Hub -> top of tower (keep extension exactly as before)
            c_hub = Connection6DoF.create_with_dofs(parent=self.tower_body, child=self.hub_body, world=self.world)
            self.world.add_connection(c_hub)
            c_hub.origin = TransformationMatrix.from_xyz_rpy(x=0.05, z=1.0, reference_frame=self.tower_body)

            # Blades -> hub (move to hub, expand and face horizontal forward)
            for i, (blade_body, angle) in enumerate(self.blades):
                c_blade = Connection6DoF.create_with_dofs(parent=self.hub_body, child=blade_body, world=self.world)
                self.world.add_connection(c_blade)
                # Attach at left edge of hub and rotate horizontally forward
                c_blade.origin = TransformationMatrix.from_xyz_rpy(
                    x=0.05, y=0.0, z=0.0,      # start at hub's edge
                    pitch=math.radians(90),     # horizontal forward
                    yaw=angle,                  # spread evenly
                    reference_frame=self.hub_body
                )

        # --- Precompute MarkerArray ---
        self.marker_array = MarkerArray()
        self.id_counter = 0
        for b in [self.base_body, self.tower_body, self.hub_body] + [b for b, _ in self.blades]:
            markers = self.body_to_markers(b, self.id_counter)
            self.marker_array.markers.extend(markers)
            self.id_counter += len(markers)

    # --- Helper to convert Body to ROS Markers ---
    def body_to_markers(self, body, id_start=0, frame="world"):
        markers = []
        for i, shape in enumerate(body.visual.shapes):
            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"{body.name.prefix}_{body.name.name}"
            marker.id = id_start + i
            marker.type = Marker.CUBE if isinstance(shape, Box) else Marker.CYLINDER
            marker.action = Marker.ADD

            # Get connection for this body
            connection = next((c for c in self.world.connections if c.child is body), None)
            if connection is not None:
                world_T_body = connection.origin
            else:
                world_T_body = shape.origin  # fallback

            pos = world_T_body.to_translation()
            tx, ty, tz = float(pos.x.to_np()), float(pos.y.to_np()), float(pos.z.to_np())
            marker.pose.position.x = tx
            marker.pose.position.y = ty
            marker.pose.position.z = tz

            quat = world_T_body.to_quaternion()
            marker.pose.orientation.x = float(quat.x.to_np())
            marker.pose.orientation.y = float(quat.y.to_np())
            marker.pose.orientation.z = float(quat.z.to_np())
            marker.pose.orientation.w = float(quat.w.to_np())

            # Set scale from shape
            if isinstance(shape, Box):
                marker.scale.x = float(shape.scale.x)
                marker.scale.y = float(shape.scale.y)
                marker.scale.z = float(shape.scale.z)
            elif isinstance(shape, Cylinder):
                marker.scale.x = float(shape.width)
                marker.scale.y = float(shape.width)
                marker.scale.z = float(shape.height)

            # Set color
            c = ColorRGBA()
            c.r = float(shape.color.R)
            c.g = float(shape.color.G)
            c.b = float(shape.color.B)
            c.a = float(shape.color.A)
            marker.color = c

            markers.append(marker)
        return markers

    def publish_windmill(self):
        self.pub.publish(self.marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = WindmillPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
