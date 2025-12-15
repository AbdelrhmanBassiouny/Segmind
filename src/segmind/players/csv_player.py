from datetime import datetime

import pandas as pd
from typing_extensions import Dict, Set

from pycram.datastructures.pose import Header
from segmind.players.data_player import FilePlayer, FrameData
from semantic_digital_twin.spatial_types import TransformationMatrix
from semantic_digital_twin.world_description.connections import FixedConnection

from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.spatial_types.spatial_types import (
    TransformationMatrix,
    Vector3,
    Quaternion,
)


class CSVEpisodePlayer(FilePlayer):
    data_frames: pd.DataFrame
    data_object_names: Set[str]

    def get_frame_data_generator(self):
        self.data_frames = pd.read_csv(self.file_path)
        self.data_object_names = {
            v.split(":")[0] for v in self.data_frames.columns if ":" in v
        }
        for i, (frame_id, objects_data) in enumerate(self.data_frames.iterrows()):
            yield FrameData(
                time=float(objects_data["time"]),
                objects_data=objects_data.to_dict(),
                frame_idx=i,
            )

    def get_joint_states(self, frame_data: FrameData) -> Dict[str, float]:
        return {}

    def _pause(self): ...

    def _resume(self): ...

    def get_objects_poses(
        self, frame_data: FrameData
    ) -> dict[Body, TransformationMatrix] | None:
        objects_poses: Dict[Body, TransformationMatrix] = {}
        objects_data = frame_data.objects_data
        current_time = frame_data.time
        for obj_name in self.data_object_names:
            obj_position = [objects_data[f"{obj_name}:position_{i}"] for i in range(3)]
            # Keep quaternion order as (qx, qy, qz, qw)
            obj_orientation = [
                objects_data[f"{obj_name}:quaternion_{i}"] for i in range(4)
            ]

            obj_pose = TransformationMatrix.from_xyz_quaternion(
                obj_position[0],
                obj_position[1],
                obj_position[2],
                obj_orientation[0],
                obj_orientation[1],
                obj_orientation[2],
                obj_orientation[3],
            )
            if self.position_shift is not None:
                # If position_shift is a Vector3 delta, add it (not overwrite)
                obj_pose = TransformationMatrix.from_xyz_quaternion(
                    obj_position[0] + self.position_shift.x,
                    obj_position[1] + self.position_shift.y,
                    obj_position[2] + self.position_shift.z,
                    obj_orientation[0],
                    obj_orientation[1],
                    obj_orientation[2],
                    obj_orientation[3],
                )

            # Map either by existing Body instance or by name if not present in world
            existing = self.world.get_bodies_by_name(name=obj_name)
            if not existing:
                objects_poses[obj_name] = obj_pose
            else:
                body = existing[0] if isinstance(existing, (list, tuple)) else existing
                objects_poses[body] = obj_pose

        return objects_poses
