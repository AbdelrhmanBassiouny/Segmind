from datetime import datetime

import pandas as pd
from typing_extensions import Dict, Set

from pycram.datastructures.pose import Pose, PoseStamped, Vector3, Quaternion, Header
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import PhysicalObject
from .data_player import FilePlayer, FrameData


class CSVEpisodePlayer(FilePlayer):
    data_frames: pd.DataFrame
    data_object_names: Set[str]

    def get_frame_data_generator(self):
        self.data_frames = pd.read_csv(self.file_path)
        self.data_object_names = {v.split(':')[0] for v in self.data_frames.columns if ':' in v}
        for i, (frame_id, objects_data) in enumerate(self.data_frames.iterrows()):
            yield FrameData(time=float(objects_data["time"]), objects_data=objects_data.to_dict(), frame_idx=i)

    def get_joint_states(self, frame_data: FrameData) -> Dict[str, float]:
        return {}

    def _pause(self):
        ...

    def _resume(self):
        ...

    def get_objects_poses(self, frame_data: FrameData) -> Dict[Object, Pose]:
        objects_poses: Dict[Object, Pose] = {}
        objects_data = frame_data.objects_data
        current_time = frame_data.time
        for obj_name in self.data_object_names:
            obj_position = [objects_data[f"{obj_name}:position_{i}"] for i in range(3)]
            obj_orientation = [objects_data[f"{obj_name}:quaternion_{i}"] for i in range(4)]
            obj_orientation[0], obj_orientation[3] = obj_orientation[3], obj_orientation[0]
            obj_pose = PoseStamped(Pose(Vector3(*obj_position), Quaternion(*obj_orientation)),
                                   Header(stamp=datetime.fromtimestamp(current_time)))
            if self.position_shift is not None:
                obj_pose.position += self.position_shift
            obj_type = PhysicalObject

            # Create the object if it does not exist in the world and set its pose
            if obj_name not in self.world.get_object_names():
                obj = Object(obj_name, obj_type, pose=obj_pose, path=f"{obj_name}.urdf")
            else:
                obj = self.world.get_object_by_name(obj_name)
                objects_poses[obj] = obj_pose
        return objects_poses
