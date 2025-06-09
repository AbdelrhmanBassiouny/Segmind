import os
import shutil
import time
from datetime import timedelta, datetime
from os import path
from os.path import abspath, dirname
from pathlib import Path

import pandas as pd
from typing_extensions import Optional, Dict, Set

from pycram.datastructures.dataclasses import MeshVisualShape, BoxVisualShape, CylinderVisualShape, Color
from pycram.datastructures.pose import Pose, PoseStamped, Vector3, Quaternion, Header
from pycram.datastructures.world import World
from pycram.ros import logdebug
from pycram.world_concepts.world_object import Object
from pycram.object_descriptors.generic import ObjectDescription as GenericObjectDescription
from pycrap.ontologies import PhysicalObject, Location, Robot
from ..episode_player import EpisodePlayer
from .data_player import FilePlayer, FrameData


class CSVEpisodePlayer(FilePlayer):
    data_frames: pd.DataFrame
    data_object_names: Set[str]

    def __init__(self, file_path: str, models_dir: Optional[str] = None, world: Optional[World] = None,
                 time_between_frames: Optional[timedelta] = None, use_realtime: bool = False,
                 stop_after_ready: bool = False):
        super().__init__(file_path=file_path, models_dir=models_dir, time_between_frames=time_between_frames, use_realtime=use_realtime, world=world,
                         stop_after_ready=stop_after_ready)

        self._spawn_objects()
        
    def _spawn_objects(self):
        directory = Path(self.models_dir)
        urdf_files = [f.name for f in directory.glob('*.urdf')]
        for file in urdf_files:
            obj_name = Path(file).stem
            pose = PoseStamped()
            if obj_name == "iCub":
                obj_name = "iCub3"
                file = "iCub3.urdf"
                obj_type = Robot
                pose = PoseStamped(Pose(Vector3(-0.8, 0, 0.55)))
            elif obj_name == "scene":
                obj_type = Location
            else:
                obj_type = PhysicalObject
            obj = Object(obj_name, obj_type, path=file, pose=pose)

    def get_frame_data_generator(self):
        self.data_frames = pd.read_csv(self.file_path)
        self.data_object_names = {v.split(':')[0] for v in self.data_frames.columns if ':' in v}
        for i, (frame_id, objects_data) in enumerate(self.data_frames.iterrows()):
            yield FrameData(time=float(objects_data["time"]), objects_data=objects_data.to_dict(), frame_idx=i)

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
            obj_pose = PoseStamped(Pose(Vector3(*obj_position), Quaternion(*obj_orientation)), Header(stamp=datetime.fromtimestamp(current_time)))
            obj_type = PhysicalObject

            # Create the object if it does not exist in the world and set its pose
            if obj_name not in self.world.get_object_names():
                obj = Object(obj_name, obj_type, pose=obj_pose, path=f"{obj_name}.urdf")
            else:
                obj = self.world.get_object_by_name(obj_name)
                objects_poses[obj] = obj_pose
        return objects_poses