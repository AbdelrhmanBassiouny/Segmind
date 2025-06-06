import os
import shutil
import time
from datetime import timedelta
from os import path
from os.path import abspath, dirname
from pathlib import Path

import pandas as pd
from typing_extensions import Optional, Dict

from pycram.datastructures.dataclasses import MeshVisualShape, BoxVisualShape, CylinderVisualShape, Color
from pycram.datastructures.pose import Pose, PoseStamped, Vector3, Quaternion
from pycram.datastructures.world import World
from pycram.ros import logdebug
from pycram.world_concepts.world_object import Object
from pycram.object_descriptors.generic import ObjectDescription as GenericObjectDescription
from pycrap.ontologies import PhysicalObject, Location, Robot
from segmind.episode_player import EpisodePlayer


class CSVEpisodePlayer(EpisodePlayer):
    def __init__(self, csv_file: str, models_dir: Optional[str] = None, world: Optional[World] = None,
                 time_between_frames: Optional[timedelta] = None):
        """
        Initializes the FAMEEpisodePlayer with the specified json file and scene id.

        :param csv_file: The json file that contains the data frames.
        :param world: The world that is used to replay the episode.
        """
        super().__init__(time_between_frames=time_between_frames)
        self.csv_file = csv_file
        self.models_dir = models_dir
        self.world = world if world is not None else World.current_world
        self.copy_model_files_to_world_data_dir()

        directory = Path(models_dir)
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

        self.data_frames = pd.read_csv(csv_file)
        self.data_object_names = {v.split(':')[0] for v in self.data_frames.columns if ':' in v}

    def copy_model_files_to_world_data_dir(self):
        """
        Copy the model files to the world data directory.
        """
        if self.models_dir is None:
            parent_dir_of_json_file = abspath(dirname(self.csv_file))
            self.models_dir = path.join(parent_dir_of_json_file, "models")
        # Copy the entire folder and its contents
        shutil.copytree(self.models_dir, self.world.conf.cache_dir + "/objects", dirs_exist_ok=True)

    def _run(self):
        for frame_id, objects_data in self.data_frames.iterrows():
            logdebug(f"Processing frame {frame_id}")
            self._wait_if_paused()
            self.process_objects_data(objects_data.to_dict())
            last_processing_time = time.time()
            self._wait_to_maintain_frame_rate(last_processing_time)
            self._ready = True

    def _pause(self):
        ...

    def _resume(self):
        ...

    def process_objects_data(self, objects_data: dict):
        objects_poses: Dict[Object, Pose] = {}
        for obj_name in self.data_object_names:
            obj_position = [objects_data[f"{obj_name}:position_{i}"] for i in range(3)]
            obj_orientation = [objects_data[f"{obj_name}:quaternion_{i}"] for i in range(4)]
            obj_orientation[0], obj_orientation[3] = obj_orientation[3], obj_orientation[0]
            obj_pose = PoseStamped(Pose(Vector3(*obj_position), Quaternion(*obj_orientation)))
            obj_type = PhysicalObject

            # Create the object if it does not exist in the world and set its pose
            if obj_name not in self.world.get_object_names():
                obj = Object(obj_name, obj_type, pose=obj_pose, path=f"{obj_name}.urdf")
            else:
                obj = self.world.get_object_by_name(obj_name)
                objects_poses[obj] = obj_pose
        if len(objects_poses):
            if not self._ready:
                for obj, pose in objects_poses.items():
                    pose.position.z -= 0.1
            self.world.reset_multiple_objects_base_poses(objects_poses)
