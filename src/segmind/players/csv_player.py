import shutil
import time
from datetime import timedelta
from os import path
from os.path import abspath, dirname

import pandas as pd
from typing_extensions import Optional, Dict

from pycram.datastructures.pose import Pose, PoseStamped
from pycram.datastructures.world import World
from pycram.world_concepts.world_object import Object
from segmind.episode_player import EpisodePlayer


class CSVEpisodePlayer(EpisodePlayer):
    def __init__(self, csv_file: str, world: Optional[World] = None,
                 time_between_frames: Optional[timedelta] = None):
        """
        Initializes the FAMEEpisodePlayer with the specified json file and scene id.

        :param csv_file: The json file that contains the data frames.
        :param world: The world that is used to replay the episode.
        """
        super().__init__(time_between_frames=time_between_frames)
        self.csv_file = csv_file
        self.data_frames = pd.read_csv(csv_file)
        self.world = world if world is not None else World.current_world
        self.copy_model_files_to_world_data_dir()
        self._pause: bool = False

    def copy_model_files_to_world_data_dir(self):
        """
        Copy the model files to the world data directory.
        """
        parent_dir_of_json_file = abspath(dirname(self.csv_file))
        models_path = path.join(parent_dir_of_json_file, "models")
        # Copy the entire folder and its contents
        shutil.copytree(models_path, self.world.conf.cache_dir + "/objects", dirs_exist_ok=True)

    def run(self):
        for frame_id, objects_data in self.data_frames.items():
            self._wait_if_paused()
            last_processing_time = time.time()
            self.process_objects_data(objects_data)
            self._wait_to_maintain_frame_rate(last_processing_time)
            self._ready = True

    def process_objects_data(self, objects_data: dict):
        objects_poses: Dict[Object, Pose] = {}
        for object_id, object_poses_data in objects_data.items():

            # Get the object pose in the map frame
            pose = self.get_pose_and_transform_to_map_frame(object_poses_data[0])

            # Get the object and mesh names
            obj_name = self.get_object_name(int(object_id))
            obj_type = self.get_object_type(int(object_id))
            mesh_name = self.get_mesh_name(object_id)

            # Create the object if it does not exist in the world and set its pose
            if obj_name not in self.world.get_object_names():
                obj = Object(obj_name, obj_type, mesh_name, pose=PoseStamped(Pose(pose.position.to_list())))
            else:
                obj = self.world.get_object_by_name(obj_name)
                objects_poses[obj] = pose
        if len(objects_poses):
            self.world.reset_multiple_objects_base_poses(objects_poses)