import json
import os
import shutil
import threading
import time

import numpy as np
from tf.transformations import quaternion_from_matrix, euler_matrix, euler_from_matrix
from typing_extensions import Optional, List, Type

from pycram.datastructures.world import World
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.datastructures.pose import Pose, Transform
from pycram.world_concepts.world_object import Object
from pycram.worlds.multiverse import Multiverse
from pycram.worlds.bullet_world import BulletWorld
from .episode_segmenter import EpisodeSegmenter


class FAMEEpisodeSegmenter(EpisodeSegmenter):
    """
    The NEEMSegmenter class is used to segment the NEEMs motion replay data by using event detectors, such as contact,
    loss of contact, and pick up events.
    """

    def __init__(self, json_file: str,  annotate_events: bool = False):
        """
        Initializes the NEEMSegmenter class.

        :param json_file: The json file that contains the data frames.
        :param annotate_events: The boolean value that indicates whether the events should be annotated.
        """
        self.neem_player_thread = FAMEEpisodePlayer(json_file)
        super().__init__(self.neem_player_thread, annotate_events)

    def run_event_detectors_on_neem(self, sql_neem_ids: Optional[List[int]] = None) -> None:
        """
        Runs the event detectors on the NEEMs motion replay data.
        :param sql_neem_ids: An optional list of integer values that represent the SQL NEEM IDs.
        """
        if sql_neem_ids is None:
            sql_neem_ids = [17]

        self.query_neems_motion_replay_data_and_start_neem_player(sql_neem_ids)

        self.run_event_detectors(self.neem_player_thread)

    def query_neems_motion_replay_data_and_start_neem_player(self, sql_neem_ids: List[int]) -> None:
        """
        Queries the NEEMs motion replay data, starts the NEEM player thread, and waits until the NEEM player thread is
        ready (i.e., the replay environment is initialized with all objects in starting poses).
        :param sql_neem_ids: A list of integer values that represent the SQL NEEM IDs.
        """
        self.neem_player_thread.query_neems_motion_replay_data(sql_neem_ids)
        self.neem_player_thread.start()
        while not self.neem_player_thread.ready:
            time.sleep(0.1)


class FAMEEpisodePlayer(threading.Thread):
    def __init__(self, json_file: str, scene_id: int = 1, world: Type[World] = BulletWorld):
        """
        Initializes the FAMEEpisodePlayer with the specified json file and scene id.

        :param json_file: The json file that contains the data frames.
        :param scene_id: The scene id.
        :param world: The world that is used to replay the episode.
        """
        super().__init__()
        self.json_file = json_file
        with open(self.json_file, 'r') as f:
            self.data_frames = json.load(f)[str(scene_id)]
        self.data_frames = {int(k): v for k, v in self.data_frames.items()}
        self.data_frames = dict(sorted(self.data_frames.items(), key=lambda x: x[0]))
        self.world = world(WorldMode.GUI)
        self.copy_model_files_to_world_data_dir()
        self._ready = False

    def copy_model_files_to_world_data_dir(self):
        parent_dir_of_json_file = os.path.abspath(os.path.dirname(self.json_file))
        models_path = os.path.join(parent_dir_of_json_file, "custom", "models")
        # Copy the entire folder and its contents
        shutil.copytree(models_path, self.world.cache_manager.data_directories[0], dirs_exist_ok=True)

    @property
    def ready(self):
        return self._ready

    def run(self):
        self.replay_episode()

    def replay_episode(self):
        for frame_id, objects_data in self.data_frames.items():
            self.process_objects_data(objects_data)
            time.sleep(0.1)

            self._ready = True

    def process_objects_data(self, objects_data: dict):
        for object_id, object_poses in objects_data.items():

            # Process the object poses
            object_pose = object_poses[0]
            position = np.array(list(map(float, object_pose['t']))) / 1000  # Convert from mm to m
            position = position.tolist()
            rotation = np.array(list(map(float, object_pose['R']))).reshape(3, 3)
            homogeneous_matrix = np.eye(4)
            homogeneous_matrix[:3, :3] = rotation
            quaternion = quaternion_from_matrix(homogeneous_matrix).tolist()
            new_transform = Transform([0, 0, 1],
                                      quaternion_from_matrix(euler_matrix(-np.pi/2, 0, 0)).tolist(),
                                      child_frame="fame_episode_camera_frame")
            self.world.local_transformer.update_transforms([new_transform])
            pose = Pose(position, quaternion, "fame_episode_camera_frame")
            pose = self.world.local_transformer.transform_pose(pose, "map")

            # Get the object and mesh names
            obj_name = f"fame_episode_object_{object_id}"
            mesh_name = self.get_mesh_name(object_id)

            # Create the object if it does not exist in the world and set its pose
            if obj_name not in self.world.get_object_names():
                obj = Object(obj_name, ObjectType.GENERIC_OBJECT, mesh_name,
                             pose=pose, scale_mesh=0.001)
            else:
                obj = self.world.get_object_by_name(f"fame_episode_object_{object_id}")
                obj.set_pose(pose)

    @staticmethod
    def get_mesh_name(object_id: str):
        return f"obj_00000{object_id}.ply"
