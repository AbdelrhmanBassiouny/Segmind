import datetime
import json

import numpy as np
from trimesh import Geometry
from typing_extensions import Type, List, Dict, Optional
import pandas as pd

from pycram import World
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import PhysicalObject
from ..episode_player import EpisodePlayer


class CSVEpisodePlayer(EpisodePlayer):
    def __init__(self, csv_file: str, world: Optional[World] = None,
                 time_between_frames: datetime.timedelta = datetime.timedelta(milliseconds=50)):
        """
        Initializes the FAMEEpisodePlayer with the specified json file and scene id.

        :param csv_file: The csv file that contains the data frames.
        :param world: The world that is used to replay the episode.
        :param time_between_frames: The time between frames.
        """
        super().__init__(time_between_frames=time_between_frames)
        self.csv_file = csv_file
        with open(self.csv_file, 'r') as f:
            self.data_frames = pd.read_csv(f, index_col=0).to_dict(orient='index')
        self.data_frames = {int(frame_id): objects_data for frame_id, objects_data in self.data_frames.items()}
        self.data_frames = dict(sorted(self.data_frames.items(), key=lambda x: x[0]))
        self.world = world if world is not None else World.current_world
        self.object_meshes: Dict[Object, Geometry] = {}
        self.correction_quaternions: Dict[Object, np.ndarray] = {}
        self.base_origin_of_objects: Dict[Object, np.ndarray] = {}
        self.average_rotation_correction_matrix: Optional[np.ndarray] = None
        self._pause: bool = False
