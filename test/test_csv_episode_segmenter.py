import datetime
import os
import shutil
from os.path import dirname
from pathlib import Path
from unittest import TestCase
import logging

logging.basicConfig(level=logging.INFO)
logdebug = logging.debug
loginfo = logging.info

# from pycram.datastructures.enums import WorldMode

from pycram.robot_description import RobotDescriptionManager


from segmind.datastructures.events import ContainmentEvent
from segmind.detectors.coarse_event_detectors import (
    GeneralPickUpDetector,
    PlacingDetector,
)
from segmind.detectors.spatial_relation_detector import (
    InsertionDetector,
    SupportDetector,
    ContainmentDetector,
)
from segmind.episode_segmenter import NoAgentEpisodeSegmenter
from segmind.players.csv_player import CSVEpisodePlayer

# from segmind.orm.ormatic_interface import *
try:
    from pycram.worlds.multiverse2 import Multiverse
except ImportError:
    Multiverse = None

from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
from semantic_digital_twin.world_description.world_entity import (
    Body,
    Region,
)
from semantic_digital_twin.spatial_types import TransformationMatrix, Vector3
from semantic_digital_twin.world import World
from semantic_digital_twin.robots.pr2 import PR2


class TestMultiverseEpisodeSegmenter(TestCase):
    world: World
    file_player: CSVEpisodePlayer
    episode_segmenter: NoAgentEpisodeSegmenter
    viz_marker_publisher: VizMarkerPublisher

    @classmethod
    def setUpClass(cls):
        multiverse_episodes_dir = (
            f"{dirname(__file__)}/../resources/multiverse_episodes"
        )
        selected_episode = "icub_montessori_no_hands"
        episode_dir = os.path.join(multiverse_episodes_dir, selected_episode)
        csv_file = os.path.join(episode_dir, f"data.csv")
        models_dir = os.path.join(episode_dir, "models")
        scene_file_path = os.path.join(models_dir, f"scene.xml")

        rdm = RobotDescriptionManager()
        rdm.load_description("iCub")

        cls.world: World = World()

        cls.spawn_objects(models_dir)

        logging.getLogger().setLevel(logging.DEBUG)

        cls.viz_marker_publisher = VizMarkerPublisher()
        cls.file_player = CSVEpisodePlayer(
            csv_file,
            world=cls.world,
            time_between_frames=datetime.timedelta(milliseconds=4),
            position_shift=Vector3(0, 0, -0.05),  # stays identical
        )

        cls.episode_segmenter = NoAgentEpisodeSegmenter(
            cls.file_player,
            annotate_events=True,
            plot_timeline=True,
            plot_save_path=f"{dirname(__file__)}/test_results/{Path(dirname(csv_file)).stem}",
            detectors_to_start=[GeneralPickUpDetector, PlacingDetector],
            initial_detectors=[InsertionDetector, SupportDetector, ContainmentDetector],
        )

    @classmethod
    def spawn_objects(cls, models_dir):
        cls.copy_model_files_to_world_data_dir(models_dir)

        directory = Path(models_dir)
        urdf_files = [f.name for f in directory.glob("*.urdf")]

        for file in urdf_files:
            obj_name = Path(file).stem

            # Default identity transform
            pose = TransformationMatrix.identity()

            if obj_name == "PR2":
                file = "PR2.urdf"
                obj_type = PR2
                pose = TransformationMatrix.from_xyz_quaternion(
                    pos_x=-0.8,
                    pos_y=0,
                    pos_z=0.55,
                    quat_x=0,
                    quat_y=0,
                    quat_z=0,
                    quat_w=1,
                )
            elif obj_name == "scene":
                obj_type = Region
            else:
                obj_type = Body

            try:
                obj = Body(obj_name, obj_type, path=file, pose=pose)
            except Exception as e:
                import pdb

                pdb.set_trace()
                print(e)
                continue

    @classmethod
    def copy_model_files_to_world_data_dir(cls, models_dir):
        print(cls.world)
        print(dir(cls.world))
        shutil.copytree(
            models_dir, cls.world.conf.cache_dir + "/objects", dirs_exist_ok=True
        )

    @classmethod
    def tearDownClass(cls):
        cls.viz_marker_publisher.stop()
        logdebug("Viz marker publisher has been stopped, exiting the world...")
        logdebug("World has been exited.")

    def tearDown(self):
        self.episode_segmenter.reset()
        self.file_player.reset()
        logdebug("File player and episode segmenter have been reset.")

    def test_containment_detector(self):
        self.episode_segmenter.reset()
        self.episode_segmenter.detectors_to_start = [PlacingDetector]
        self.episode_segmenter.initial_detectors = [
            ContainmentDetector,
            SupportDetector,
        ]
        self.episode_segmenter.start()
        self.assertTrue(
            any(
                [
                    isinstance(e, ContainmentEvent)
                    for e in self.episode_segmenter.logger.get_events()
                ]
            )
        )

    def test_csv_replay(self):
        self.episode_segmenter.start()
