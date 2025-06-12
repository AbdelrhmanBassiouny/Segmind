import datetime
import os
import shutil
import threading
from os.path import dirname
from pathlib import Path

import pycram

from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import PoseStamped, Pose, Vector3
from pycram.datastructures.world import World
from pycram.robot_description import RobotDescriptionManager
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld
from pycrap.ontologies import Robot, Location, PhysicalObject
from segmind.datastructures.events import AbstractAgentObjectInteractionEvent
from segmind.detectors.coarse_event_detectors import GeneralPickUpDetector
from segmind.detectors.spatial_relation_detector import InsertionDetector
from segmind.episode_segmenter import NoAgentEpisodeSegmenter
from segmind.players.multiverse_player import MultiversePlayer


def spawn_objects(models_dir: str):
    copy_model_files_to_world_data_dir(models_dir)
    directory = Path(models_dir)
    urdf_files = [f.name for f in directory.glob('*.urdf')]
    for file in urdf_files:
        obj_name = Path(file).stem
        pose = PoseStamped()
        if obj_name == "iCub":
            file = "iCub.urdf"
            obj_type = Robot
            pose = PoseStamped(Pose(Vector3(-0.8, 0, 0.55)))
        elif obj_name == "scene":
            obj_type = Location
        else:
            obj_type = PhysicalObject
        obj = Object(obj_name, obj_type, path=file, pose=pose)


def copy_model_files_to_world_data_dir(models_dir: str):
    """
    Copy the model files to the world data directory.
    """
    # Copy the entire folder and its contents
    shutil.copytree(models_dir, World.current_world.conf.cache_dir + "/objects", dirs_exist_ok=True)

rdm = RobotDescriptionManager()
rdm.load_description("iCub3")

world: BulletWorld = BulletWorld(WorldMode.GUI)
# viz_marker_publisher = VizMarkerPublisher()
pycram.ros.set_logger_level(pycram.datastructures.enums.LoggerLevel.ERROR)

multiverse_episodes_dir = f"{dirname(__file__)}/../resources/multiverse_episodes"
episode_name = "icub_montessori_no_hands"
episode_dir = os.path.join(multiverse_episodes_dir, episode_name)
models_dir = os.path.join(episode_dir, "models")

spawn_objects(models_dir)

csv_file = os.path.join(episode_dir, f"data.csv")
multiverse_player = MultiversePlayer(world=world,
                                     time_between_frames=datetime.timedelta(milliseconds=4),
                                     stop_after_ready=False)
episode_segmenter = NoAgentEpisodeSegmenter(multiverse_player, annotate_events=True,
                                                plot_timeline=True,
                                                plot_save_path=f'{dirname(__file__)}/test_results/multiverse_episode',
                                                detectors_to_start=[GeneralPickUpDetector],
                                                initial_detectors=[InsertionDetector])
# Create a thread
thread = threading.Thread(target=episode_segmenter.start)
# Start the thread
thread.start()

input("Press Enter to continue...")

all_events = episode_segmenter.logger.get_events()
actionable_events = [event for event in all_events if isinstance(event, AbstractAgentObjectInteractionEvent)]
actionable_events = sorted(actionable_events, key=lambda event: event.timestamp)
for actionable_event in actionable_events:
    print(next(actionable_event.action_description.__iter__()))

multiverse_player.join()
thread.join()
