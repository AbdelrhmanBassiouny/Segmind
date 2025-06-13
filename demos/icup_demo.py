import datetime
import logging
import os
import shutil
import threading
from os.path import dirname
from pathlib import Path

from segmind.datastructures.events import AbstractAgentObjectInteractionEvent, PlacingEvent, PickUpEvent, InsertionEvent
from segmind.detectors.coarse_event_detectors import GeneralPickUpDetector, select_transportable_objects
from segmind.detectors.spatial_relation_detector import InsertionDetector
from segmind.episode_segmenter import NoAgentEpisodeSegmenter
from segmind.players.multiverse_player import MultiversePlayer
from segmind.utils import get_arm_and_grasp_description_for_object
from typing_extensions import Dict

import pycram
from pycram.datastructures.enums import WorldMode, Arms
from pycram.datastructures.pose import PoseStamped, Pose, Vector3
from pycram.datastructures.world import World
from pycram.designators.action_designator import PickUpActionDescription, PlaceActionDescription
from pycram.language import SequentialPlan
from pycram.process_module import real_robot
from pycram.robot_description import RobotDescriptionManager
from pycram.ros import logerr
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld
from pycrap.ontologies import Robot, Location, PhysicalObject


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

while True:
    user_input = input("Continue? (y/n) ")
    if user_input == "n":
        break

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

    episode_segmenter.stop()
    thread.join()
    logerr("Joined Thread.")

    all_events = episode_segmenter.logger.get_events()
    actionable_events = [event for event in all_events if isinstance(event, AbstractAgentObjectInteractionEvent)]
    actionable_events = sorted(actionable_events, key=lambda event: event.timestamp)
    pickable_objects = select_transportable_objects(World.current_world.objects)
    action_descriptions = []
    all_inserted_objects = [event.tracked_object for event in actionable_events if isinstance(event, InsertionEvent)]
    pickable_objects = [obj for obj in pickable_objects if obj not in all_inserted_objects]
    object_pick_up_actions: Dict[Object, PickUpActionDescription] = {}
    object_picked_arm: Dict[Object, Arms] = {}
    mapped_objects: Dict[Object, Object] = {}
    # logerr(str(actionable_events))

    for i, actionable_event in enumerate(actionable_events):
        action_descriptions.append(actionable_event.action_description)

        if isinstance(actionable_event, PickUpEvent):
            # logerr("Constructing pickup action")
            if actionable_event.tracked_object not in pickable_objects:
                if len(pickable_objects) > 0:
                    to_pick_object = pickable_objects[0]
                    pickable_objects.remove(pickable_objects[0])
                else:
                    raise ValueError("No pickup objects detected")
            else:
                to_pick_object = actionable_event.tracked_object
                pickable_objects.remove(actionable_event.tracked_object)

            mapped_objects[actionable_event.tracked_object] = to_pick_object
            arm, grasp = get_arm_and_grasp_description_for_object(to_pick_object)
            object_picked_arm[to_pick_object] = arm
            action_descriptions[-1] = PickUpActionDescription(to_pick_object, arm=arm, grasp_description=grasp)
            # logerr("Finished pickup action")
        elif isinstance(actionable_event, PlacingEvent):
            # logerr("Constructing placing action")
            if actionable_event.tracked_object in mapped_objects:
                object_to_place = mapped_objects[actionable_event.tracked_object]
            else:
                raise ValueError("Placing a not picked object")
            place_pose = actionable_event.tracked_object.pose
            place_pose.position.z += 0.05
            action_descriptions[-1] = PlaceActionDescription(object_to_place, target_location=place_pose,
                                                             arm=object_picked_arm[object_to_place])
            # logerr("Finished placing action")
            # action_descriptions.append(ParkArmsActionDescription(Arms.BOTH))

        # print(next(action_descriptions[-1].__iter__()))

    with real_robot:
        plan = SequentialPlan(*action_descriptions)
        print(plan)
        plan.plot()
        plan.perform()
