import datetime
import os
import shutil
import time
from os.path import dirname
from pathlib import Path

import pytest
from segmind.players.multiverse_player import MultiversePlayer

import pycram
from pycram.datastructures.enums import WorldMode, Arms, Grasp
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import PoseStamped, Pose, Vector3
from pycram.datastructures.world import World
from pycram.designator import ObjectDesignatorDescription
from pycram.designators.action_designator import PickUpActionDescription, ParkArmsActionDescription
from pycram.external_interfaces import giskard
from pycram.failures import ObjectNotGraspedError
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot, real_robot
from pycram.robot_description import RobotDescriptionManager
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld
from pycrap.ontologies import Location, PhysicalObject, Robot


@pytest.fixture(scope="module")
def set_up_demo(episode_name: str = "icub_montessori_no_hands"):
    rdm = RobotDescriptionManager()
    rdm.load_description("iCub3")

    world: BulletWorld = BulletWorld(WorldMode.GUI)
    # viz_marker_publisher = VizMarkerPublisher()
    pycram.ros.set_logger_level(pycram.datastructures.enums.LoggerLevel.DEBUG)

    multiverse_episodes_dir = f"{dirname(__file__)}/../resources/multiverse_episodes"
    episode_dir = os.path.join(multiverse_episodes_dir, episode_name)
    models_dir = os.path.join(episode_dir, "models")

    spawn_objects(models_dir)

    csv_file = os.path.join(episode_dir, f"data.csv")
    multiverse_player = MultiversePlayer(world=world,
                                         time_between_frames=datetime.timedelta(milliseconds=4),
                                         stop_after_ready=False)
    multiverse_player.start()

    while not multiverse_player.ready:
        time.sleep(0.1)

    yield world
    # viz_marker_publisher._stop_publishing()


def spawn_objects(models_dir):
    copy_model_files_to_world_data_dir(models_dir)
    directory = Path(models_dir)
    urdf_files = [f.name for f in directory.glob('*.urdf')]
    for file in urdf_files:
        obj_name = Path(file).stem
        pose = PoseStamped()
        if obj_name == "iCub":
            file = "iCub3.urdf"
            obj_type = Robot
            pose = PoseStamped(Pose(Vector3(-0.8, 0, 0.55)))
        elif obj_name == "scene":
            obj_type = Location
        else:
            obj_type = PhysicalObject
        obj = Object(obj_name, obj_type, path=file, pose=pose)


def copy_model_files_to_world_data_dir(models_dir):
    """
    Copy the model files to the world data directory.
    """
    # Copy the entire folder and its contents
    shutil.copytree(models_dir, World.current_world.conf.cache_dir + "/objects", dirs_exist_ok=True)


def test_icub_pick_up(set_up_demo):
    object_description = ObjectDesignatorDescription(names=["montessori_object_2"])
    grasp_idx = 0
    grasps = [GraspDescription(Grasp.FRONT),
              # GraspDescription(Grasp.LEFT, Grasp.TOP),
              # GraspDescription(Grasp.RIGHT, Grasp.TOP),
              # GraspDescription(Grasp.FRONT, Grasp.TOP),
              # GraspDescription(Grasp.BACK, Grasp.BOTTOM),
              # GraspDescription(Grasp.LEFT, Grasp.BOTTOM),
              # GraspDescription(Grasp.RIGHT, Grasp.BOTTOM),
              # GraspDescription(Grasp.FRONT, Grasp.BOTTOM)
              ]
    giskard.achieve_translation_goal([0.2, 0.25, 1.3], "l_gripper_tool_frame", "root_link")
    # while grasp_idx < len(grasps):
    #     grasp = grasps[grasp_idx]
    #     try:
    #         with real_robot:
    #         # with simulated_robot:
    #             plan = SequentialPlan(#ParkArmsActionDescription(Arms.BOTH),
    #                                   PickUpActionDescription(object_description, [Arms.LEFT],
    #                                                           [grasp]))
    #             plan.perform()
    #         print(grasp)
    #         break
    #     except ObjectNotGraspedError as e:
    #         print(e)
    #         grasp_idx += 1

    plan.plot()
