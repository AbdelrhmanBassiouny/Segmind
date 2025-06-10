from datetime import datetime
from typing import Dict

from typing_extensions import Optional, List

from pycram.datastructures.pose import Pose, Quaternion, Vector3, PoseStamped, Header
from pycram.ros import logwarn, logdebug
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import Floor
from .data_player import DataPlayer, FrameData, FrameDataGenerator
from .utils.multiverse_client import MultiverseMetaData, MultiverseConnector


class MultiversePlayer(DataPlayer):

    def __init__(self, simulation_name: str = "replay", world_name: str = "world", objects_names: Optional[List[str]] = None, **kwargs) -> None:
        super().__init__(**kwargs)
        self.objects_names: Optional[List[str]] = objects_names if objects_names is not None else\
            [obj.root_link.name for obj in self.world.objects if not issubclass(obj.obj_type, Floor)]
        self.multiverse_meta_data = MultiverseMetaData(
            world_name=world_name,
            simulation_name=simulation_name,
            length_unit="m",
            angle_unit="rad",
            mass_unit="kg",
            time_unit="s",
            handedness="rhs",
        )
        self.multiverse_connector = MultiverseConnector(port="1996",  multiverse_meta_data=self.multiverse_meta_data)

        self.multiverse_connector.run()

        self._init_request_meta_data()

    def _init_request_meta_data(self):
        self.multiverse_connector.request_meta_data["send"] = {}
        self.multiverse_connector.request_meta_data["receive"] = {}
        if self.objects_names is None:
            self.multiverse_connector.request_meta_data["receive"][""] = [""]
        else:
            for object_name in self.objects_names:
                self.multiverse_connector.request_meta_data["receive"][object_name] = ["position", "quaternion"]

        self.object_data_dict = {}
        self.multiverse_connector.send_and_receive_meta_data()
        self.response_meta_data = self.multiverse_connector.response_meta_data
        for object_name, object_attributes in self.response_meta_data["receive"].items():
            self.object_data_dict[object_name] = {}
            for attribute_name, attribute_values in object_attributes.items():
                self.object_data_dict[object_name][attribute_name] = attribute_values

    def get_frame_data_generator(self) -> FrameDataGenerator:
        i = -1
        while True:
            i += 1
            sim_time = self.multiverse_connector.sim_time  # The current simulation time
            self.multiverse_connector.send_data = [sim_time]
            self.multiverse_connector.send_and_receive_data()
            receive_data = self.multiverse_connector.receive_data[1:]
            idx = 0
            for object_name, object_attributes in self.response_meta_data["receive"].items():
                self.object_data_dict[object_name] = {}
                for attribute_name, attribute_value in object_attributes.items():
                    self.object_data_dict[object_name][attribute_name] = receive_data[idx:idx + len(attribute_value)]
                    idx += len(attribute_value)
            world_time = self.multiverse_connector.world_time
            yield FrameData(world_time, self.object_data_dict, i)

    def get_objects_poses(self, frame_data: FrameData) -> Dict[Object, PoseStamped]:
        objects_poses: Dict[Object, PoseStamped] = {}
        for object_name, object_attributes in frame_data.objects_data.items():
            obj = self.world.get_object_by_root_link_name(object_name)
            if obj is None:
                logdebug(f"Object {object_name} not found")
                continue
            position = object_attributes["position"]
            quaternion = object_attributes["quaternion"]
            pose = PoseStamped(Pose(Vector3(position[0], position[1], position[2]),
                                    Quaternion(quaternion[1], quaternion[2], quaternion[3], quaternion[0])),
                               header=Header(stamp=datetime.fromtimestamp(frame_data.time)))
            objects_poses[obj] = pose
        return objects_poses

    def _join(self, timeout=None):
        self.multiverse_connector.stop()

    def _pause(self):
        pass

    def _resume(self):
        pass
