from __future__ import annotations
import pybullet as p
import pybullet_data
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, is_dataclass, fields, Field, MISSING
from typing import Optional, Any, Callable
import sys
import builtins


DEFAULT_URDF_PATH = "panda_urdf/panda_arm.urdf"


@dataclass
class MimicInfo:
    joint: Joint
    multiplier: float = 1.0
    offset: float = 0.0


class Joint:
    def __init__(self, robot_id: int, idx: int):
        self.idx = idx
        self.robot_id = robot_id
        self.mimic_list: dict[Joint, MimicInfo] = {}
        self.is_mimic = False
    
    def add_mimic(self, mimic_info: MimicInfo):
        self.mimic_list[mimic_info.joint] = mimic_info
    
    def set_pos(self, val: float):
        p.setJointMotorControl2(self.robot_id, self.idx, p.POSITION_CONTROL, targetPosition=val)
        for mimic_joint, mimic_info in self.mimic_list.items():
            mimic_val = mimic_info.multiplier * val + mimic_info.offset
            mimic_joint.set_pos(mimic_val)


class RobotJoints:
    def __init__(self, urdf_path: str, fixed_base: Optional[bool] = None, base_position: Optional[list] = None):
        self.urdf_path = urdf_path
        self.tree = ET.parse(self.urdf_path)
        self.root = self.tree.getroot()

        if fixed_base is None:
            fixed_base = self._detect_fixed()
        if base_position is None:
            base_position = [0, 0, 0] if fixed_base else [0, 0, 1]

        self.robot_id = p.loadURDF(self.urdf_path, basePosition=base_position, useFixedBase=fixed_base)

        self._find_joints()
        self._find_mimics()
    
    def _detect_fixed(self) -> bool:
        for gazebo in self.root.findall("gazebo"):
            static_elem = gazebo.find("static")
            if static_elem is not None:
                if static_elem.text == "true":
                    return True
        return False

    def _find_joints(self):
        self.index_to_name: dict[int, str] = {}
        self.name_to_index: dict[str, int] = {}
        self.joints: dict[str, Joint] = {}
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            name = joint_info[1].decode("utf-8")
            self.index_to_name[i] = name
            self.name_to_index[name] = i
            self.joints[name] = Joint(self.robot_id, i)
    
    def _find_mimics(self):
        for joint in self.root.findall("joint"):
            mimic_elem = joint.find("mimic")
            if mimic_elem is not None:
                mimicking_joint = self.joints[joint.get("name")]
                mimicked_joint = self.joints[mimic_elem.get("joint")]
                mimicked_joint.add_mimic(
                    MimicInfo(
                        joint=mimicking_joint,
                        multiplier=float(mimic_elem.get("multiplier", 1.0)),
                        offset=float(mimic_elem.get("offset", 0.0))
                    )
                )
                mimicking_joint.is_mimic = True
    
    def add_control_sliders(self):
        self.sliders = {}

        for name, joint in self.joints.items():
            if joint.is_mimic:
                continue

            info = p.getJointInfo(self.robot_id, joint.idx)
            joint_type = info[2]
            if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                lower = info[8] if info[8] < info[9] else -3.14
                upper = info[9] if info[8] < info[9] else 3.14
                self.sliders[name] = p.addUserDebugParameter(name, lower, upper, 0.0)
    
    def update_from_sliders(self):
        for name, slider_id in self.sliders.items():
            joint = self.joints[name]
            val = p.readUserDebugParameter(slider_id)
            joint.set_pos(val)


def start_pybullet():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # Hide sidebar GUI
    p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
    # p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)


def get_global_object(obj: type | str) -> type:
    if isinstance(obj, type):
        return obj
    try:
        return vars(builtins)[obj]
    except KeyError as e:
        try:
            return globals()[obj]
        except KeyError as e2:
            raise KeyError(f"Unknown global var {obj}") from e2


class ArgvParser:
    EQ_SEP = "="
    USAGE = "Usage: --[arg name]=[arg value]"

    def __init__(self, dataclass_object: type):
        dataclass_object = get_global_object(dataclass_object)
        if not is_dataclass(dataclass_object):
            raise ValueError("Argument type must be a dataclass Object")
        self._dataclass_object = dataclass_object
        self._dataclass_fields = fields(dataclass_object)
        self._arg_dict: dict[str] = {}
    
    def _available_args(self) -> list[str]:
        return [field.name for field in self._dataclass_fields]
    
    @classmethod
    def _field_cli_display(cls, field: Field) -> str:
        return "--" + field.name + cls.EQ_SEP
    
    def _match_field(self, cli_arg: str, field: Field) -> bool:
        return cli_arg.startswith(ArgvParser._field_cli_display(field))
    
    @staticmethod
    def _eval_str_field_val(val: str, field: Field) -> Any:
        field_type = get_global_object(field.type)
        if field_type in (int, float, str):
            return field_type(val)
        if field_type is bool:
            val_ = val.lower()
            try:
                return {"true": True, "false": False}[val_]
            except KeyError as e:
                raise ValueError(f"Cannot interpret '{val}' as bool") from e
        print(f"Warning: don't know how to evaluate field type {field_type}")
        return field_type(val)
            
    def _parse_field(self, cli_arg: str, field: Field) -> Any:
        if not self._match_field(cli_arg, field):
            raise ValueError("Field doesn't match")
        str_val = cli_arg[cli_arg.index(ArgvParser.EQ_SEP)+1:]
        return self._eval_str_field_val(str_val, field)
    
    def _parse_arg(self, cli_arg: str):
        for field in self._dataclass_fields:
            if self._match_field(cli_arg, field):
                self._arg_dict[field.name] = self._parse_field(cli_arg, field)
                return
        raise ValueError(f"Unknown cli argument or bad synthax: {cli_arg}\n{ArgvParser.USAGE}\nAvailable arguments: {', '.join(self._available_args())}")

    def _enforce_required_fields(self):
        for field in self._dataclass_fields:
            if field.default is MISSING:
                if field.name not in self._arg_dict:
                    raise ValueError(f"Didn't provide a value for required argument '{field.name}'")

    def parse_argv(self) -> Any:
        self._arg_dict.clear()
        for arg in sys.argv[1:]:
            self._parse_arg(arg)
        self._enforce_required_fields()
        return self._dataclass_object(**self._arg_dict)


def parse_argv(f: Callable[[type]]):
    """
    Assuming the function f takes exactly one argument which must be annotated as a dataclass instance
    """
    annots = {k: v for k, v in f.__annotations__.items() if k != "return"}
    if len(annots) != 1:
        raise ValueError(f"The function {f.__name__} must take exactly one argument which must be annotated as a dataclass instance")
    dataclass_object = list(annots.values())[0]
    parser = ArgvParser(dataclass_object)
    parsed_arg = parser.parse_argv()

    def inner_f() -> Any:
        return f(parsed_arg)
    
    return inner_f


@dataclass
class Settings:
    path: str = DEFAULT_URDF_PATH
    fixed: bool = None


@parse_argv
def visualize(settings: Settings):
    floor_id = p.loadURDF("plane.urdf")
    floor_rgb = [0.831, 0.965, 1.0]
    p.changeVisualShape(floor_id, -1, rgbaColor=[*floor_rgb, 1])
    robot = RobotJoints(urdf_path=settings.path, fixed_base=settings.fixed)
    robot.add_control_sliders()

    while True:
        robot.update_from_sliders()

        p.stepSimulation()
        time.sleep(1.0 / 240.0)


if __name__ == "__main__":
    start_pybullet()
    try:
        visualize()
    except KeyboardInterrupt:
        p.disconnect()
