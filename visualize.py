from __future__ import annotations
import pybullet as p
import pybullet_data
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Optional
from parse_argv import parse_argv
from io import StringIO
from xacro import process_file
import tempfile
import os


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
    def __init__(self, urdf_path: str, fixed_base: Optional[bool] = None, base_position: Optional[list] = None, enable_self_collision: bool = False):
        self.urdf_path = urdf_path
        with open(self.urdf_path) as f:
            print("\n"*10)
            print(f.read())
            print("\n"*10)
        self.tree = ET.parse(self.urdf_path)
        self.root = self.tree.getroot()

        if fixed_base is None:
            fixed_base = self._detect_fixed()
        if base_position is None:
            base_position = [0, 0, 0] if fixed_base else [0, 0, 1]

        if enable_self_collision:
            self.robot_id = p.loadURDF(self.urdf_path, basePosition=base_position, useFixedBase=fixed_base, flags=p.URDF_USE_SELF_COLLISION)
        else:
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


@dataclass
class Settings:
    urdf: str = DEFAULT_URDF_PATH
    fixed: bool = None
    self_collision: bool = False


def create_robot(settings: Settings) -> int:
    _make_robot = lambda: RobotJoints(urdf_path=settings.urdf, fixed_base=settings.fixed, enable_self_collision=settings.self_collision)
    urdf_path = settings.urdf
    is_xacro = urdf_path.endswith("xacro")
    if not is_xacro:
        return _make_robot()
    
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    urdf_str = process_file(urdf_path).toprettyxml()
    with tempfile.NamedTemporaryFile(suffix='.urdf', mode='w+', dir=urdf_dir, delete=True) as f:
        f.write(urdf_str)
        f.flush()
        settings.urdf = f.name
        robot = _make_robot()
    return robot


@parse_argv
def visualize(settings: Settings):
    floor_id = p.loadURDF("plane.urdf")
    floor_rgb = [0.831, 0.965, 1.0]
    p.changeVisualShape(floor_id, -1, rgbaColor=[*floor_rgb, 1])
    robot = create_robot(settings)
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
