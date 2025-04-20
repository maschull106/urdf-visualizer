from __future__ import annotations
import pybullet as p
import pybullet_data
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass

URDF_PATH = "test.urdf"


class Joint:
    def __init__(self, robot_id: int, idx: int):
        self.idx = idx
        self.robot_id = robot_id
    
    def set_pos(self, val: float):
        p.setJointMotorControl2(self.robot_id, self.idx, p.POSITION_CONTROL, targetPosition=val)


class RobotJoints:
    def __init__(self, urdf_path: str, base_position: list):
        self.urdf_path = urdf_path
        self.tree = ET.parse(URDF_PATH)
        self.root = self.tree.getroot()
        self.robot_id = p.loadURDF(URDF_PATH, basePosition=base_position)

        self.find_joints()
    
    def find_joints(self):
        self.index_to_name: dict[int, str] = {}
        self.name_to_index: dict[str, int] = {}
        self.joints: dict[str, Joint] = {}
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            name = joint_info[1].decode("utf-8")
            self.index_to_name[i] = name
            self.name_to_index[name] = i
            self.joints[name] = Joint(self.robot_id, i)

    def add_control_sliders(self):
        self.sliders = {}

        for name, joint in self.joints.items():
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


def visualize():
    floor_id = p.loadURDF("plane.urdf")
    robot = RobotJoints(URDF_PATH, base_position=[0, 0, 1])
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
