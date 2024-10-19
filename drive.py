from stable_baselines3 import PPO
import gymnasium as gym
from gymnasium import spaces

import numpy as np
import math
import os
import time

import pybullet as p 
import pybullet_data

from utils import parser as parse

class Wheeled_Bipedal():

    def __init__(self, model="model"):

        # Set up 
        p.connect(p.GUI)

        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high = np.array([1.0, 1.0]), dtype=np.float32)

        self.observation_space = spaces.Box(
            np.array([-np.inf, -np.inf, -np.inf, -np.inf]), 
            np.array([np.inf, np.inf, np.inf, np.inf])
        )

        self.modelType = model
        
        self.prev_pitch = 0
        self.steer = 0
        self.forward = 0
        self.joint_data = {}
        self.left_wheel = 0
        self.right_wheel = 0

        self.set_up_environment()


    def extract_joint_info(self, robotID):
        num_joints = p.getNumJoints(robotID)
        for i in range(num_joints):
            joint_info = p.getJointInfo(robotID, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_state = p.getJointState(robotID, i)
            joint_position = joint_state[0]
            joint_velocity = joint_state[1]
            self.joint_data[i] = {
                'name': joint_name,
                'position': joint_position,
                'velocity': joint_velocity
            }

    def get_joint(self, name):
        for joint_id, data in self.joint_data.items():
            if data['name'] == name:
                return joint_id, data
        return None 

    def set_up_environment(self):

        p.resetSimulation()
        p.setTimeStep(0.01)
        p.setGravity(0, 0, -9.81)

        robotPath = pybullet_data.getDataPath()
        self.planeid = p.loadURDF(os.path.join(robotPath, "plane.urdf"), basePosition=[0, 0, 0])
        self.robotID = p.loadURDF(self.modelType, basePosition=[0, 0, 0.3])

    
    def moveRobot(self, action):

        # Get joint IDs for wheels and hips
        left_wheel_joint_id, _ = self.get_joint('left_wheel_joint')
        right_wheel_joint_id, _ = self.get_joint('right_wheel_joint')
        left_hip_joint_id, _ = self.get_joint('left_hip_motor_joint')
        right_hip_joint_id, _ = self.get_joint('right_hip_motor_joint')

        self.left_wheel = action[0]*10 
        self.right_wheel = action[0]*10 

        # Set wheel velocities
        p.setJointMotorControl2(self.robotID, left_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=self.left_wheel)
        p.setJointMotorControl2(self.robotID, right_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=self.right_wheel)

        offset = action[1]*np.pi/6

        p.setJointMotorControl2(self.robotID, left_hip_joint_id, p.POSITION_CONTROL, targetPosition=offset, force=10.0, maxVelocity=2.0)
        p.setJointMotorControl2(self.robotID, right_hip_joint_id, p.POSITION_CONTROL, targetPosition=offset, force=10.0, maxVelocity=2.0)
        

    def get_obs(self):
        self.extract_joint_info(self.robotID)
        
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        
        roll, pitch, yaw =  p.getEulerFromQuaternion(robot_orientation)
        
        pitch_rate = pitch-self.prev_pitch
        self.prev_pitch= pitch

        _, left_wheel_joint_info = self.get_joint('left_wheel_joint')
        left_wheel_joint_velocity = left_wheel_joint_info['velocity']

        _, right_wheel_joint_info = self.get_joint('right_wheel_joint')
        right_wheel_joint_velocity = right_wheel_joint_info['velocity']

        velocity = (left_wheel_joint_velocity + right_wheel_joint_velocity)/2

        pitch = pitch - self.forward/math.pi

        return np.array([pitch/np.pi, pitch_rate/np.pi, robot_position[0], velocity], dtype=np.float32)
        
    
    def step(self, action):

        self.teleop()
        self.moveRobot(action)

        p.stepSimulation()

    def teleop(self):  
        keys = p.getKeyboardEvents()
        if ord('i') in keys and keys[ord('i')] & p.KEY_IS_DOWN:
            self.forward = 1.0
        elif ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN:
            self.steer = 1.0
        if ord('k') in keys and keys[ord('k')] & p.KEY_IS_DOWN:
            self.forward = -1.0
        elif ord('l') in keys and keys[ord('l')] & p.KEY_IS_DOWN:  # Changed from ':' to ';'
            self.steer = -1.0
        

def main():

    config = parse.parser("config/config2.yaml")
    filename = config['filename']

    model_filename = os.path.join('results', filename)
    model = PPO.load(model_filename+'.zip')

    robot = Wheeled_Bipedal(model="model/werdna_v2_bullet.urdf")

    while True:
        time.sleep(1./60.)

        obs = robot.get_obs()

        actions, _ = model.predict(obs)

        robot.step(actions)




if __name__ == "__main__":
    main()
