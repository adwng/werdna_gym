import os
import time

import pybullet as p
import pybullet_data

import gymnasium as gym
from gymnasium import spaces
from gymnasium.utils import seeding

from utils.collision_detector import CollisionDetector

import numpy as np

class Werdna3Env(gym.Env):

    def __init__(self, render_mode="GUI", modelType="model/werdna_v2_bullet.urdf"):
        super().__init__()
        if render_mode == 'GUI':
            p.connect(p.GUI)
        elif render_mode == 'DIRECT':
            p.connect(p.DIRECT)

        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high = np.array([1.0, 1.0]), dtype=np.float32)

        # Observation space: [pitch, pitch_rate, wheel velocity, x position]
        self.observation_space = spaces.Box(
            np.array([-np.inf, -np.inf, -np.inf, -np.inf]), 
            np.array([np.inf, np.inf, np.inf, np.inf])
        )

        self.modelType = modelType
        self.prev_pitch = 0
        self.left_wheel = 0
        self.right_wheel = 0
        self.joint_data = {}
        self.current_time_step = 0
        self.max_time_step = 2048
        self.seed()

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
        return None  # Return None if the joint with the specified name is not found

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

    def checkCollision(self):

        ground = self.planeid
        left_knee = (self.robotID, "left_knee_motor_link")
        right_knee = (self.robotID, "right_knee_motor_link")
        

        col_detector = CollisionDetector(
            self.planeid,
            [(left_knee, ground), (right_knee, ground)],
        )

        return col_detector.in_collision(0.001)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self, seed=None):

        if seed is not None:
            self.seed(seed)

        p.resetSimulation()
        p.setTimeStep(0.01)
        p.setGravity(0, 0, -9.81)

        robotPath = pybullet_data.getDataPath()
        self.planeid = p.loadURDF(os.path.join(robotPath, "plane.urdf"), basePosition=[0, 0, 0])
        self.robotID = p.loadURDF(self.modelType, basePosition=[0, 0, 0.3])
        
        observation = self.get_obs()
        info = self.get_info()

        self.current_time_step = 0
        self.prev_pitch = 0
        self.left_wheel = 0
        self.right_wheel = 0
        
        return observation, info

    def step(self, action):

        self.moveRobot(action)
        p.stepSimulation()

        observations = self.get_obs()
        info = self.get_info()
        done = self.check_done()
        truncated = self.check_terminal_state()
        reward = self.calculate_reward()

        self.current_time_step += 1

        return observations, reward, done, truncated, info

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

        position = robot_position[0]

        return np.array([pitch, pitch_rate, position, velocity], dtype=np.float32)

    def get_info(self):
        _, left_hip_data = self.get_joint('left_hip_joint')
        _, right_hip_data = self.get_joint('right_hip_joint')
        _, left_knee_data = self.get_joint('left_knee_joint')
        _, right_knee_data = self.get_joint('right_knee_joint')

        left_hip_position = left_hip_data['position']
        right_hip_position = right_hip_data['position']
        left_knee_position = left_knee_data['position']
        right_knee_position = right_knee_data['position']

        # Return as a dictionary
        return {
            'left_hip_position': left_hip_position,
            'right_hip_position': right_hip_position,
            'left_knee_position': left_knee_position,
            'right_knee_position': right_knee_position
        }

    def calculate_reward(self):

        # Get position, orientation, roll, pitch, and yaw
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        x,y,z = robot_position
        roll, pitch, yaw = p.getEulerFromQuaternion(robot_orientation)

        # Get current's position and velocity
        _, left_wheel_joint_info = self.get_joint('left_wheel_joint')
        left_wheel_joint_velocity = left_wheel_joint_info['velocity']

        _, right_wheel_joint_info = self.get_joint('right_wheel_joint')
        right_wheel_joint_velocity = right_wheel_joint_info['velocity']

        # Get z position based on pitch
        z_position = z*np.sin(pitch)
        z_position_reward = np.exp(-((z_position / 0.05) ** 2))

        # Get x position based on delta
        x_position = robot_position[0]

        x_position_reward = np.exp(-((x_position / 0.05) ** 2))

        # Get velocity based on delta
        velocity = (left_wheel_joint_velocity + right_wheel_joint_velocity)/2  

        velocity = z* velocity * np.cos(pitch)
        
        velocity_reward = -abs(velocity)

        # Estimate weights for accounting for pitch using z_position, and linear position using x_position, add 0.1 for every step to encourage standstill
        reward = (0.6*z_position_reward) + (0.4*x_position_reward) + (0.0*velocity_reward) + 0.1
        return reward

    def check_terminal_state(self):

        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)

        # print(robot_position[0])

        roll, pitch, yaw = p.getEulerFromQuaternion(robot_orientation)
        pitch_deg = np.rad2deg(pitch)

        # Early in training, allow more leeway
        if (pitch_deg > 35):
            return True
        elif(pitch_deg<-35):
            return True
        elif self.current_time_step > self.max_time_step:
            return True
        elif abs(robot_position[0])> 1.0:
            return True
        elif self.checkCollision():
            return True
            
        return False

    def check_done(self):
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        euler_angles = p.getEulerFromQuaternion(robot_orientation)
        pitch = euler_angles[1]

        if self.current_time_step > self.max_time_step and abs(np.rad2deg(pitch) < 15):
            return True
        return False
    
    def render(self, mode='human', close=False):
        pass

    def close(self):
        p.disconnect()
