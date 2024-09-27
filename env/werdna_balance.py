import os
import time
import pybullet as p
import pybullet_data
import gymnasium as gym
from gymnasium import spaces
from gymnasium.utils import seeding
import numpy as np

class WerdnaEnv(gym.Env):

    def __init__(self, render_mode="GUI", modelType="model/werdna_bullet.urdf"):
        super().__init__()
        if render_mode == 'GUI':
            p.connect(p.GUI)
        elif render_mode == 'DIRECT':
            p.connect(p.DIRECT)

        self.action_space = spaces.Box(
            low=np.array([-1]),  # Lower bound for both left and right wheel
            high=np.array([1]),   # Upper bound for both left and right wheel
            dtype=np.float32
        )

        # Observation space: [pitch, pitch_rate, wheel velocity, x position]
        self.observation_space = spaces.Box(
            np.array([-np.pi, -15, -np.inf, -15]), 
            np.array([np.pi, 15, np.inf, 15])
        )

        self.modelType = modelType
        self.terminal_reward = 0
        self.prev_pitch = 0
        self.vt = 0
        self.reward = 0
        self.joint_data = {}
        self.current_time_step = 0
        self.max_time_step = 3000
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
        
        vt = 10 * action[0]  # Scaled to [-15, 15] for higher velocity control

        # Set wheel velocities
        p.setJointMotorControl2(self.robotID, left_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=vt)
        p.setJointMotorControl2(self.robotID, right_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=vt)

        # Keep the hips and knees stationary
        for joint_name in ['left_hip_joint', 'left_knee_joint', 'right_hip_joint', 'right_knee_joint']:
            joint_id, _ = self.get_joint(joint_name)
            p.setJointMotorControl2(self.robotID, joint_id, p.POSITION_CONTROL, targetPosition=0)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self, seed=None):
        p.resetSimulation()
        p.setTimeStep(0.01)
        p.setGravity(0, 0, -9.81)
        self.current_time_step = 0
        self.prev_pitch = 0
        if seed is not None:
            self.seed(seed)
        robotPath = pybullet_data.getDataPath()
        self.planeid = p.loadURDF(os.path.join(robotPath, "plane.urdf"), basePosition=[0, 0, 0])
        self.robotID = p.loadURDF(self.modelType, basePosition=[0, 0, 0.0855])
        
        observation = self.get_obs()
        info = self.get_info()
        return observation, info

    def step(self, action):
        p.stepSimulation()
        self.moveRobot(action)
        observations = self.get_obs()
        info = self.get_info()
        done = self.check_done()
        truncated = self.check_terminal_state()
        reward = self.calculate_reward()
        self.current_time_step += 1
        return observations, reward, done, truncated, info

    def get_obs(self):
        # Get the robot's position and orientation
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        self.extract_joint_info(self.robotID)
        euler_angles = p.getEulerFromQuaternion(robot_orientation)
        pitch = euler_angles[1]
        pitch_rate = pitch - self.prev_pitch
        self.prev_pitch = pitch
        left_wheel_velocity = self.joint_data[self.get_joint('left_wheel_joint')[0]]['velocity']
        velocity = left_wheel_velocity  # Average velocity of both wheels (assuming symmetric)
        x_position = robot_position[0]  # Track x position for reward calculation
        return np.array([pitch, pitch_rate, x_position, velocity], dtype=np.float32)

    def get_info(self):
        robot_position, _ = p.getBasePositionAndOrientation(self.robotID)
        return {'position': robot_position}

    def calculate_reward(self):
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        euler_angles = p.getEulerFromQuaternion(robot_orientation)
        pitch = euler_angles[1]
        pitch_rate = self.prev_pitch
        velocity = (self.joint_data[self.get_joint('left_wheel_joint')[0]]['velocity']
                    + self.joint_data[self.get_joint('right_wheel_joint')[0]]['velocity']) / 2
        # Encourage the robot to stay upright and maintain stable velocity
        reward = (1 - abs(pitch) * 0.2) + (1 - abs(pitch_rate) * 0.2) + (1 - abs(velocity) * 0.01)
        return reward

    def check_terminal_state(self):
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        euler_angles = p.getEulerFromQuaternion(robot_orientation)
        pitch_deg = np.rad2deg(euler_angles[1])
        max_distance = 0.1
        distance_from_origin = np.linalg.norm(np.array(robot_position[:2]))
        if abs(pitch_deg) >= 20 or distance_from_origin >= max_distance:
            return True
        return False

    def check_done(self):
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        euler_angles = p.getEulerFromQuaternion(robot_orientation)
        pitch = euler_angles[1]

        if self.current_time_step >= self.max_time_step and abs(np.rad2deg(pitch) < 10):
            return True
        return False

    def close(self):
        p.disconnect()
