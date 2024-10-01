import os
import time

import pybullet as p
import pybullet_data

import gymnasium as gym
from gymnasium import spaces
from gymnasium.utils import seeding

from utils.collision_detector import CollisionDetector

import numpy as np

class WerdnaEnv(gym.Env):

    def __init__(self, render_mode="GUI", modelType="model/werdna_bullet.urdf"):
        super().__init__()
        if render_mode == 'GUI':
            p.connect(p.GUI)
        elif render_mode == 'DIRECT':
            p.connect(p.DIRECT)

        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high = np.array([1.0, 1.0]), dtype=np.float32)

        # Observation space: [pitch, pitch_rate, wheel velocity, x position]
        self.observation_space = spaces.Box(
            np.array([-np.pi, -np.pi, -1.0, -1.0]), 
            np.array([np.pi, np.pi, 1.0, 1.0])
        )

        self.modelType = modelType
        self.prev_pitch = 0
        self.vt = 0
        self.hip_position = 0
        self.knee_position = 0
        self.joint_data = {}
        self.current_time_step = 0
        self.max_time_step = 1500
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
        left_hip_joint_id, _ = self.get_joint('left_hip_joint')
        right_hip_joint_id, _ = self.get_joint('right_hip_joint')
        
        velocity = action[0]*10
        self.vt = velocity

        self.left_wheel = velocity 
        self.right_wheel = velocity

        # Set wheel velocities
        p.setJointMotorControl2(self.robotID, left_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=self.left_wheel)
        p.setJointMotorControl2(self.robotID, right_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=self.right_wheel)

        offset = action[1]*np.pi/6

        p.setJointMotorControl2(self.robotID, left_hip_joint_id, p.POSITION_CONTROL, targetPosition=offset, force=5.0, maxVelocity=2.0)
        p.setJointMotorControl2(self.robotID, right_hip_joint_id, p.POSITION_CONTROL, targetPosition=offset, force=5.0, maxVelocity=2.0)

    def checkCollision(self):

        ground = self.planeid
        left_hip = (self.robotID, "left_hip_link")
        left_knee = (self.robotID, "left_knee_link")
        right_hip = (self.robotID, "right_hip_link")
        right_knee = (self.robotID, "right_knee_link")

        col_detector = CollisionDetector(
            self.planeid,
            [(left_hip, ground), (left_knee, ground), (right_hip, ground), (right_knee, ground)],
        )

        return col_detector.in_collision(0.001)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self, seed=None):

        if seed is not None:
            self.seed(seed)

        p.resetSimulation()
        p.setTimeStep(0.02)
        p.setGravity(0, 0, -9.81)

        robotPath = pybullet_data.getDataPath()
        self.planeid = p.loadURDF(os.path.join(robotPath, "plane.urdf"), basePosition=[0, 0, 0])
        self.robotID = p.loadURDF(self.modelType, basePosition=[0, 0, 0.15])
        
        observation = self.get_obs()
        info = self.get_info()

        self.current_time_step = 0
        self.prev_pitch = 0
        self.vt = 0 
        
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
        # print(np.rad2deg(pitch))
        pitch_rate = pitch-self.prev_pitch
        self.prev_pitch= pitch

        normalized_wheel_velocity = self.vt / 10

        return np.array([pitch/np.pi, pitch_rate/np.pi, normalized_wheel_velocity, robot_position[0]], dtype=np.float32)

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

        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        roll, pitch, yaw = p.getEulerFromQuaternion(robot_orientation)

        # Penalize for deviation from upright position (using squared penalty for larger deviations)
        pitch_penalty = abs(pitch) ** 2  # Stronger penalty for larger deviations

        # Penalize for rapid changes in pitch (squared to penalize sudden large changes)
        pitch_rate_penalty = abs(pitch - self.prev_pitch) ** 2

        # Reward for maintaining upright posture (inverse of penalty)
        upright_bonus = max(1 - pitch_penalty, 0.0)  # Ensure non-negative bonuss

        # Reward for smooth posture transition (inverse of pitch rate penalty)
        upright_rate_bonus = max(1 - pitch_rate_penalty, 0.0)

        # Total reward combining posture and movement smoothness
        reward = 0.5 * (upright_bonus + upright_rate_bonus)

        # Penalize collisions
        if self.checkCollision():
            reward -= 2  # Stronger penalty for collisions
        else:
            reward += 1  # Reward for not colliding

        # Optional: Reward for forward movement or other tasks
        # velocity_reward = max(self.get_velocity(), 0)  # Could reward forward motion
        # reward += velocity_reward

        # Clip the reward to ensure it stays non-negative
        reward = max(reward, 0.0)


        return reward

    def check_terminal_state(self):

        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)

        # print(robot_position[0])

        roll, pitch, yaw = p.getEulerFromQuaternion(robot_orientation)
        pitch_deg = np.rad2deg(pitch)

        # Early in training, allow more leeway
        if (pitch_deg > 45):
            return True
        elif(pitch_deg<-30):
            return True
        # elif (self.current_time_step>50 and self.checkCollision()):
        #     return True
        elif self.current_time_step > self.max_time_step:
            return True
        elif abs(robot_position[0])> 1.0:
            return True
            
        return False

    def check_done(self):
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        euler_angles = p.getEulerFromQuaternion(robot_orientation)
        pitch = euler_angles[1]

        if self.current_time_step >= 100 and abs(np.rad2deg(pitch) < 10):
            return True
        return False
    
    def render(self, mode='human', close=False):
        pass

    def close(self):
        p.disconnect()
