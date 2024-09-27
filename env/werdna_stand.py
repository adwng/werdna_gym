import os
import math
import pybullet as p
import pybullet_data
import gymnasium as gym
from gymnasium import spaces
from gymnasium.utils import seeding
import numpy as np

class WerdnaStandEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

    def __init__(self, render_mode="GUI", modelType="model/werdna_bullet.urdf"):

        if render_mode == 'GUI':
            p.connect(p.GUI)
        elif render_mode == 'DIRECT':
            p.connect(p.DIRECT)

        self.action_space = spaces.Discrete(9)

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

    def inverse_kinematics(self, f=0, x=0):
        """
        Performs inverse kinematics calculations to get knee and hip angles.
        
        Args:
            f: The target foot position (height).
            x: The x-axis displacement (currently unused).
            
        Returns:
            A tuple (knee_theta, hip_theta) representing the joint angles.
        """
        L1 = L2 = 0.1
        if f > 0:
            knee_theta = (np.pi / 3) + np.arccos((L1**2 + L2**2 - f**2) / (2 * L1 * L2))
            hip_theta = np.arcsin(x / f) - np.arccos((L1**2 + f**2 - L2**2) / (2 * L1 * f))
            return knee_theta, hip_theta
        else:
            return 0,0

    def moveRobot(self, action):
        # Get joint IDs for wheels and hips
        left_wheel_joint_id, _ = self.get_joint('left_wheel_joint')
        right_wheel_joint_id, _ = self.get_joint('right_wheel_joint')
        
        dv = 0.1
        deltav = [-5.*dv,-2.5*dv, -1.*dv, -0.5*dv, 0, 0.5*dv, 1.*dv,2.5*dv, 5.*dv][action]
        vt = self.vt + deltav
        self.vt = vt

        # print(f'Speed: {vt}')

        # Set wheel velocities
        p.setJointMotorControl2(self.robotID, left_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=vt)
        p.setJointMotorControl2(self.robotID, right_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=vt)        

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self, seed=None):
        if seed is not None:
            self.seed(seed)

        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(0.02)

        self.current_time_step = 0
        self.prev_pitch = 0
        self.vt = 0

        robotPath = pybullet_data.getDataPath()
        self.planeid = p.loadURDF(os.path.join(robotPath, "plane.urdf"), basePosition=[0, 0, 0])
        self.robotID = p.loadURDF(self.modelType, basePosition=[0, 0, 0.0855])

        observation = self.get_obs()
        info = self.get_info()

        left_hip_joint_id, _ = self.get_joint('left_hip_joint')
        left_knee_joint_id, _ = self.get_joint('left_knee_joint')
        right_hip_joint_id, _ = self.get_joint('right_hip_joint')
        right_knee_joint_id, _ = self.get_joint('right_knee_joint')

        # Calculate knee and hip angles using inverse kinematics
        left_knee, left_hip = self.inverse_kinematics(0.1, 0)
        right_knee, right_hip = self.inverse_kinematics(0.1, 0)

        # Set knee and hip joint positions
        p.setJointMotorControl2(self.robotID, left_knee_joint_id, p.POSITION_CONTROL, targetPosition=left_knee, force = 10.0, maxVelocity = 2.0)
        p.setJointMotorControl2(self.robotID, left_hip_joint_id, p.POSITION_CONTROL, targetPosition=left_hip, force = 10.0, maxVelocity = 2.0)
        p.setJointMotorControl2(self.robotID, right_knee_joint_id, p.POSITION_CONTROL, targetPosition=right_knee, force = 10.0, maxVelocity = 2.0)
        p.setJointMotorControl2(self.robotID, right_hip_joint_id, p.POSITION_CONTROL, targetPosition=right_hip, force = 10.0, maxVelocity = 2.0)

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
        # Get the robot's position and orientation
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        self.extract_joint_info(self.robotID)
        euler_angles = p.getEulerFromQuaternion(robot_orientation)
        pitch = euler_angles[1]
        pitch_rate = pitch - self.prev_pitch
        self.prev_pitch = pitch
        left_wheel_velocity = self.joint_data[self.get_joint('left_wheel_joint')[0]]['velocity']
        right_wheel_velocity = self.joint_data[self.get_joint('right_wheel_joint')[0]]['velocity']
        velocity = (left_wheel_velocity + right_wheel_velocity)/2  # Average velocity of both wheels 
        x_position = (left_wheel_velocity + right_wheel_velocity)/0.4855
        return np.array([pitch, pitch_rate, x_position, velocity], dtype=np.float32)

    def get_info(self):
        robot_position, _ = p.getBasePositionAndOrientation(self.robotID)
        return {'position': robot_position}

    def calculate_reward(self):

        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        euler_angles = p.getEulerFromQuaternion(robot_orientation)
        pitch = euler_angles[1]

        pitch_rate = pitch - self.prev_pitch

        velocity = (self.joint_data[self.get_joint('left_wheel_joint')[0]]['velocity']
                    + self.joint_data[self.get_joint('right_wheel_joint')[0]]['velocity']) / 2
        
        # Encourage the robot to stay upright and maintain stable velocity
        reward = (1 - abs(0 - pitch))*0.01  # Limit pitch error impact
        reward += min(max(pitch_rate, -0.5), 0.5) * 0.01           # Slightly increase pitch rate influence
        reward += min(max(velocity, -5), 5) * 0.005                # Slightly increase velocity influence
        # print(f'Reward: {reward}')

        return reward

    def check_terminal_state(self):
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        euler_angles = p.getEulerFromQuaternion(robot_orientation)
        pitch_deg = np.rad2deg(euler_angles[1])
        max_distance = 0.1
        distance_from_origin = np.linalg.norm(np.array(robot_position[:2]))
        if abs(pitch_deg) >= 30:
            # print(pitch_deg)
            return True
        else:
            return False

    def check_done(self):
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        euler_angles = p.getEulerFromQuaternion(robot_orientation)
        pitch = euler_angles[1]

        if self.current_time_step >= self.max_time_step and abs(np.rad2deg(pitch) < 15):
            print('Done')
            return True
        return False

    def render(self, mode='human', close=False):
        pass
    def close(self):
        p.disconnect()
