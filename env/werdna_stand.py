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

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)

        # Observation space: [pitch, pitch_rate]
        self.observation_space = spaces.Box(
            np.array([-1, -1, -1]), 
            np.array([1, 1, 1]), dtype=np.float32
        )

        self.modelType = modelType
        self.prev_pitch = 0
        self.vt = 0
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
        
        self.vt = action[0]*10

        # print(f'Speed: {vt}')

        # Set wheel velocities
        p.setJointMotorControl2(self.robotID, left_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=self.vt)
        p.setJointMotorControl2(self.robotID, right_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=self.vt)        

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

        self.extract_joint_info(self.robotID)
        
        _, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        
        roll, pitch, yaw =  p.getEulerFromQuaternion(robot_orientation)
        pitch_rate = pitch-self.prev_pitch
        self.prev_pitch= pitch

        normalized_wheel_velocity = self.vt

        return np.array([pitch/np.pi, pitch_rate/np.pi, normalized_wheel_velocity], dtype=np.float32)

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

        # Total reward combining posture and movement smoothness
        reward = 1 - abs(0 - pitch)*0.01 

        # Clip the reward to ensure it stays non-negative
        reward = max(reward, 0.0)

        return reward

    def check_terminal_state(self):
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)

        roll, pitch, yaw = p.getEulerFromQuaternion(robot_orientation)
        pitch_deg = np.rad2deg(pitch)

        # Early in training, allow more leeway
        if ((pitch_deg) < -50 or (pitch_deg>80)) or abs(robot_position[0])>0.5:
                return True
        elif self.current_time_step > self.max_time_step:
            return True
            
        return False

    def check_done(self):
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        euler_angles = p.getEulerFromQuaternion(robot_orientation)
        pitch = euler_angles[1]

        if self.current_time_step > 500 and abs(np.rad2deg(pitch)) < 30:
            return True
        else:
            return False

    def render(self, mode='human', close=False):
        pass
    
    def close(self):
        p.disconnect()
