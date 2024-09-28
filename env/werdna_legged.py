import os
import pybullet as p 
import pybullet_data

import gymnasium as gym
from gymnasium import spaces
from gymnasium.utils import seeding
import numpy as np

class WerdnaLeggedEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

    def __init__(self, render_mode='GUI', modelType='model/werdna_bullet.urdf'):

        if render_mode == 'GUI':
            p.connect(p.GUI)
        elif render_mode == 'DIRECT':
            p.connect(p.DIRECT)

        # Action space : Wheel speed, left leg displacement, right leg position
        self.action_space = spaces.Box(
            np.array(
                [-1.0, -1.0, -1.0]
            ),
            np.array(
                [1.0, 1.0, 1.0]
            ),
            dtype=np.float32
        )

        # Define observation space limits
        obs_low = np.array([-np.pi,   # Min pitch (radians)
                            -10.0,    # Min pitch_rate (radians/second)
                            -np.pi,   # Min roll (radians)
                            -10.0,    # Min roll_rate (radians/second)
                            -0.0,     # Min left_leg_position 
                            -0.0      # Min right_leg_position 
                        ])

        obs_high = np.array([np.pi,    # Max pitch (radians)
                            10.0,     # Max pitch_rate (radians/second)
                            np.pi,    # Max roll (radians)
                            10.0,     # Max roll_rate (radians/second)
                            0.1,      # Max left_leg_position 
                            0.1       # Max right_leg_position 
                            ])

        # Create the observation space using Box
        self.observation_space = spaces.Box(low=obs_low, high=obs_high, dtype=np.float32)

        self.modelType = modelType
        self.prev_pitch, self.prev_roll = 0
        self.vt, self.left_p, self.right_p
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
        left_hip_joint_id, _ = self.get_joint('left_hip_joint')
        right_hip_joint_id, _ = self.get_joint('right_hip_joint')
        left_knee_joint_id, _ = self.get_joint('left_knee_joint')
        right_knee_joint_id, _ = self.get_joint('right_knee_joint')

        self.vt = action[0] * 10
        self.left_p = 0.05 * (action[1] + 1)
        self.right_p = 0.05 * (action[2] + 1)

        # Set wheel velocities
        p.setJointMotorControl2(self.robotID, left_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=self.vt)
        p.setJointMotorControl2(self.robotID, right_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=self.vt) 

        # Calculate knee and hip angles using inverse kinematics
        left_knee, left_hip = self.inverse_kinematics(self.left_p, 0)
        right_knee, right_hip = self.inverse_kinematics(self.right_p, 0)

        # Set knee and hip joint positions
        p.setJointMotorControl2(self.robotID, left_knee_joint_id, p.POSITION_CONTROL, targetPosition=left_knee, force = 10.0, maxVelocity = 2.0)
        p.setJointMotorControl2(self.robotID, left_hip_joint_id, p.POSITION_CONTROL, targetPosition=left_hip, force = 10.0, maxVelocity = 2.0)
        p.setJointMotorControl2(self.robotID, right_knee_joint_id, p.POSITION_CONTROL, targetPosition=right_knee, force = 10.0, maxVelocity = 2.0)
        p.setJointMotorControl2(self.robotID, right_hip_joint_id, p.POSITION_CONTROL, targetPosition=right_hip, force = 10.0, maxVelocity = 2.0)
    
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self, seed=None):
        if seed is not None:
            self.seed(seed)

        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(0.02)

        robotPath = pybullet_data.getDataPath()
        self.planeid = p.loadURDF(os.path.join(robotPath, "plane.urdf"), basePosition=[0, 0, 0])
        self.robotID = p.loadURDF(self.modelType, basePosition=[0, 0, 0.0855])

        observation = self.get_obs()
        info = self.get_info()

        self.current_time_step = 0
        self.prev_pitch, self.prev_roll = 0
        self.vt, self.left_p, self.right_p = 0

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
        
        _, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        
        roll, pitch, yaw =  p.getEulerFromQuaternion(robot_orientation)
        pitch_rate = pitch-self.prev_pitch
        roll_rate = roll - self.prev_roll
        self.prev_pitch, self.prev_roll = pitch, roll

        return np.array([pitch, pitch_rate, roll, roll_rate, self.left_p, self.right_p], dtype=np.float32)
    
    def get_info(self):
        _, left_hip_data = self.get_joint('left_hip_joint')
        _, right_hip_data = self.get_joint('right_hip_joint')
        _, left_knee_data = self.get_joint('left_knee_joint')
        _, right_knee_data = self.get_joint('right_knee_joint')

        left_hip_position = left_hip_data['position']
        right_hip_position = right_hip_data['position']
        right_knee_position = right_knee_data['position']
        left_knee_position = left_knee_data['position']

        return np.array([left_hip_position, right_hip_position, left_knee_position, right_knee_position])
    
    def check_terminal_state(self):
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)

        roll, pitch, yaw = p.getEulerFromQuaternion(robot_orientation)
        roll_deg, pitch_deg = np.rad2deg(roll), np.rad2deg(pitch)

        if abs(roll_deg > 30) or abs (pitch_deg > 45):
            return True
        else:
            return False

    def check_done(self):
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)

        roll, pitch, yaw = p.getEulerFromQuaternion(robot_orientation)
        roll_deg, pitch_deg = np.rad2deg(roll), np.rad2deg(pitch)

        if self.current_time_step >= 500 and (abs(roll_deg > 30) and abs (pitch_deg > 45)):
            return True
        else:
            return False

    def calculate_reward(self):
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)

        roll, pitch, yaw = p.getEulerFromQuaternion(robot_orientation)
        pitch_rate = pitch-self.prev_pitch
        roll_rate = roll - self.prev_roll

        reward = (1 - abs(0 - pitch))*0.01  # Limit pitch error impact
        reward += (1 - abs(0 - roll))*0.01  # Limit pitch error impact
        reward += min(max(pitch_rate, -0.5), 0.5) * 0.01           # Slightly increase pitch rate influence
        reward += min(max(roll_rate, -0.5), 0.5) * 0.01           # Slightly increase pitch rate influence

        return reward

    def render(self, mode='human', close=False):
        pass

    def close(self):
        p.disconnect()