import os
import pybullet as p 
import pybullet_data

import gymnasium as gym
from gymnasium import spaces
from gymnasium.utils import seeding

import numpy as np

from utils.collision_detector import CollisionDetector

class WerdnaLeggedEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

    def __init__(self, render_mode='GUI', modelType='model/werdna_bullet.urdf'):

        if render_mode == 'GUI':
            p.connect(p.GUI)
        elif render_mode == 'DIRECT':
            p.connect(p.DIRECT)

        # Action space : Wheel speed, left leg displacement, right leg position
        self.action_space = spaces.Box(low=np.array([-1.0]), high = np.array([1.0]), dtype=np.float32)


        # Define observation space limits
        obs_low = np.array([-np.pi,   # Min pitch (radians)
                            -np.pi,    # Min pitch_rate (radians/second)
                            -1.0,   #wheel velocity
                            -1.0    # Min leg_position    
                        ])

        obs_high = np.array([np.pi,    # Max pitch (radians)
                            np.pi,     # Max pitch_rate (radians/second)
                            1.0,    # wheel velocity
                            1.0     # Max_leg_position      
                            ])

        # Create the observation space using Box
        self.observation_space = spaces.Box(low=obs_low, high=obs_high, dtype=np.float32)

        self.modelType = modelType
        self.prev_pitch = 0
        self.prev_roll = 0
        self.left_wheel = 0
        self.right_wheel= 0
        self.left_p = 0
        self.right_p = 0
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
    
    def checkCollision(self):

        ground = self.planeid
        left_hip = (self.robotID, "left_hip_link")
        left_knee = (self.robotID, "left_knee_link")
        right_hip = (self.robotID, "right_hip_link")
        right_knee = (self.robotID, "right_knee_link")
        # chassis = (self.robotID, "base_link")

        col_detector = CollisionDetector(
            self.planeid,
            [(left_hip, ground), (left_knee, ground), (right_hip, ground), (right_knee, ground)],
        )

        return col_detector.in_collision(0.0001)

    def inverse_kinematics(self, height=0, displacement=0):
        if height != 0:
            L1 = L2 = 0.1
            knee_theta = (np.pi / 3) + np.arccos((L1**2 + L2**2 - height**2) / (2 * L1 * L2))
            hip_theta = np.arcsin(displacement / height) - np.arccos((L1**2 + height**2 - L2**2) / (2 * L1 * height)) 

            return hip_theta, knee_theta
        else:
            return 0, 0

    def moveRobot(self, action):
        # Get joint IDs for wheels and hips
        left_wheel_joint_id, _ = self.get_joint('left_wheel_joint')
        right_wheel_joint_id, _ = self.get_joint('right_wheel_joint')
        left_hip_joint_id, _ = self.get_joint('left_hip_joint')
        right_hip_joint_id, _ = self.get_joint('right_hip_joint')
        left_knee_joint_id, _ = self.get_joint('left_knee_joint')
        right_knee_joint_id, _ = self.get_joint('right_knee_joint')

        velocity = action[0]*10
        

        self.left_wheel = velocity 
        self.right_wheel = velocity
        self.left_wheel = np.clip(self.left_wheel, -5.0, 5.0)
        self.right_wheel = np.clip(self.right_wheel, -5.0, 5.0)

        # Set wheel velocities
        p.setJointMotorControl2(self.robotID, left_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=self.left_wheel)
        p.setJointMotorControl2(self.robotID, right_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=self.right_wheel) 
        
        # height = 0.1*(action[0]+1)
        self.left_p = 0.1
        self.right_p = 0.1

        # Calculate knee and hip angles using inverse kinematics
        left_hip, left_knee = self.inverse_kinematics(self.left_p, 0)
        right_hip, right_knee = self.inverse_kinematics(self.right_p, 0)

        # Set knee and hip joint positions
        p.setJointMotorControl2(self.robotID, left_knee_joint_id, p.POSITION_CONTROL, targetPosition=left_knee, force = 8.0, maxVelocity = 2.0)
        p.setJointMotorControl2(self.robotID, left_hip_joint_id, p.POSITION_CONTROL, targetPosition=left_hip, force = 8.0, maxVelocity = 2.0)
        p.setJointMotorControl2(self.robotID, right_knee_joint_id, p.POSITION_CONTROL, targetPosition=right_knee, force = 8.0, maxVelocity = 2.0)
        p.setJointMotorControl2(self.robotID, right_hip_joint_id, p.POSITION_CONTROL, targetPosition=right_hip, force = 8.0, maxVelocity = 2.0)
    
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self, seed=None):

        if seed is not None:
            self.seed(seed)

        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(0.01)

        robotPath = pybullet_data.getDataPath()
        self.planeid = p.loadURDF(os.path.join(robotPath, "plane.urdf"), basePosition=[0, 0, 0])
        self.robotID = p.loadURDF(self.modelType, basePosition=[0, 0, 0.0855])

        observation = self.get_obs()
        info = self.get_info()

        self.current_time_step = 0
        self.prev_pitch = 0
        self.left_wheel = 0
        self.right_wheel = 0
        self.left_p = 0
        self.right_p = 0

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
        self.prev_pitch = pitch

        normalized_wheel_velocity = self.left_wheel/10

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
    
    def check_terminal_state(self):
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)

        roll, pitch, yaw = p.getEulerFromQuaternion(robot_orientation)
        pitch_deg = np.rad2deg(pitch)

        # Early in training, allow more leeway
        # if abs(pitch_deg>20):
        #     pass
        if (self.current_time_step>50 and self.checkCollision()):
            return True
        if self.current_time_step > self.max_time_step:
            return True
        elif abs(robot_position[0])> 0.1:
            return True
            
        return False
    
    def check_done(self):
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)

        roll, pitch, yaw = p.getEulerFromQuaternion(robot_orientation)
        roll_deg, pitch_deg = np.rad2deg(roll), np.rad2deg(pitch)

        # if self.current_time_step > self.max_time_step and abs (pitch_deg < 45):
        #     return True
        if self.current_time_step > 500 and not self.checkCollision():
            return True
        else:
            return False

    def calculate_reward(self):
        # robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robotID)
        # roll, pitch, yaw = p.getEulerFromQuaternion(robot_orientation)

        # # Penalize for deviation from upright position
        # pitch_penalty = abs(pitch)   

        # pitch_rate_penalty = abs(pitch-self.prev_pitch)

        # # Reward for maintaining upright posture
        # upright_bonus = (0.5 * pitch_penalty) 
        # upright_rate_bonus = (0.5 * pitch_rate_penalty)

        # # Total reward combining posture and movement smoothness
        # reward = 1 - upright_bonus - upright_rate_bonus

        reward=0

        if self.checkCollision():
            reward += 2
        else:
            reward -= 2.0

        # Clip the reward to ensure it stays non-negative
        reward = max(reward, 0.0)

        return reward

    def render(self, mode='human', close=False):
        pass

    def close(self):
        p.disconnect()