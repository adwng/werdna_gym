import pybullet as p
import pybullet_data
import time
import numpy as np

joint_data = {}

def get_joint(name):
    for joint_id, data in joint_data.items():
        if data['name'] == name:
            return joint_id, data
    return None  # Return None if the joint with the specified name is not found


def extract_joint_info(robot_id):
    """
    Extracts joint information from the robot.

    Args:
        robot_id: The ID of the robot in the PyBullet simulation.

    Returns:
        A dictionary with joint indices as keys and another dictionary with joint name,
        position, and velocity as values.
    """
    num_joints = p.getNumJoints(robot_id)

    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')

        # Get the joint state (position, velocity, etc.)
        joint_state = p.getJointState(robot_id, i)
        joint_position = joint_state[0]  # Position
        joint_velocity = joint_state[1]  # Velocity

        # Store the data in the dictionary
        joint_data[i] = {
            'name': joint_name,
            'position': joint_position,
            'velocity': joint_velocity
        }

        # print(joint_info)

    return joint_data

def teleop():
    left_wheel_joint_id, left_wheel_joint_info = get_joint('left_wheel_joint')
    right_wheel_joint_id, right_wheel_joint_info = get_joint('right_wheel_joint')
    left_hip_joint_id, left_hip_joint_info = get_joint('left_hip_joint')
    left_knee_joint_id, left_knee_joint_info = get_joint('left_knee_joint')
    right_hip_joint_id, right_hip_joint_info = get_joint('right_hip_joint')
    right_knee_joint_id, right_knee_joint_info = get_joint('right_knee_joint')

    max_wheel_velocity = 10.0
    max_height = 0.10
    max_displacement = 0.05

    keys = p.getKeyboardEvents()

    left_wheel_velocity = 0
    right_wheel_velocity = 0

    # Initialize leg positions
    global left_leg_position, right_leg_position, left_leg_displacement, right_leg_displacement

    # Check for key inputs and adjust velocities/positions accordingly
    if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
        left_wheel_velocity = max_wheel_velocity
        right_wheel_velocity = max_wheel_velocity
    elif ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
        left_wheel_velocity = -max_wheel_velocity
        right_wheel_velocity = max_wheel_velocity
    elif ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
        left_wheel_velocity = max_wheel_velocity
        right_wheel_velocity = -max_wheel_velocity
    elif ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
        left_wheel_velocity = -max_wheel_velocity
        right_wheel_velocity = -max_wheel_velocity

    if ord('i') in keys and keys[ord('i')] & p.KEY_IS_DOWN:
        right_leg_position += 0.001
        left_leg_position += 0.001
    elif ord('k') in keys and keys[ord('k')] & p.KEY_IS_DOWN:
        right_leg_position -= 0.001
        left_leg_position -= 0.001
    if ord('o') in keys and keys[ord('o')] & p.KEY_IS_DOWN:
        right_leg_displacement += 0.001
        left_leg_displacement += 0.001
    elif ord('l') in keys and keys[ord('l')] & p.KEY_IS_DOWN:  # Changed from ':' to ';'
        right_leg_displacement -= 0.001
        left_leg_displacement -= 0.001

    if left_leg_position > max_height:
        left_leg_position = max_height
    if right_leg_position > max_height:
        right_leg_position = max_height

    if (left_leg_displacement > max_displacement):
        left_leg_displacement = max_displacement
    elif (left_leg_displacement < -max_displacement):
        left_leg_displacement = -max_displacement

    if (right_leg_displacement > max_displacement):
        right_leg_displacement = max_displacement
    elif (right_leg_displacement < -max_displacement):
        right_leg_displacement = -max_displacement

    # Set wheel velocities
    p.setJointMotorControl2(robot_id, left_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=left_wheel_velocity)
    p.setJointMotorControl2(robot_id, right_wheel_joint_id, p.VELOCITY_CONTROL, targetVelocity=right_wheel_velocity)

    # Calculate knee and hip angles using inverse kinematics
    left_hip, left_knee = inverse_kinematics(left_leg_position, left_leg_displacement)
    right_hip, right_knee = inverse_kinematics(right_leg_position, right_leg_displacement)

    # Set knee and hip joint positions
    p.setJointMotorControl2(robot_id, left_knee_joint_id, p.POSITION_CONTROL, targetPosition=left_knee, force = 10.0, maxVelocity = 2.0)
    p.setJointMotorControl2(robot_id, left_hip_joint_id, p.POSITION_CONTROL, targetPosition=left_hip, force = 10.0, maxVelocity = 2.0)
    p.setJointMotorControl2(robot_id, right_knee_joint_id, p.POSITION_CONTROL, targetPosition=right_knee, force = 10.0, maxVelocity = 2.0)
    p.setJointMotorControl2(robot_id, right_hip_joint_id, p.POSITION_CONTROL, targetPosition=right_hip, force = 10.0, maxVelocity = 2.0)
    

# def inverse_kinematics(f=0, x=0):
#     """
#     Performs inverse kinematics calculations to get knee and hip angles.
    
#     Args:
#         f: The target foot position (height).
#         x: The x-axis displacement (currently unused).
        
#     Returns:
#         A tuple (knee_theta, hip_theta) representing the joint angles.
#     """
#     L1 = L2 = 0.1
#     if f > 0:
#         knee_theta = (np.pi / 3) + np.arccos((L1**2 + L2**2 - f**2) / (2 * L1 * L2))
#         hip_theta = -np.arccos((L1**2 + f**2 - L2**2) / (2 * L1 * f))
#         return knee_theta, hip_theta
#     else:
#         return 0,0

def inverse_kinematics(height=0, displacement=0):
    if height != 0:
        L1 = L2 = 0.1
        knee_theta = (np.pi / 3) + np.arccos((L1**2 + L2**2 - height**2) / (2 * L1 * L2))
        hip_theta = np.arcsin(displacement / height) - np.arccos((L1**2 + height**2 - L2**2) / (2 * L1 * height)) 

        return hip_theta, knee_theta
    else:
        return 0, 0

# Example usage
if __name__ == "__main__":
    # Initialize leg positions
    left_leg_position = 0.0
    right_leg_position = 0.0
    left_leg_displacement = 0.0
    right_leg_displacement = 0.0

    # Connect to PyBullet and create a GUI window
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Set Gravity and load Plane
    p.setGravity(0, 0, -9.81)
    global planeId 
    planeID = p.loadURDF("plane.urdf")

    # Load the robot URDF file
    robot_id = p.loadURDF("model/werdna_stand_bullet.urdf", useFixedBase=False, basePosition=[0, 0, 0.15])

    # Extract joint information
    joint_info = extract_joint_info(robot_id)

    # Run the simulation with teleop control
    while True:
        p.stepSimulation()
        teleop()
        position, orientation= p.getBasePositionAndOrientation(robot_id)
        print(position[0])
        time.sleep(1./240.)  # Set the simulation to run at 240hz for more precise control
