import numpy as np
import matplotlib.pyplot as plt

# Constants
L1 = L2 = 0.1

def inverse_kinematics(height=0, displacement=0):
    if height != 0:
        L1 = L2 = 0.1
        knee_theta =  np.arccos((L1**2 + L2**2 - height**2) / (2 * L1 * L2))
        hip_theta = np.arcsin(displacement / height) - np.arccos((L1**2 + height**2 - L2**2) / (2 * L1 * height)) 

        return hip_theta, knee_theta
    else:
        return 0, 0

def rad2deg(theta):
    return theta * (180 / np.pi)

def interpolate(start, target, steps):
    return np.linspace(start, target, steps)

def main():
    height = float(input("Height (positive float, or negative to exit): "))
    displacement = float(input("Displacement (positive float): "))

    # Limit height and displacement to a maximum of 0.1
    height = min(height, 0.1)
    displacement = min(displacement, 0.1)

    angle_hip, angle_knee = inverse_kinematics(height, displacement)

    # Define target positions
    target_positions = [angle_hip, angle_knee, -angle_hip, -angle_knee]

    print("_____LEFT______")
    print(f'The angle for knee and hip are {angle_knee} and {angle_hip}')
    print("_____RIGHT______")
    print(f'The angle for knee and hip are {-angle_knee} and {-angle_hip}')

    # # Interpolation
    # current_positions = [0.3, 1.2, -0.3, 1.2]  # Assuming starting at 0 for all angles
    # steps = 100
    # interpolated_positions = []

    # for start, target in zip(current_positions, target_positions):
    #     interpolated_positions.append(interpolate(start, target, steps))

    # # Plotting
    # plt.figure(figsize=(10, 5))
    # for i, angles in enumerate(interpolated_positions):
    #     plt.plot(angles, label=f'Target Angle {i+1}')

    # plt.title('Angle Interpolation')
    # plt.xlabel('Steps')
    # plt.ylabel('Angle (radians)')
    # plt.legend()
    # plt.grid()
    # plt.show()

if __name__ == "__main__":
    main()
