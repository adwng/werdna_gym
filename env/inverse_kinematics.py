import numpy as np
import matplotlib.pyplot as plt

# # Constants
# L1 = L2 = 0.1

# def inverse_kinematics(x=0, y=0):
#     if y > 0:
#         L1 = L2 = 0.1

#         height = np.sqrt(x**2 + y**2)

#         knee_theta = np.arccos(
#             (L1*L1 + L2*L2 - height*height)/
#             ((2*L1 *L2))
#         )

#         hip_theta = np.arcsin(-x/y) + np.arccos((L1*L1 + height*height - L2* L2)/(2*L2*height))

#         return hip_theta, knee_theta
#     else:
#         return 0, 0

def inverse_kinematics(x=0, y=0):
    if y > 0:
        L1 = L2 = 0.1

        height = np.sqrt(x**2 + y**2)

        knee_theta = np.arccos(
            (L1*L1 + L2*L2 - height*height)/
            ((2*L1 *L2))
        )

        hip_theta = np.arcsin(-x/y) + np.arccos((L1*L1 + height*height - L2* L2)/(2*L2*height))

        return np.pi/5-hip_theta, -(np.pi/5 + knee_theta)
    else:
        return 0, 0
# FOR ROBOT, REMOVE OFFSET

def main():
    x = float(input("x (positive float): "))
    y = float(input("y (positive float): "))

    # Limit height and displacement to a maximum of 0.1
    x = min(x, 0.05)
    y = min(y, 0.1)

    angle_hip, angle_knee = inverse_kinematics(x, y)


    print("_____LEFT______")
    print(f'The angle for hip and knee are {angle_hip} and {angle_knee}')

if __name__ == "__main__":
    main()
