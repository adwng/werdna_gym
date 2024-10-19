import os
import numpy as np
import matplotlib.pyplot as plt
from sklearn.decomposition import KernelPCA
from stable_baselines3 import PPO
from env.werdna_balance import WerdnaEnv

def collect_observations_and_actions(model, env, n_steps=1000):
    """
    Collect observations and actions from the environment while using the trained model.
    """
    obs, _ = env.reset()
    observations = []
    actions = []
    
    for step in range(n_steps):
        action, _ = model.predict(obs)
        obs, reward, truncated, terminated, info = env.step(action)
        
        observations.append(obs)
        actions.append(action)
        
        if terminated or truncated:
            obs, _ = env.reset()

    return np.array(observations), np.array(actions)

def apply_polynomial_kpca(data, n_components=2, degree=3):
    """
    Apply Polynomial Kernel PCA on the provided data.
    """
    kpca = KernelPCA(n_components=n_components, kernel='poly', degree=degree)
    reduced_data = kpca.fit_transform(data)
    return reduced_data

def plot_kpca_results(reduced_observations, reduced_actions):
    """
    Plot the Kernel PCA results for observations and actions in distinct graphs.
    """
    plt.figure(figsize=(12, 6))

    # Plot for Observations
    plt.subplot(1, 2, 1)
    plt.scatter(reduced_observations[:, 0], reduced_observations[:, 1], c='blue', alpha=0.5)
    plt.title("Polynomial KPCA of Observations")
    plt.xlabel("Component 1")
    plt.ylabel("Component 2")

    # Plot for Actions
    plt.subplot(1, 2, 2)
    plt.scatter(reduced_actions[:, 0], reduced_actions[:, 1], c='orange', alpha=0.5)
    plt.title("Polynomial KPCA of Actions")
    plt.xlabel("Component 1")
    plt.ylabel("Component 2")

    plt.tight_layout()
    plt.show()

def plot_raw_data(observations, actions):
    """
    Plot raw observations and actions in distinct graphs.
    """
    plt.figure(figsize=(12, 12))

    # Plot Observations
    plt.subplot(2, 2, 1)
    plt.plot(observations[:, 0], label='Pitch', color='blue')
    plt.title("Pitch")
    plt.xlabel("Time Steps")
    plt.ylabel("Value")

    plt.subplot(2, 2, 2)
    plt.plot(observations[:, 1], label='Pitch Rate', color='orange')
    plt.title("Pitch Rate")
    plt.xlabel("Time Steps")
    plt.ylabel("Value")

    plt.subplot(2, 2, 3)
    plt.plot(observations[:, 2], label='Position', color='green')
    plt.title("Position")
    plt.xlabel("Time Steps")
    plt.ylabel("Value")

    plt.subplot(2, 2, 4)
    plt.plot(observations[:, 3], label='Velocity', color='red')
    plt.title("Velocity")
    plt.xlabel("Time Steps")
    plt.ylabel("Value")

    plt.tight_layout()
    plt.show()

    # Plot Actions
    plt.figure(figsize=(12, 6))

    plt.subplot(2, 1, 1)
    plt.plot(actions[:, 0]*10, label='Wheel_Velocity', color='purple')
    plt.title("Scaled Action: Wheel Velocity")
    plt.xlabel("Time Steps")
    plt.ylabel("Value (scaled to [-10, 10])")

    plt.subplot(2, 1, 2)
    plt.plot(actions[:, 1]*np.pi/6, label='Hip Rotation', color='magenta')
    plt.title("Scaled Action: Rotation")
    plt.xlabel("Time Steps")
    plt.ylabel("Value (scaled to [-π/6, π/6])")

    plt.tight_layout()
    plt.show()

def main():
    # Load the configuration and model as usual
    config = {"algo": "PPO", "filename": "ppo_werdna_stand_balance-v4.zip", "env": "werdna_balance_v2"}
    robot_model = "/model/werdna_stand_bullet.urdf"  # Replace with actual model type
    complete_filename = os.path.join('results', config['filename'])
    
    # Initialize the environment
    env = WerdnaEnv(modelType=robot_model, render_mode='GUI')
    
    # Load the trained model
    if config['algo'] == 'PPO':
        model = PPO.load(complete_filename)
    
    # Set the environment for the model
    model.set_env(env)
    
    # Collect observations and actions
    observations, actions = collect_observations_and_actions(model, env)
    
    # Apply Polynomial KPCA
    reduced_observations = apply_polynomial_kpca(observations, n_components=2, degree=3)
    reduced_actions = apply_polynomial_kpca(actions, n_components=2, degree=3)
    
    # Plot the reduced data using KPCA
    plot_kpca_results(reduced_observations, reduced_actions)
    
    # Plot raw observations and actions
    plot_raw_data(observations, actions)
    
    # Close the environment
    env.close()

if __name__ == "__main__":
    main()
