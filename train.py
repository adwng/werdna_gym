import os
import torch
import numpy as np

from stable_baselines3 import PPO, DDPG, DQN
from utils import parser as p
from env.werdna_balance import WerdnaEnv
from env.werdna_stand import WerdnaStandEnv
from env.werdna_legged import WerdnaLeggedEnv

def main():
    config = p.parser("config/config.yaml")

    robot_model = config['robot_model']
    connect_type = config['connect_type']
    algo = config['algo']
    filename = config['filename']
    env_name = config['env']
    total_timesteps =  1000000  

    # Print each variable
    print(f"Robot Model: {robot_model}")
    print(f"Connect Type: {connect_type}")
    print(f'Algorithm Used: {algo}')
    print(f"Zip File: {filename}")
    print(f"Environment: {env_name}")

    # Specify the device as "cuda" (GPU) if available, or fall back to "cpu"
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")
    
    if device == torch.device('cuda'):
        print(f"Training on GPU: {torch.cuda.get_device_name(0)}")
    else:
        print("Training on CPU")

    # Choose the environment
    if env_name == 'werdna_balance':
        env = WerdnaEnv(modelType=robot_model, render_mode=connect_type)
    elif env_name == 'werdna_stand':
        env = WerdnaStandEnv(modelType=robot_model, render_mode=connect_type)
    elif env_name == 'werdna_legged':
        env = WerdnaLeggedEnv(modelType=robot_model, render_mode=connect_type)
    else:
        raise ValueError(f"Unknown environment: {env_name}")

    # Algorithm selection

    if algo == 'PPO':
        model = PPO("MlpPolicy", env, verbose=1, device=device)
        model.learn(total_timesteps=total_timesteps)

    elif algo == 'DDPG':
        from stable_baselines3.common.noise import NormalActionNoise
        n_actions = env.action_space.shape[-1]
        action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
        model = DDPG("MlpPolicy", env, action_noise=action_noise, verbose=1, device=device)
        model.learn(total_timesteps=total_timesteps, log_interval=10)

    else:
        raise ValueError(f"Unknown algorithm: {algo}")

    # Create the results directory if it doesn't exist
    results_dir = "results"
    os.makedirs(results_dir, exist_ok=True)

    # Save the model
    print(f"Saving model to {os.path.join(results_dir, filename)}")
    model.save(os.path.join(results_dir, filename))

    env.close()

if __name__ == "__main__":
    main()
