import os
import torch
import numpy as np

from stable_baselines3 import PPO
from utils import parser as p
from env.werdna_balance import WerdnaEnv
from env.werdna_balance_v2 import Werdna2Env
from env.werdna_balance_v3 import Werdna3Env

def main():
    config = p.parser("config/config2.yaml")

    robot_model = config['robot_model']
    connect_type = config['connect_type']
    algo = config['algo']
    filename = config['filename']
    env_name = config['env']
    total_timesteps =  config['timesteps']
    tb_log_name = config['tb_log_name']

    # Print each variable
    print(f"Robot Model: {robot_model}")
    print(f"Connect Type: {connect_type}")
    print(f'Algorithm Used: {algo}')
    print(f"Zip File: {filename}")
    print(f"Environment: {env_name}")
    print(f"Total Timesteps{total_timesteps}")

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
    elif env_name == 'werdna_balance_v2':   
        env = Werdna2Env(modelType=robot_model, render_mode=connect_type)
    elif env_name == "werdna_balance_v3":
        env = Werdna3Env(modelType=robot_model, render_mode=connect_type)
    else:
        raise ValueError(f"Unknown environment: {env_name}")

    # Algorithm selection

    if algo == 'PPO':
        model = PPO("MlpPolicy", 
                    env, 
                    verbose=1,
                    device=device,
                    ent_coef=0.001,
                    # use_sde=True,
                    tensorboard_log='logs/ppo_werdna_v2_tensorboard'
                )
        model.learn(total_timesteps=total_timesteps, tb_log_name=tb_log_name)
    else:
        raise ValueError(f"Unknown algorithm: {algo}")

    # Create the results directory if it doesn't exist
    results_dir = os.path.join("results", tb_log_name)
    os.makedirs(results_dir, exist_ok=True)

    # Save the model
    print(f"Saving model to {os.path.join(results_dir, filename)}")
    model.save(os.path.join(results_dir, filename))

    env.close()

if __name__ == "__main__":
    main()
