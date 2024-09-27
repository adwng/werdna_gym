from utils import parser as p
from env.werdna_balance import WerdnaEnv
from env.werdna_stand import WerdnaStandEnv
import os

from stable_baselines3 import PPO, DDPG,DQN


def main():

    config = p.parser("config/config.yaml")

    robot_model = config['robot_model']
    connect_type = config['connect_type']
    algo = config['algo']
    filename = config['filename']

    # Print each variable
    print(f"Robot Model: {robot_model}")
    print(f"Connect Type: {connect_type}")
    print(f'Algorithm Used: {algo}')
    print(f"Zip File: {filename}")

    env = WerdnaStandEnv(modelType=robot_model, render_mode=connect_type)

    if algo == 'DQN':
        model=DQN("MlpPolicy", env, verbose=1)
        model.learn(total_timesteps= 1000000)
    elif algo == 'PPO':
        model = PPO("MlpPolicy", env, verbose=1)
        model.learn(total_timesteps= 1000000, log_interval=4)

    # Create the results directory if it doesn't exist
    results_dir = "results"
    os.makedirs(results_dir, exist_ok=True)

    # Save the model
    model.save(os.path.join(results_dir, filename))

    env.close()

if __name__ == "__main__":
    main()