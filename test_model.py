import os
import time
from stable_baselines3 import PPO, DQN, DDPG

from utils import parser as p
from env.werdna_balance import WerdnaEnv
from env.werdna_stand import WerdnaStandEnv
from env.werdna_legged import WerdnaLeggedEnv

def main():

    total_reward = 0

    config = p.parser("config/config.yaml")

    robot_model = config['robot_model']
    connect_type = config['connect_type']
    algo = config['algo']
    complete_filename = os.path.join('results',config['filename'])
    env_name = config['env']

    # Print each variable
    print(f"Robot Model: {robot_model}")
    print(f"Connect Type: {connect_type}")
    print(f'Algorithm Used: {algo}')
    print(f"Environment: {env_name}")
    print(f"Zip File: {complete_filename}")

    if env_name == 'werdna_balance':
        env = WerdnaEnv(modelType=robot_model, render_mode='GUI')
    elif env_name == 'werdna_stand':
        env = WerdnaStandEnv(modelType=robot_model, render_mode='GUI')
    elif env_name == 'werdna_legged':
        env = WerdnaLeggedEnv(modelType=robot_model, render_mode='GUI')

    if algo == 'PPO':
        model = PPO.load(complete_filename) 
    elif algo == 'DDPG':
        model = DDPG.load(complete_filename)

    # Set the environment for the model
    model.set_env(env)

    # Number of episodes to test
    # num_episodes = 3000

    # total_reward = 0

    # obs ,_= env.reset()  # Reset the environment

    # for episode in range(num_episodes):
        
    #     action, _ = model.predict(obs)  # Get the action from the model
    #     obs, reward, truncated, terminated, info = env.step(action)  # Take the action in the environment
    #     total_reward += reward  # Accumulate reward

    # print(f"Episode {episode + 1} finished with total reward: {total_reward}")

    # # Close the environment if it supports closing
    # env.close()

    while 1:
        frame = 0
        terminated = False
        obs, _ = env.reset() # Reset the environment

        while not (terminated):

            time.sleep(1./60.)

            action, _ = model.predict(obs)
            obs, reward, truncated, terminated, info = env.step(action)

            total_reward += reward
            frame += 1

            print(f"Episode {frame} finished with total reward: {total_reward}")

        env.close()

if __name__ =="__main__":
    main()
