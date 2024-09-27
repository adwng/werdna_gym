import os
from stable_baselines3 import PPO, DQN
from utils import parser as p
from env.werdna_balance import WerdnaEnv
from env.werdna_stand import WerdnaStandEnv

def main():

    config = p.parser("config/config.yaml")

    robot_model = config['robot_model']
    connect_type = config['connect_type']
    algo = config['algo']
    complete_filename = os.path.join('results',config['filename'])

    # Print each variable
    print(f"Robot Model: {robot_model}")
    print(f"Connect Type: {connect_type}")
    print(f'Algorithm Used: {algo}')
    print(f"Zip File: {complete_filename}")

    env = WerdnaStandEnv(modelType=robot_model, render_mode=connect_type)

    if algo == 'DQN':
        model=DQN.load(complete_filename)
    elif algo == 'PPO':
        model = PPO.load(complete_filename) 

    # Set the environment for the model
    model.set_env(env)

    # Number of episodes to test
    num_episodes = 3000

    total_reward = 0

    obs ,_= env.reset()  # Reset the environment

    for episode in range(num_episodes):
        
        action, _ = model.predict(obs)  # Get the action from the model
        obs, reward, truncated, terminated, info = env.step(action)  # Take the action in the environment
        total_reward += reward  # Accumulate reward

    print(f"Episode {episode + 1} finished with total reward: {total_reward}")

    # Close the environment if it supports closing
    env.close()

if __name__ =="__main__":
    main()
