import os
import time
import pybullet as p  # Add pybullet for video recording
from stable_baselines3 import PPO
import imageio  # Add imageio to combine images into a video
from utils import parser as pa
from env.werdna_balance import WerdnaEnv
from env.werdna_balance_v2 import Werdna2Env
from env.werdna_balance_v3 import Werdna3Env

def main():

    total_reward = 0

    # Parse the configuration file
    config = pa.parser("config/config2.yaml")

    # Extract settings from the config
    robot_model = config['robot_model']
    connect_type = config['connect_type']
    algo = config['algo']
    tb_log_name = config['tb_log_name']
    complete_filename = os.path.join('results', f'{tb_log_name}', config['filename'])
    env_name = config['env']
    record_video = config.get('record_video', False)  # Check if video recording is enabled

    # Print each variable
    print(f"Robot Model: {robot_model}")
    print(f"Connect Type: {connect_type}")
    print(f'Algorithm Used: {algo}')
    print(f"Environment: {env_name}")
    print(f"Zip File: {complete_filename}")
    print(f"Record Video: {record_video}")

    # Initialize the environment based on the provided configuration
    if env_name == 'werdna_balance':
        env = WerdnaEnv(modelType=robot_model, render_mode='GUI')
    elif env_name == 'werdna_balance_v2':
        env =  Werdna2Env(modelType=robot_model, render_mode='GUI')
    elif env_name == 'werdna_balance_v3':
        env = Werdna3Env(modelType=robot_model, render_mode="GUI")

    # Load the trained model
    if algo == 'PPO':
        model = PPO.load(complete_filename)

    # Set the environment for the model
    model.set_env(env)

    if record_video:
        # Create a folder to save the video and images
        video_folder = "video"
        image_folder = os.path.join(video_folder, "frames")
        if not os.path.exists(video_folder):
            os.makedirs(video_folder)
        if not os.path.exists(image_folder):
            os.makedirs(image_folder)

    # Keep running episodes
    episode_num = 0
    while 1:
        frame = 0
        terminated = False
        obs, _ = env.reset()  # Reset the environment
        total_reward = 0

        # Start video recording by saving frames if record_video is true
        episode_num += 1
        frame_filenames = []

        while not terminated or truncated:
            time.sleep(1. / 60.)

            # Get the predicted action from the model
            action, _ = model.predict(obs)

            # Take the action in the environment
            obs, reward, truncated, terminated, info = env.step(action)

            total_reward += reward
            frame += 1

            if record_video:
                # Capture the frame from the PyBullet window
                width, height, rgb_img, _, _ = p.getCameraImage(1920, 1080)  # Get frame from camera
                frame_filename = os.path.join(image_folder, f"episode_{episode_num}_frame_{frame}.png")
                frame_filenames.append(frame_filename)

                # Save the frame
                imageio.imwrite(frame_filename, rgb_img)

                print(f"Frame {frame} saved, total reward so far: {total_reward}")

        if record_video:
            # After the episode, combine the frames into a video
            video_filename = f"{complete_filename}.mp4"  # Name video as the complete_filename
            with imageio.get_writer(video_filename, fps=60) as video:
                for frame_filename in frame_filenames:
                    img = imageio.imread(frame_filename)
                    video.append_data(img)

            print(f"Episode {episode_num} finished, total reward: {total_reward}")
            print(f"Video saved to {video_filename}")

            # Clean up frame files after video creation
            for frame_filename in frame_filenames:
                os.remove(frame_filename)

        # Close the environment
        env.close()

if __name__ == "__main__":
    main()
