from stable_baselines3.common.env_checker import check_env
from env.werdna_balance import WerdnaEnv
from env.werdna_stand import WerdnaStandEnv

# env = WerdnaEnv(render_mode="GUI", modelType="model/werdna_bullet.urdf")
env = WerdnaStandEnv(render_mode="GUI", modelType="model/werdna_bullet.urdf")

# It will check your custom environment and output additional warnings if needed
check_env(env)