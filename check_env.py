from stable_baselines3.common.env_checker import check_env
from env.werdna_balance import WerdnaEnv
from env.werdna_balance_v2 import Werdna2Env
from env.werdna_balance_v3 import Werdna3Env


# env = WerdnaEnv(render_mode="GUI", modelType="model/werdna_stand_bullet.urdf")
# env = Werdna2Env(render_mode="GUI", modelType="model/werdna_stand_bullet.urdf")
env = Werdna3Env(render_mode="GUI", modelType="model/werdna_v2_bullet.urdf")


# It will check your custom environment and output additional warnings if needed
check_env(env)