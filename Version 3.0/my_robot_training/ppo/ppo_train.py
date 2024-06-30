import gym
import rospy
import rospkg

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.env_checker import check_env
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment
from gym.envs.registration import register
from openai_ros.task_envs.parking import parking
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest

LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/parking/config",
                               yaml_file_name="parking.yaml")

ros_ws_abspath = rospy.get_param("/parking/ros_ws_abspath", None)

register(
    id="Parking-v0",
    entry_point='openai_ros.task_envs.parking.parking:ParkingTaskENV',
    max_episode_steps=10000,
)

rospy.init_node("stay_up", anonymous=True, log_level=rospy.FATAL)
env = StartOpenAI_ROS_Environment("Parking-v0")
env.reset()

# Parallel environments
#env = make_vec_env("Parking-v0", n_envs=1)

check_env(env)

#rospy.loginfo("test")
#print("test2")

model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=250000)
model.save("ppo_cartpole")

#del model # remove to demonstrate saving and loading

#model = PPO.load("ppo_cartpole")

#obs = env.reset()
#while True:
#    action, _states = model.predict(obs)
#    obs, rewards, dones, info = env.step(action)
#    env.render()