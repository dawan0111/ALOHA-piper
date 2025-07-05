import abc
import time
import numpy as np
import collections
import matplotlib.pyplot as plt
import dm_env
from typing import Dict, Any

class Envionrment:
    """
    Base class for environments in the act_environment package.
    This class provides a template for creating environments that interact with robots and observers.
    It defines the basic structure for resetting the environment, stepping through it, and obtaining observations and rewards.
    """
    def __init__(self, robot, observer, DT=0.02):
        self.robot = robot
        self.observer = observer
        self.DT = DT

    def reset(self):
        self.robot.reset()
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=0.0,
            discount=None,
            observation=self.get_observation())
     
    def step(self, action: np.array) -> dm_env.TimeStep:
        self.robot.apply_action(action)

        time.sleep(self.DT)

        obs = self.get_observation()
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(obs),
            discount=None,
            observation=obs)

    def get_observation(self) -> Dict[str, Any]:
        obs = self.observer.get_observation()
        return obs

    def get_reward(self, obs: Dict[str, Any]) -> float:
        return 0.0

    def check_termination(self, obs: Dict[str, Any]) -> dm_env.StepType:
        return dm_env.StepType.MID
