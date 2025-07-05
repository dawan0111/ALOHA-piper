from act_environment.envs._environment import Envionrment
from act_environment.envs.robots.dual_piper_robot import DualPiperRobot
from act_environment.envs.observers.aloha_observer import AlohaObserver

class DualArmPiperEnv(Envionrment):
    def __init__(self, node, dt=0.02):
        """
        DualArmPiperEnv

        Args:
            node: rclpy Node
            dt: timestep duration in seconds
        """
        robot = DualPiperRobot(node=node)
        observer = AlohaObserver(node=node)
        super().__init__(robot=robot, observer=observer, DT=dt)