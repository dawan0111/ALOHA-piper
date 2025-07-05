import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from act_environment.envs.act_environment import DualArmPiperEnv
import threading
import time

class EnvMainNode(Node):
    def __init__(self):
        super().__init__("dual_arm_piper_env_node")
        self.env = DualArmPiperEnv(node=self)
        self.timestep = self.env.reset()
        self._running = True
        self._step_thread = threading.Thread(target=self._run_loop)
        self._step_thread.start()

    def _run_loop(self):
        while rclpy.ok() and self._running:
            try:
                action = [0.0] * self.env.robot.action_size
                self.timestep = self.env.step(action)
                self.get_logger().info(f"[STEP] {self.timestep.step_type}, Reward: {self.timestep.reward}")
            except Exception as e:
                self.get_logger().error(f"[STEP ERROR] {e}")

    def get_env(self):
        return self.env

    def stop(self):
        self._running = False
        self._step_thread.join()


def main(args=None):
    rclpy.init(args=args)

    node = EnvMainNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down environment node...")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
