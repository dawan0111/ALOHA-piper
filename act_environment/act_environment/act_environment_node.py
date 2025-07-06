import rclpy
import numpy as np
import cv2
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
                pos = [1.0,0.0,0.0,0.0,0.0,0.0,0.0]

                action = pos + pos
                self.timestep = self.env.step(np.array(action))
                self.get_logger().info(f"[STEP] {self.timestep.step_type}, Reward: {self.timestep.reward}")
                images = self.timestep.observation['images']

                if images is not None:
                    # OpenCV images concat 2x2 and visualize
                    if 'cam_high' in images and 'cam_left_wrist' in images and 'cam_low' in images and 'cam_right_wrist' in images:
                        # Concatenate images
                        top_row = np.hstack((images['cam_high'], images['cam_left_wrist']))
                        bottom_row = np.hstack((images['cam_low'], images['cam_right_wrist']))
                        combined_image = np.vstack((top_row, bottom_row))
                        cv2.imshow("Dual Arm Piper Environment", combined_image)
                        cv2.waitKey(1)  # Update the window


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
