import numpy as np
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from act_environment.envs.robots._robot import RobotInterface
from act_environment.envs.robots._action_type import ActionType

class DualPiperRobot(RobotInterface):
    def __init__(self, node: Node):
        super().__init__(node)

        self.action_size = 14  # 7 joints for left arm + 7 joints for right arm

        self.left_pub = node.create_publisher(JointState, '/left_arm/joint_ctrl_single', 1)
        self.right_pub = node.create_publisher(JointState, '/right_arm/joint_ctrl_single', 1)

        self.action_type = ActionType.POSITION
        self.gripper_min = 0.0
        self.gripper_max = 0.07

    def set_action_type(self, action_type: ActionType):
        self.action_type = action_type

    def set_gripper_range(self, gripper_min: float, gripper_max: float):
        self.gripper_min = gripper_min
        self.gripper_max = gripper_max

    def denormalize_gripper(self, value: float) -> float:
        return np.clip(value, 0.0, 1.0) * (self.gripper_max - self.gripper_min) + self.gripper_min

    def apply_action(self, action: np.ndarray):
        assert action.shape[-1] == 14, f"Expected action shape (..., 14), got {action.shape}"
        left_cmd = action[:7].copy()
        right_cmd = action[7:].copy()

        # Gripper denormalization
        left_cmd[6] = self.denormalize_gripper(left_cmd[6])
        right_cmd[6] = self.denormalize_gripper(right_cmd[6])

        print("left gripper:", left_cmd[6])
        print("right gripper:", right_cmd[6])

        msg_left = self._create_joint_msg(left_cmd)
        msg_right = self._create_joint_msg(right_cmd)

        if self.action_type == ActionType.POSITION:
            self.left_pub.publish(msg_left)
            self.right_pub.publish(msg_right)
        else:
            raise NotImplementedError(f"{self.action_type} not supported yet for DualPiperRobot")

    def _create_joint_msg(self, joint_values: np.ndarray) -> JointState:
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [f"joint{i+1}" for i in range(6)] + ["gripper"]

        if self.action_type == ActionType.POSITION:
            msg.position = joint_values.tolist()
        elif self.action_type == ActionType.VELOCITY:
            msg.velocity = joint_values.tolist()
        elif self.action_type == ActionType.TORQUE:
            msg.effort = joint_values.tolist()
        else:
            raise ValueError(f"Unsupported ActionType: {self.action_type}")
        return msg

    def reset(self):
        pos1 = [0.0, 1.0, -1.25, 0.00, 0.75, 0.00, 0.0]
        pos2 = [0.0, 1.0, -1.25, 0.00, 0.75, 0.00, 0.0]

        self.apply_action(np.array(pos1 + pos2, dtype=np.float32))  # Reset to zero position
        return None