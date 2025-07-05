import abc
import numpy as np
from ._action_type import ActionType 

class RobotInterface(abc.ABC):
    """
    Abstract base class for robot controllers supporting multiple action modes.
    """

    def __init__(self, node):
        # Default to POSITION control
        self.node = node
        self.robot_action_size = 0
        self._action_type: ActionType = ActionType.POSITION

    def set_action_type(self, action_type: ActionType):
        """Set the current action type for control."""
        if not isinstance(action_type, ActionType):
            raise ValueError("Invalid action type. Must be an ActionType enum.")
        self._action_type = action_type

    def apply_action(self, action: np.ndarray):
        """
        Dispatch to the correct action function based on selected action_type.
        """
        if self._action_type == ActionType.POSITION:
            self.apply_joint_position(action)
        elif self._action_type == ActionType.VELOCITY:
            self.apply_joint_velocity(action)
        elif self._action_type == ActionType.TORQUE:
            self.apply_joint_torque(action)
        else:
            raise NotImplementedError(f"Action type {self._action_type} not implemented.")

    @abc.abstractmethod
    def reset(self) -> None:
        """Reset the robot to its initial state."""
        raise NotImplementedError("reset() must be overridden by subclass.")

    def apply_joint_position(self, action: np.ndarray):
        """Override to implement position control (optional)."""
        raise NotImplementedError("Position control not implemented.")

    def apply_joint_velocity(self, action: np.ndarray):
        """Override to implement velocity control (optional).") """
        raise NotImplementedError("Velocity control not implemented.")

    def apply_joint_torque(self, action: np.ndarray):
        """Override to implement torque control (optional)."""
        raise NotImplementedError("Torque control not implemented.")
