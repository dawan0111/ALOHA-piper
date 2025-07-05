import abc
from rclpy.node import Node
from typing import Dict, Any

class ObserverInterface(abc.ABC):
    """
    Abstract base class for observers that collect sensor or state information.
    Must be constructed with a ROS2 node context.
    """

    def __init__(self, node: Node):
        self.node = node

    @abc.abstractmethod
    def get_observation(self) -> Dict[str, Any]:
        """
        Return the current observation as a dictionary.
        Must be implemented by any subclass.
        """
        raise NotImplementedError("get_observation() must be implemented by subclass.")