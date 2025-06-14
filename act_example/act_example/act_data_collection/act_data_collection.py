import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        
        # Declare parameter for duration
        self.declare_parameter('record_duration', 10.0)
        self.record_client = self.create_client(Empty, 'start_record')

        while not self.record_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for record service...')

    def call_record_service(self):
        req = Empty.Request()
        future = self.record_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Record service call succeeded.')
        else:
            self.get_logger().error('Record service call failed.')

    def collect_once(self):
        duration = self.get_parameter('record_duration').get_parameter_value().double_value
        self.get_logger().info(f"Starting recording for {duration} seconds...")

        self.call_record_service()  # Start
        time.sleep(duration)
        self.call_record_service()  # Stop

        self.get_logger().info("Recording stopped.")

def main():
    rclpy.init()
    node = DataCollector()

    try:
        while True:
            user_input = input("Press Enter to start recording or 'q' to quit: ")
            if user_input.lower() == 'q':
                break
            node.collect_once()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
