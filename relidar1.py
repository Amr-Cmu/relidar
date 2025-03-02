import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')

        # Subscriber to the 'new_scan' topic (Lidar)
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'new_scan',
            self.lidar_scan_callback,
            10
        )

        # Subscriber to the 'Ultra_state1' topic (Ultrasonic left)
        self.ultra_state1_subscription = self.create_subscription(
            Float32,
            'Ultra_state1',
            self.ultra_state1_callback,
            10
        )

        # Subscriber to the 'Ultra_state2' topic (Ultrasonic right)
        self.ultra_state2_subscription = self.create_subscription(
            Float32,
            'Ultra_state2',
            self.ultra_state2_callback,
            10
        )

        # Publisher to the 'scan' topic (Filtered Scan)
        self.publisher = self.create_publisher(
            LaserScan,
            'scan',
            10
        )

        # Minimum range for filtering (start scanning from 0.5 meter)
        self.min_ranges_scan = 0.50
        self.max_ranges_scan = 100.0

        # Variables to store ultrasonic sensor data
        self.ultra_state1 = None
        self.ultra_state2 = None

        self.get_logger().info('Lidar Filter Node has been started.')

    def ultra_state1_callback(self, msg):
        self.ultra_state1 = msg.data
        self.get_logger().debug(f'Received Ultra_state1 data: {self.ultra_state1}')

    def ultra_state2_callback(self, msg):
        self.ultra_state2 = msg.data
        self.get_logger().debug(f'Received Ultra_state2 data: {self.ultra_state2}')

    def lidar_scan_callback(self, msg):
        # Convert ranges to a NumPy array
        range_new = np.array(msg.ranges)
        
        # Replace values less than 0.5 meters with infinity
        range_new[range_new < self.min_ranges_scan] = np.inf

        # Incorporate ultrasonic sensor data for left and right
        if self.ultra_state1 is not None:
            index_left = int(len(range_new) * 0.25)  
            range_new[index_left] = min(self.ultra_state1, range_new[index_left])

        if self.ultra_state2 is not None:
            # Place the right ultrasonic reading into the range data (adjust position accordingly)
            # Ultrasonic sensor 2 is 22 cm right and 25 cm in front of the lidar
            index_right = int(len(range_new) * 0.75)  
            range_new[index_right] = min(self.ultra_state2, range_new[index_right])

        # Update the filtered ranges back to the message
        filtered_msg = msg
        filtered_msg.ranges = range_new.tolist()

        # Publish the filtered message
        self.publisher.publish(filtered_msg)

        self.get_logger().debug('Published filtered LaserScan message.')

def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# import numpy as np

# class LidarFilterNode(Node):
#     def __init__(self):
#         super().__init__('lidar_filter_node')
        
#         # Subscriber to the 'scan' topic
#         self.subscription = self.create_subscription(
#             LaserScan,
#             'new_scan',
#             self.scan_callback,
#             10
#         )

#         # Publisher to the 'filter_lidar' topic
#         self.publisher = self.create_publisher(
#             LaserScan,
#             'scan',
#             10
#         )

#         # Minimum range for filtering (start scanning from 1 meter)
#         self.min_ranges_scan = 0.50
#         self.max_ranges_scan = 100.0

#         self.get_logger().info('Lidar Filter Node has been started.')

#     def scan_callback(self, msg):
#         # Convert ranges to a NumPy array
#         range_new = np.array(msg.ranges)
        
#         # # Replace values less than 1 meter with infinity
#         range_new[range_new < self.min_ranges_scan] = np.inf
        
#         # Update the filtered ranges back to the message
#         filtered_msg = msg
#         filtered_msg.ranges = range_new.tolist()

#         # Publish the filtered message
#         self.publisher.publish(filtered_msg)

#         self.get_logger().debug('Published filtered LaserScan message.')

# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarFilterNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Node stopped by user.')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()