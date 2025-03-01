import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')
        
        # Subscriber to the 'scan' topic
        self.subscription = self.create_subscription(
            LaserScan,
            'new_scan',
            self.scan_callback,
            10
        )

        # Subscriber to 'Ultra_state1' and 'Ultra_state2'
        self.ultra_sub1 = self.create_subscription(
            LaserScan,
            'Ultra_state1',
            self.ultra_callback1,
            10
        )
        self.ultra_sub2 = self.create_subscription(
            LaserScan,
            'Ultra_state2',
            self.ultra_callback2,
            10
        )

        # Publisher to the 'filter_lidar' topic
        self.publisher = self.create_publisher(
            LaserScan,
            'scan',
            10
        )

        # Minimum range for filtering (start scanning from 1 meter)
        self.min_ranges_scan = 0.50
        self.max_ranges_scan = 100.0

        # Store ultrasonic values
        self.ultra1_range = None
        self.ultra2_range = None

        self.get_logger().info('Lidar Filter Node has been started.')

    def ultra_callback1(self, msg):
        if 0.2 <= msg.ranges[0] <= 4.0:
            self.ultra1_range = msg.ranges[0]

    def ultra_callback2(self, msg):
        if 0.2 <= msg.ranges[0] <= 4.0:
            self.ultra2_range = msg.ranges[0]

    def scan_callback(self, msg):
        # Convert ranges to a NumPy array
        range_new = np.array(msg.ranges)
        
        # Replace values less than 1 meter with infinity
        range_new[range_new < self.min_ranges_scan] = np.inf
        
        # Integrate ultrasonic sensor data
        if self.ultra1_range is not None:
            range_new[0] = min(range_new[0], self.ultra1_range)
        if self.ultra2_range is not None:
            range_new[-1] = min(range_new[-1], self.ultra2_range)
        
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