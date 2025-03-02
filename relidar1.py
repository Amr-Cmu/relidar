import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')
        
        # Subscriber to the 'new_scan' topic
        self.subscription_lidar = self.create_subscription(
            LaserScan,
            'new_scan',
            self.scan_callback,
            10
        )
        
        # Subscribers to ultrasonic topics
        self.subscription_ultra1 = self.create_subscription(
            Float32,
            'Ultra_state1',
            self.ultra1_callback,
            10
        )
        
        self.subscription_ultra2 = self.create_subscription(
            Float32,
            'Ultra_state2',
            self.ultra2_callback,
            10
        )
        
        # Publisher to the 'scan' topic
        self.publisher = self.create_publisher(
            LaserScan,
            'scan',
            10
        )
        
        # Ultrasonic sensor values
        self.ultra1_value = np.inf  # Left ultrasonic
        self.ultra2_value = np.inf  # Right ultrasonic
        
        # Minimum and maximum range filtering
        self.min_ranges_scan = 0.50
        self.max_ranges_scan = 100.0
        
        # Ultrasonic sensor positions relative to Lidar
        self.ultra_offset_x = 0.25  # 25 cm in front
        self.ultra_offset_y = 0.22  # 22 cm left/right
        
        self.get_logger().info('Lidar Filter Node has been started.')

    def ultra1_callback(self, msg):
        self.ultra1_value = msg.data if msg.data > 0 else np.inf

    def ultra2_callback(self, msg):
        self.ultra2_value = msg.data if msg.data > 0 else np.inf

    def scan_callback(self, msg):
        range_new = np.array(msg.ranges)
        
        # Replace values less than min_ranges_scan with infinity
        range_new[range_new < self.min_ranges_scan] = np.inf
        
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        
        # Calculate indices for left and right ultrasonic data
        left_index = int((np.arctan2(self.ultra_offset_y, self.ultra_offset_x) - angle_min) / angle_increment)
        right_index = int((np.arctan2(-self.ultra_offset_y, self.ultra_offset_x) - angle_min) / angle_increment)
        
        # Update Lidar ranges with ultrasonic values
        if 0 <= left_index < len(range_new):
            range_new[left_index] = min(range_new[left_index], self.ultra1_value)
        
        if 0 <= right_index < len(range_new):
            range_new[right_index] = min(range_new[right_index], self.ultra2_value)
        
        # Create new LaserScan message
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