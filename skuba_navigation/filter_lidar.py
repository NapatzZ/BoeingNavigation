import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from skuba_interfaces.srv import FilterLidar
import numpy as np
import copy

class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile_sensor_data
        )
        self.publisher_ = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.srv = self.create_service(
            FilterLidar,
            '/skuba/cheesecake/nav/filter',
            self.filter_service_callback
        )
        self.enable_angle_filter = False
        self.get_logger().info('Lidar Filter Service ready')

    def filter_service_callback(self, request, response):
        self.enable_angle_filter = request.enable
        if self.enable_angle_filter:
            response.current_mode = "angle filter  + distance filter"
        else:
            response.current_mode = "distance filter only"
        response.success = True
        self.get_logger().info(f'Filter mode changed: {response.current_mode}')
        return response

    def listener_callback(self, msg):
        filtered_msg = copy.deepcopy(msg)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        angle_start_1 = np.deg2rad(260)
        angle_end_1 = 2 * np.pi
        angle_start_2 = 0.0
        angle_end_2 = np.deg2rad(100)

        for i in range(len(filtered_msg.ranges)):
            current_range = filtered_msg.ranges[i]
            if np.isinf(current_range) or np.isnan(current_range):
                continue
            angle = angle_min + i * angle_increment
            angle = angle % (2 * np.pi)
            if current_range < 0.5:
                filtered_msg.ranges[i] = np.inf
                continue
            if self.enable_angle_filter:
                in_front_sector = (
                    (angle_start_1 <= angle <= angle_end_1) or
                    (angle_start_2 <= angle <= angle_end_2)
                )
                if in_front_sector:
                    filtered_msg.ranges[i] = np.inf

        self.publisher_.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
