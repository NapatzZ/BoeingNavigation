import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from skuba_interfaces.srv import FilterLidar
from ..core.lidar_filter import LidarFilter


class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self._filter = LidarFilter()
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile_sensor_data
        )
        self.publisher_ = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.srv = self.create_service(
            FilterLidar,
            '/skuba/nav/filter_lidar',
            self.filter_service_callback
        )
        self.get_logger().info('Lidar Filter Service ready')

    def filter_service_callback(self, request, response):
        response.current_mode = self._filter.set_mode(request.enable)
        response.success = True
        self.get_logger().info(f'Filter mode changed: {response.current_mode}')
        return response

    def listener_callback(self, msg):
        self.publisher_.publish(self._filter.filter_scan(msg))


def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
