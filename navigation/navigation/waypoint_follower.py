import os
import sys
import time

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator
from robot_localization.srv import FromLL

from navigation.gps_utils import quaternion_from_euler

import yaml


class DecafWPF(Node):

    def __init__(self, wps_file_path):
        super().__init__('decaf_wpf')

        self.wps_file_path = wps_file_path

        self.from_ll_client = self.create_client(FromLL, '/fromLL')
        self.navigator = BasicNavigator("decaf_basic_navigator")

        self.gps_wps = []
        self.poses = []


    def set_pose_waypoints(self):
        self.get_logger().info("Starting to load GPS waypoints...")

        def read_gps_wps(wps_file_path):
            with open(wps_file_path, 'r') as wps_file:
                self.gps_wps = yaml.safe_load(wps_file)["waypoints"] # dict
            
        def request_ll_to_point(latitude, longitude, altitude):
            request = FromLL.Request()
            request.ll_point.latitude = latitude
            request.ll_point.longitude = longitude
            request.ll_point.altitude = altitude
            
            future = self.from_ll_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.result() is not None:
                print(future)
                return future.result().map_point
            else:
                self.get_logger().error('Service call failed')
                return None

        # Read the GPS waypoints from the yaml file
        read_gps_wps(self.wps_file_path)

        print(self.gps_wps)

        # Convert all the GPS lat, lon, alt values to UTM points
        for gps_wp in self.gps_wps:
            point = request_ll_to_point(gps_wp["lat"], gps_wp["lon"], gps_wp["alt"])

            point.x = point.x * -1
            point.y = point.y * -1

            print(point)
            quaternion = quaternion_from_euler(0, 0, gps_wp["yaw"])
   
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()  
            pose_stamped.pose.position = point
            pose_stamped.pose.orientation = quaternion

            self.poses.append(pose_stamped)

        assert len(self.gps_wps) == len(self.poses), "The number of GPS waypoints must equal to the number of PoseStamped waypoints!"

        self.get_logger().info("Finished loading GPS waypoints as PoseStamped messages...")

    def start_wpf(self):
        """
        Function to start the waypoint following
        """
        self.navigator.waitUntilNav2Active(localizer='global_costmap/global_costmap')
        
        # Get the pose waypoints
        while not self.from_ll_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /fromLL not available, waiting...')
        self.set_pose_waypoints()

        # Navigate through the poses
        self.navigator.followWaypoints(self.poses)
        while (not self.navigator.isTaskComplete()):
            time.sleep(0.1)
        print("wps completed successfully")


def main(args=None):
    # allow to pass the waypoints file as an argument
    default_yaml_file_path = os.path.join(get_package_share_directory(
        "navigation"), "config", "igvc_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    rclpy.init(args=args)

    decaf_wpf = DecafWPF(yaml_file_path)
    decaf_wpf.start_wpf()

    decaf_wpf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()