import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge, CvBridgeError

from .cv_utils.lanes import find_lanes, project_lanes

import numpy as np

class DetectLanes(Node):

    def __init__(self):
        super().__init__('detect_lanes')

        self.get_logger().info("Looking for lanes...")
        
        self.image_sub = self.create_subscription(Image, "/image_in", self.callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(Image, "/lane_mask", 1)

        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/camera_info", self.set_camera_info, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.depth_sub = self.create_subscription(Image, "/camera/depth/image_raw", self.set_depth_map, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)

        self.projected_lanes_pub = self.create_publisher(PointCloud2, "/projected_lanes", 10)


        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_map = None
    
    def set_camera_info(self, data):
        
        self.camera_info = {
            'fx': data.k[0],
            'fy': data.k[4],
            'cx': data.k[2],
            'cy': data.k[5],
        }

    def set_depth_map(self, data):
        try:
            self.depth_map = self.bridge.imgmsg_to_cv2(data, 'passthrough')
        except CvBridgeError as e:
            print(e)
            self.depth_map = None
            print("Seting depth map to None...")
        

    def callback(self, data):

        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        try:
            # Process image
            out_img = find_lanes(cv_img)

            img_to_pub = self.bridge.cv2_to_imgmsg(out_img, 'mono8')
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)

            # If depth map is available, project lanes
            if self.depth_map is not None and self.camera_info is not None:
                lanes_pc = project_lanes(out_img, self.depth_map, self.camera_info).tolist()

                # Create the point cloud fields
                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                ]

                # Create the PointCloud2 message
                projected_lanes_msg = point_cloud2.create_cloud(data.header, fields, lanes_pc)

                self.projected_lanes_pub.publish(projected_lanes_msg)


        except CvBridgeError as e:
            print(e)

def main(args=None):

    rclpy.init(args=args)

    detect_lanes = DetectLanes()
    while rclpy.ok():
        rclpy.spin_once(detect_lanes)

    detect_lanes.destroy_node()
    rclpy.shutdown()
