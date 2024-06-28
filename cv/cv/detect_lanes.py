import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

from .cv_utils.lanes import find_lanes

class DetectLanes(Node):

    def __init__(self):
        super().__init__('detect_lanes')

        self.get_logger().info("Looking for lanes...")
        
        self.image_sub = self.create_subscription(Image, "/image_in", self.callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(Image, "/lane_mask", 1)

        self.bridge = CvBridge()

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


        except CvBridgeError as e:
            print(e)

def main(args=None):

    rclpy.init(args=args)

    detect_lanes = DetectLanes()
    while rclpy.ok():
        rclpy.spin_once(detect_lanes)

    detect_lanes.destroy_node()
    rclpy.shutdown()
