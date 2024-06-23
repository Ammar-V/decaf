import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

from .detect import detect

class DetectObject(Node):

    def __init__(self):
        super().__init__('detect_object')

        self.get_logger().info("Looking for object...")
        
        self.image_sub = self.create_subscription(Image, "/image_in", self.callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)

        self.point_pub = self.create_publisher(Point, '/detected_object', 10)

        self.bridge = CvBridge()

    def callback(self, data):

        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        try:
            # Process image
            out_img, (x_scaled, y_scaled, z_scaled) = detect(cv_img)

            img_to_pub = self.bridge.cv2_to_imgmsg(out_img, 'bgr8')
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)

            if z_scaled > 0:
                pt = Point()
                pt.x = x_scaled
                pt.y = y_scaled
                pt.z = z_scaled

                self.point_pub.publish(pt)

        except CvBridgeError as e:
            print(e)

def main(args=None):

    rclpy.init(args=args)

    detect_object = DetectObject()
    while rclpy.ok():
        rclpy.spin_once(detect_object)

    detect_object.destroy_node()
    rclpy.shutdown()
