import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    
    # Convert the image to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_100)
    parameters =  aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, parameters)
    corners, markerIds, rejectedCandidates = detector.detectMarkers(gray)
    
    

def main():
    rclpy.init()
    node = rclpy.create_node('aruco_marker_detection')
    qos_profile = QoSProfile(depth=10)
    subscriber = node.create_subscription(Image, '/camera/camera/color/image_raw', image_callback, qos_profile)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
