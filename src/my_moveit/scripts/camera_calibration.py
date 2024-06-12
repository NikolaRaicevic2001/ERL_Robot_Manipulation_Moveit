import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, CameraInfo  # Add import for CameraInfo
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import transforms3d as tf2_geometry_msgs

# Global variable to store camera matrix and distortion coefficients
camera_matrix = None
dist_coeffs = None

def camera_info_callback(msg):
    global camera_matrix, dist_coeffs
    camera_matrix = np.array(msg.k).reshape((3, 3))
    dist_coeffs = np.array(msg.d)

def image_callback(msg):
    if camera_matrix is None or dist_coeffs is None:
        print("Camera calibration data not available!")
        return
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    
    # Convert the image to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
    # Define ArUco dictionary and parameters
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_100)
    parameters =  aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, markerIds, rejectedCandidates = detector.detectMarkers(gray)

    # Detect ArUco markers
    if markerIds is not None:
        for i in range(len(markerIds)):
            # Draw marker outlines
            cv2.aruco.drawDetectedMarkers(cv_image, corners, markerIds)
            
            # Draw ArUco ID
            org = tuple(corners[i][0][0].astype(int))  # Convert to tuple of integers
            cv2.putText(cv_image, str(markerIds[i]), org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Estimate pose of the marker
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, camera_matrix, dist_coeffs)
            
            # Convert rotation vector to rotation matrix
            rot_mat, _ = cv2.Rodrigues(rvec)

            # Construct transformation matrix
            transformation_matrix = np.block([[rot_mat, tvec.reshape(-1, 1)], [0, 0, 0, 1]])
            quaternion = tf2_geometry_msgs.quaternions.mat2quat(transformation_matrix[:3, :3])
            translation = tvec[0]

            print("Quaternion:", quaternion)
            print("Translation:", translation)

    # Display the modified image with detected ArUco markers
    cv2.imshow("ArUco Marker Detection", cv_image)
    cv2.waitKey(1)

def main():
    rclpy.init()
    node = rclpy.create_node('aruco_marker_detection')
    qos_profile = QoSProfile(depth=10)
    
    # Subscribe to camera info topic to get calibration data
    camera_info_subscriber = node.create_subscription(CameraInfo, '/camera/camera/color/camera_info', camera_info_callback, qos_profile)

    subscriber = node.create_subscription(Image, '/camera/camera/color/image_raw', image_callback, qos_profile)
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
