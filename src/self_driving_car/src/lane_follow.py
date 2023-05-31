import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from prius_msgs.msg import Control
import cv2

# Lane detection parameters
# Adjust these parameters according to your specific lane detection algorithm
# These parameters assume you are using OpenCV's Canny edge detection and Hough line transform
CANNY_THRESHOLD_LOW = 50
CANNY_THRESHOLD_HIGH = 150
HOUGH_THRESHOLD = 100
MIN_LINE_LENGTH = 10
MAX_LINE_GAP = 10

# Lane-following control parameters
STEERING_LEFT = 1.0
STEERING_STRAIGHT = 0.5

class LaneFollower:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/prius/front_camera', Image, self.image_callback)
        self.control_pub = rospy.Publisher('/prius', Control, queue_size=1)
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Perform lane detection
            lanes_image = self.detect_lanes(cv_image)
            
            # Calculate steering angle based on lane detection results
            steering_angle = self.calculate_steering_angle(lanes_image)
            
            # Publish control message
            control_msg = Control()
            control_msg.throttle = 1.0
            control_msg.brake = 0.0
            control_msg.steer = steering_angle
            control_msg.shift_gears = 0
            self.control_pub.publish(control_msg)
            
        except Exception as e:
            rospy.logerr(e)
    
    def detect_lanes(self, image):
        # Perform necessary image processing for lane detection
        # Example: Convert to grayscale, apply Canny edge detection, apply Hough line transform
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray_image, CANNY_THRESHOLD_LOW, CANNY_THRESHOLD_HIGH)
        lines = cv2.HoughLinesP(edges, 1, cv2.cv.CV_PI/180, HOUGH_THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP)
        
        # Create an image with lanes drawn on it
        lanes_image = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(lanes_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        
        return lanes_image
    
    def calculate_steering_angle(self, lanes_image):
        # Calculate the steering angle based on the lane detection results
        # Example: Determine the position of lanes and adjust the steering angle accordingly
        # In this example, we simply set a fixed steering angle
        steering_angle = STEERING_STRAIGHT
        
        return steering_angle

def main():
    rospy.init_node('lane_follower')
    lane_follower = LaneFollower()
    rospy.spin()

if __name__ == '__main__':
    main()
