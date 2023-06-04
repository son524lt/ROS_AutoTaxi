import rospy
import numpy as np
from prius_msgs.msg import Control
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
import cv2

expected_deviation = -70
Kp = 0.1
Ki = 0.01
Kd = 0.05
error_prev = 0
error_sum = 0

pub_control = rospy.Publisher('/prius', Control, queue_size=1)
pub = rospy.Publisher('/prius/front_camera/lane_detection', SensorImage, queue_size=1)

def calculate_deviation(lane_contours, image_width):
    centroids = []
    for contour in lane_contours:
        x, _, w, _ = cv2.boundingRect(contour)
        cx = x + w // 2
        centroids.append(cx)

    if centroids:
        deviation = np.mean(centroids) - image_width / 2
        return deviation
    else:
        return None

def control_car(deviation):
    global error_prev, error_sum

    error = deviation - expected_deviation
    error_sum += error
    error_diff = error - error_prev

    steer = np.clip(-Kp * error - Ki * error_sum - Kd * error_diff, -1, 1)
    throttle = 1.0
    brake = 0.0
    shift_gears = Control.FORWARD

    control_msg = Control()
    control_msg.header.stamp = rospy.Time.now()
    control_msg.throttle = throttle
    control_msg.brake = brake
    control_msg.steer = steer
    control_msg.shift_gears = shift_gears

    pub_control.publish(control_msg)

    error_prev = error

def image_callback(msg):
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
        upper_yellow = np.array([30, 255, 255], dtype=np.uint8)
        lower_white = np.array([0, 0, 200], dtype=np.uint8)
        upper_white = np.array([180, 30, 255], dtype=np.uint8)

        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        white_mask = cv2.inRange(hsv_image, lower_white, upper_white)

        lane_mask = cv2.bitwise_or(yellow_mask, white_mask)

        kernel = np.ones((5, 5), np.uint8)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(lane_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        lane_contours = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:
                epsilon = 0.1 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                lane_contours.append(approx)

        cv2.drawContours(cv_image, lane_contours, -1, (0, 255, 0), 2)

        deviation = calculate_deviation(lane_contours, cv_image.shape[1])

        if deviation is not None:
            deviation_text = "Deviation: {:.2f} pixels".format(deviation)
            cv2.putText(cv_image, deviation_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 255, 50), 2, cv2.LINE_AA)

        lane_detection_msg = CvBridge().cv2_to_imgmsg(cv_image, "bgr8")
        pub.publish(lane_detection_msg)
        control_car(deviation)

    except Exception as e:
        rospy.logerr(e)

def lane_detection_node():
    rospy.init_node('lane_detection')
    rospy.Subscriber('/prius/front_camera/image_raw', SensorImage, image_callback)
    rospy.spin()

if __name__ == '__main__':
    lane_detection_node()
