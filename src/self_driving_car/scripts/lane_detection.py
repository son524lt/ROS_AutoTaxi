#!/usr/bin/env python3

import rospy
import numpy as np
from PIL import Image
# from keras.models import load_model
import cv2
from prius_msgs.msg import Control
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
import math

# def check_sign():
#     # Load saved model
#     model = load_model('traffic_sign_model.h5')

#     # Open camera
#     camera = cv2.VideoCapture(0)  # Số 0 đại diện cho camera mặc định, bạn có thể thay đổi nếu bạn muốn sử dụng camera khác

#     while True:
#         # Đọc khung hình từ camera
#         ret, frame = camera.read()

#         # Chuyển đổi khung hình sang định dạng RGB
#         frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#         # Resize khung hình
#         image = Image.fromarray(frame_rgb)
#         image = image.resize((64, 64))

#         # Chuẩn bị dữ liệu đầu vào
#         x_new = np.array(image)
#         x_new = np.expand_dims(x_new, axis=0)

#         # Dự đoán
#         y_predict = model.predict(x_new)
#         global predicted_class
#         predicted_class = np.argmax(y_predict)

#         # Hiển thị kết quả dự đoán
#         cv2.putText(frame, f"Predicted Class: {predicted_class}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
#         cv2.imshow("Camera", frame)

#         # # Gọi hàm điều khiển xe
#         # control_car()

#         # Thoát khỏi vòng lặp nếu nhấn phím 'q'
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     # Giải phóng camera và đóng cửa sổ hiển thị
#     camera.release()
#     cv2.destroyAllWindows()

# def lane_following(frame):
#     # Thực hiện xử lý điều khiển bám lane
#     lane_center = frame.shape[1] // 2  # Trung tâm của lane đường bên phải
#     lane_deviation = predicted_class - lane_center  # Sai lệch vị trí so với lane trung tâm

#     # Tính toán tốc độ và góc quay dựa trên sai lệch vị trí
#     kp = 0.1  # Hệ số điều chỉnh P
#     steering_angle = kp * lane_deviation  # Góc quay dựa trên sai lệch vị trí
#     steering_angle = np.clip(steering_angle, -math.pi/4, math.pi/4)  # Giới hạn góc quay trong khoảng -pi/4 đến pi/4


#     pub.publish()


# def velocity_callback(velocity):
#     global linear_velocity, angular_velocity
#     # Lấy dữ liệu vận tốc tuyến tính và góc quay từ lệnh
#     linear_velocity = velocity.linear.x
#     angular_velocity = velocity.angular.z

#     # Tính toán tốc độ cho bánh trái và bánh phải
#     wheel_radius = 0.5  # Bán kính bánh xe
#     base_width = 1.0  # Khoảng cách giữa hai bánh xe

#     left_wheel_velocity = (linear_velocity - angular_velocity * base_width / 2) / wheel_radius
#     right_wheel_velocity = (linear_velocity + angular_velocity * base_width / 2) / wheel_radius

#     # In ra màn hình giá trị tốc độ của bánh trái và bánh phải
#     rospy.loginfo("Left Wheel Velocity: %.2f, Right Wheel Velocity: %.2f", left_wheel_velocity, right_wheel_velocity)
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

expected_deviation = -70  # Set the expected deviation value
Kp = 0.1  # Proportional gain for PID control
Ki = 0.01  # Integral gain for PID control
Kd = 0.05  # Derivative gain for PID control

# Initialize variables for PID control
error_prev = 0
error_sum = 0


def control_car(deviation):
    global error_prev, error_sum

    # PID control calculations
    error = expected_deviation - deviation
    error_sum += error
    error_diff = error - error_prev

    steer = np.clip(-Kp * error - Ki * error_sum - Kd * error_diff, -1, 1)
    throttle = 1.0
    brake = 0.0
    shift_gears = Control.FORWARD

    # Create a Control message
    control_msg = Control()
    control_msg.header.stamp = rospy.Time.now()
    control_msg.throttle = throttle
    control_msg.brake = brake
    control_msg.steer = steer
    control_msg.shift_gears = shift_gears

    # Publish the Control message
    pub_control.publish(control_msg)

    # Store the current error for the next iteration
    error_prev = error

def image_callback(msg):
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define yellow and white color ranges for lane detection
        lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
        upper_yellow = np.array([30, 255, 255], dtype=np.uint8)
        lower_white = np.array([0, 0, 200], dtype=np.uint8)
        upper_white = np.array([180, 30, 255], dtype=np.uint8)

        # Create masks for yellow and white colors
        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        white_mask = cv2.inRange(hsv_image, lower_white, upper_white)

        # Combine yellow and white masks
        lane_mask = cv2.bitwise_or(yellow_mask, white_mask)

        # Apply morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)

        # Find lane contours
        contours, _ = cv2.findContours(lane_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter out small contours and approximate the lane contours
        lane_contours = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:
                epsilon = 0.1 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                lane_contours.append(approx)

        # Draw lane contours on the original image
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
    # rospy.init_node('car_controller')
    # rospy.Subscriber('cmd_vel', Twist, velocity_callback)
    # check_sign()
    lane_detection_node()
