#include <ros/ros.h>
#include <prius_msgs/Control.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

double expected_deviation = -60;
double Kp = 0.05;
double Ki = 0.0001;
double Kd = 0.04;
double error_prev = 0;
double error_sum = 0;

ros::Publisher pub_control;
ros::Publisher pub;

double calculate_deviation(const std::vector<std::vector<cv::Point>>& lane_contours, int image_width)
{
    std::vector<double> centroids;
    for (const auto& contour : lane_contours)
    {
        cv::Rect rect = cv::boundingRect(contour);
        double cx = rect.x + rect.width / 2.0;
        centroids.push_back(cx);
    }

    if (!centroids.empty())
    {
        double deviation = cv::mean(centroids)[0] - image_width / 2.0;
        return deviation;
    }
    else
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

void control_car(double deviation)
{
    static double error_sum = 0;
    static double error_prev = 0;

    double error = deviation - expected_deviation;
    error_sum += error;
    double error_diff = error - error_prev;

    double steer = std::clamp(-Kp * error - Ki * error_sum - Kd * error_diff, -1.0, 1.0);
    double throttle = 1.0;
    double brake = 0.0;
    int shift_gears = prius_msgs::Control::FORWARD;

    prius_msgs::Control control_msg;
    control_msg.header.stamp = ros::Time::now();
    control_msg.throttle = throttle;
    control_msg.brake = brake;
    control_msg.steer = steer;
    control_msg.shift_gears = shift_gears;

    pub_control.publish(control_msg);

    error_prev = error;
}

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat hsv_image;
        cv::cvtColor(cv_image->image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Scalar lower_yellow(20, 100, 100);
        cv::Scalar upper_yellow(30, 255, 255);
        cv::Scalar lower_white(0, 0, 200);
        cv::Scalar upper_white(180, 30, 255);

        cv::Mat yellow_mask, white_mask;
        cv::inRange(hsv_image, lower_yellow, upper_yellow, yellow_mask);
        cv::inRange(hsv_image, lower_white, upper_white, white_mask);

        cv::Mat lane_mask;
        cv::bitwise_or(yellow_mask, white_mask, lane_mask);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(lane_mask, lane_mask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(lane_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<cv::Point>> lane_contours;
        for (const auto& contour : contours)
        {
            if (cv::contourArea(contour) > 100)
            {
                double epsilon = 0.1 * cv::arcLength(contour, true);
                std::vector<cv::Point> approx;
                cv::approxPolyDP(contour, approx, epsilon, true);
                lane_contours.push_back(approx);
            }
        }

        cv::drawContours(cv_image->image, lane_contours, -1, cv::Scalar(0, 255, 0), 2);

        double deviation = calculate_deviation(lane_contours, cv_image->image.cols);

        if (!std::isnan(deviation))
        {
            std::string deviation_text = "Deviation: " + std::to_string(deviation) + " pixels";
            cv::putText(cv_image->image, deviation_text, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50, 255, 50), 2, cv::LINE_AA);
        }

        sensor_msgs::ImagePtr lane_detection_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image->image).toImageMsg();
        pub.publish(lane_detection_msg);
        control_car(deviation);
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void lane_detection_node()
{
    ros::NodeHandle nh;
    pub_control = nh.advertise<prius_msgs::Control>("/prius", 1);
    pub = nh.advertise<sensor_msgs::Image>("/prius/front_camera/lane_detection", 1);
    ros::Subscriber sub = nh.subscribe("/prius/front_camera/image_raw", 1, image_callback);
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane_detection");
    lane_detection_node();
    return 0;
}
