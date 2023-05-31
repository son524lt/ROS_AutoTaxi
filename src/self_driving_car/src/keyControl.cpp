#include <ros/ros.h>
#include <prius_msgs/Control.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "prius_control_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<prius_msgs::Control>("/prius", 1);

  // Create the control message
  prius_msgs::Control control_msg;
  control_msg.throttle = 1.0;
  control_msg.brake = 0.0;
  control_msg.steer = 0.0;
  control_msg.shift_gears = 0;

  // Publish control messages in a loop
  ros::Rate loop_rate(10);  // Adjust the loop rate as needed
  while (ros::ok())
  {
    pub.publish(control_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
