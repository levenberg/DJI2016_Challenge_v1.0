#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <dji_sdk/dji_drone.h>

DJIDrone* drone;
//using the body coordinate in horizontal frame, all control is velocity.
void base_controller_callback(const geometry_msgs::TwistConstPtr& msg) {
    drone->attitude_control(0x5B, msg->linear.x, msg->linear.y, 1.5, msg->angular.z * 57.3);        
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle n;

    drone = new DJIDrone(n);
    drone->request_sdk_permission_control();
    sleep(1);
    drone->takeoff();
    sleep(1);
 
    ros::Subscriber sub = n.subscribe("cmd_vel", 10, base_controller_callback);
    ros::spin();

    return 0;
}
