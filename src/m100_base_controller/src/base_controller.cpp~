#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <dji_sdk/dji_drone.h>
#include <pthread.h>

DJIDrone* drone;

float x = 0.0, y = 0.0 ,z = 0.0;

void base_controller_callback(const geometry_msgs::TwistConstPtr& msg) {
    x = msg->linear.x;
    y = msg->linear.y;
    z = msg->angular.z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle n;

    drone = new DJIDrone(n);
    drone->request_sdk_permission_control();
    sleep(1);
    drone->takeoff();
    sleep(1);

    ros::Rate r(50);
 
    ros::Subscriber sub = n.subscribe("cmd_vel", 10, base_controller_callback);

    while (n.ok()) {
        ros::spinOnce();
        // usleep(10);
        drone->attitude_control(0x5b, x, y, 1.5, z * 57.3);
        r.sleep();
    }
        
    ros::spinOnce();
    return 0;
}


