#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "base_controller_test");
    ros::NodeHandle n;
    ros::Rate r(100);

    DJIDrone* drone = new DJIDrone(n);
    drone->request_sdk_permission_control();
    sleep(1);
    drone->takeoff();
    sleep(4);

    while(n.ok()) {
        drone->attitude_control(0x5b, 2, 2, 2, 0);
	r.sleep();
    }
    return 0;
}
