#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "m100_publisher");  // give a name
    ros::NodeHandle n;

    ros::Rate r(100);

    tf::TransformBroadcaster broadcaster;

    const double pi = 3.1415926535;

    while(n.ok()) {
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(pi / 2, 0, -pi / 2), tf::Vector3(0.12, 0.0, 0.1)),
                ros::Time::now(),"base_link", "m100_guidance"));
        r.sleep();
    }
}
