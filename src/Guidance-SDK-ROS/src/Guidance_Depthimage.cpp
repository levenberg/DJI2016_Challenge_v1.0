#include <stdio.h>
#include <string.h>
#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "DJI_guidance.h"
#include "DJI_utility.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>  // IMU
#include <geometry_msgs/Vector3Stamped.h>  // velocity
#include <sensor_msgs/LaserScan.h>  // obstacle distance & ultrasonic
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

ros::Publisher depth_image_pub;

using namespace cv;

#define WIDTH  320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

char        	key = 0;
e_vbus_index	CAMERA_ID = e_vbus1;
DJI_lock        g_lock;
DJI_event       g_event;
Mat		g_depth(HEIGHT,WIDTH,CV_16SC1);
Mat		depth8(HEIGHT, WIDTH, CV_8UC1);

int my_callback(int data_type, int data_len, char *content)
{
    g_lock.enter();

    /* image data */
    if (e_image == data_type && NULL != content)
    {
        image_data* data = (image_data*)content;

        if ( data->m_depth_image[CAMERA_ID] ){
            memcpy(g_depth.data, data->m_depth_image[CAMERA_ID], IMAGE_SIZE * 2);
            g_depth.convertTo(depth8, CV_8UC1);
            imshow("depth", depth8);
            //publish depth image
            cv_bridge::CvImage depth_16;
            g_depth.copyTo(depth_16.image);
            depth_16.header.frame_id  = "guidance";
            depth_16.header.stamp	  = ros::Time::now();
            depth_16.encoding	  = sensor_msgs::image_encodings::MONO16;
            depth_image_pub.publish(depth_16.toImageMsg());
        }

        key = waitKey(1);
    }

    g_lock.leave();
    g_event.set_event();

    return 0;
}

#define RETURN_IF_ERR(err_code) {  \
    if(err_code) {                 \
        release_transfer();        \
        return -1;                 \
    }                              \
}

int main(int argc, char** argv) {
    /* initialize ros */
    ros::init(argc, argv, "GuidanceNode");
    ros::NodeHandle my_node;
    depth_image_pub = my_node.advertise<sensor_msgs::Image>("image",1);

    /* initialize guidance */
    reset_config();
    int err_code = init_transfer();
    RETURN_IF_ERR(err_code);

    int online_status[CAMERA_PAIR_NUM];
    err_code = get_online_status(online_status);
    RETURN_IF_ERR(err_code);

    // get cali param
    stereo_cali cali[CAMERA_PAIR_NUM];
    err_code = get_stereo_cali(cali);
    RETURN_IF_ERR(err_code);

    /* select data */
    err_code = select_greyscale_image(CAMERA_ID, true);
    RETURN_IF_ERR(err_code);
    err_code = select_greyscale_image(CAMERA_ID, false);
    RETURN_IF_ERR(err_code);
    err_code = select_depth_image(CAMERA_ID);
    RETURN_IF_ERR(err_code);
    select_imu();
    select_ultrasonic();
    select_obstacle_distance();
    select_velocity();
    /* start data transfer */
    err_code = set_sdk_event_handler(my_callback);
    RETURN_IF_ERR(err_code);
    err_code = start_transfer();
    RETURN_IF_ERR(err_code);

    // for setting exposure
    exposure_param para;
    para.m_is_auto_exposure = 1;
    para.m_step = 10;
    para.m_expected_brightness = 120;
    para.m_camera_pair_index = CAMERA_ID;

    std::cout << "start_transfer" << std::endl;

    while (ros::ok())
    {
        g_event.wait_event();
        ros::spinOnce();
    }

    /* release data transfer */
    err_code = stop_transfer();
    RETURN_IF_ERR(err_code);
    //make sure the ack packet from GUIDANCE is received
    sleep(1);
    std::cout << "release_transfer" << std::endl;
    err_code = release_transfer();
    RETURN_IF_ERR(err_code);

    return 0;
}
