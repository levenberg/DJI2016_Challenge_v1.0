/*
 * GuidanceNode.cpp
 *
 *  Created on: Apr 29, 2015
 */
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

// ros::Publisher depth_image_pub;
// ros::Publisher left_image_pub;
// ros::Publisher right_image_pub;
// ros::Publisher imu_pub;
// ros::Publisher velocity_pub;
ros::Publisher obstacle_distance_pub;
ros::Publisher ultrasonic_pub;

using namespace cv;

#define WIDTH  320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

char        	key       = 0;
e_vbus_index	CAMERA_ID = e_vbus1;
DJI_lock        g_lock;
DJI_event       g_event;

int my_callback(int data_type, int data_len, char *content)
{
    g_lock.enter();

    /* obstacle distance */
    if ( e_obstacle_distance == data_type && NULL != content )
    {
        obstacle_distance *oa = (obstacle_distance*)content;

        // publish obstacle distance
        sensor_msgs::LaserScan g_oa;
        g_oa.ranges.resize(CAMERA_PAIR_NUM);
        g_oa.header.frame_id = "guidance";
        g_oa.header.stamp    = ros::Time::now();
        for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
            g_oa.ranges[i] = 0.01f * oa->distance[i];
        obstacle_distance_pub.publish(g_oa);
    }

    /* ultrasonic */
    if ( e_ultrasonic == data_type && NULL != content )
    {
        ultrasonic_data *ultrasonic = (ultrasonic_data*)content;

        // publish ultrasonic data
        sensor_msgs::LaserScan g_ul;
        g_ul.ranges.resize(CAMERA_PAIR_NUM);
        g_ul.intensities.resize(CAMERA_PAIR_NUM);
        g_ul.header.frame_id = "guidance";
        g_ul.header.stamp    = ros::Time::now();
        for ( int d = 0; d < CAMERA_PAIR_NUM; ++d ){
            g_ul.ranges[d] = 0.001f * ultrasonic->ultrasonic[d];
            g_ul.intensities[d] = 1.0 * ultrasonic->reliability[d];
        }
        ultrasonic_pub.publish(g_ul);
    }

    g_lock.leave();
    g_event.set_event();

    return 0;
}

#define RETURN_IF_ERR(err_code) {  \
    if( err_code ) {               \
        release_transfer();        \
        return -1;                 \
    }                              \
}

int main(int argc, char** argv)
{
    /* initialize ros */
    ros::init(argc, argv, "GuidanceNode");
    ros::NodeHandle my_node;
    // depth_image_pub		= my_node.advertise<sensor_msgs::Image>("/guidance/depth_image",1);
    // left_image_pub		= my_node.advertise<sensor_msgs::Image>("/guidance/left_image",1);
    // right_image_pub		= my_node.advertise<sensor_msgs::Image>("/guidance/right_image",1);
    // imu_pub  		= my_node.advertise<geometry_msgs::TransformStamped>("/guidance/imu",1);
    // velocity_pub  		= my_node.advertise<geometry_msgs::Vector3Stamped>("/guidance/velocity",1);
    obstacle_distance_pub	= my_node.advertise<sensor_msgs::LaserScan>("/guidance/obstacle_distance",1);
    ultrasonic_pub		= my_node.advertise<sensor_msgs::LaserScan>("/guidance/ultrasonic",1);

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

    while (ros::ok()) {
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
