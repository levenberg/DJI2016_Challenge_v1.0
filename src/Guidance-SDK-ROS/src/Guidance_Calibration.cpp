#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>

#include "DJI_guidance.h"
#include "DJI_utility.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include "yaml-cpp/yaml.h"

ros::Publisher depth_image_pub;
ros::Publisher cam_info_left_pub;
ros::Publisher cam_info_right_pub;
ros::Publisher obstacle_distance_pub;

using namespace cv;

#define WIDTH  320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

char        	key       = 0;
e_vbus_index	CAMERA_ID = e_vbus1;
DJI_lock        g_lock;
DJI_event       g_event;
Mat		g_depth(HEIGHT,WIDTH,CV_16SC1);
Mat		depth8(HEIGHT, WIDTH, CV_8UC1);

std::string camera_params_left;
std::string camera_params_right;

static const char CAM_YML_NAME[]    = "camera_name";
static const char WIDTH_YML_NAME[]  = "image_width";
static const char HEIGHT_YML_NAME[] = "image_height";
static const char K_YML_NAME[]      = "camera_matrix";
static const char D_YML_NAME[]      = "distortion_coefficients";
static const char R_YML_NAME[]      = "rectification_matrix";
static const char P_YML_NAME[]      = "projection_matrix";
static const char DMODEL_YML_NAME[] = "distortion_model";

// struct to parse camera calibration YAML
struct SimpleMatrix
{
    int rows;
    int cols;
    double* data;

    SimpleMatrix(int rows, int cols, double* data)
        : rows(rows), cols(cols), data(data)
    {}
};

void transfer_SimpleMatrix_from_YML_to_ROSmsg(const YAML::Node& node, SimpleMatrix& m)
{
    int rows, cols;
    rows = node["rows"].as<int>();
    cols = node["cols"].as<int>();
    const YAML::Node& data = node["data"];
    for (int i = 0; i < rows*cols; ++i)
    {
        m.data[i] = data[i].as<double>();
    }
}

void read_params_from_yaml_and_fill_cam_info_msg(std::string& file_name, sensor_msgs::CameraInfo& cam_info)
{
    std::ifstream fin(file_name.c_str());
    YAML::Node doc = YAML::Load(fin);

    cam_info.width = doc[WIDTH_YML_NAME].as<int>();
    cam_info.height = doc[HEIGHT_YML_NAME].as<int>();

    SimpleMatrix K_(3, 3, &cam_info.K[0]);
    transfer_SimpleMatrix_from_YML_to_ROSmsg(doc[K_YML_NAME], K_);
    SimpleMatrix R_(3, 3, &cam_info.R[0]);
    transfer_SimpleMatrix_from_YML_to_ROSmsg(doc[R_YML_NAME], R_);
    SimpleMatrix P_(3, 4, &cam_info.P[0]);
    transfer_SimpleMatrix_from_YML_to_ROSmsg(doc[P_YML_NAME], P_);

    cam_info.distortion_model = doc[DMODEL_YML_NAME].as<std::string>();

    const YAML::Node& D_node = doc[D_YML_NAME];
    int D_rows, D_cols;
    D_rows = D_node["rows"].as<int>();
    D_cols = D_node["cols"].as<int>();
    const YAML::Node& D_data = D_node["data"];
    cam_info.D.resize(D_rows*D_cols);
    for (int i = 0; i < D_rows*D_cols; ++i)
    {
        cam_info.D[i] = D_data[i].as<float>();
    }
}

int my_callback(int data_type, int data_len, char *content)
{
    g_lock.enter();

    /* image data */
    if (e_image == data_type && NULL != content)
    {
        ros::Time time_in_this_loop = ros::Time::now();
        image_data* data = (image_data*)content;

        sensor_msgs::CameraInfo g_cam_info_left;
        g_cam_info_left.header.stamp = time_in_this_loop;
        g_cam_info_left.header.frame_id = "m100_guidance";

        read_params_from_yaml_and_fill_cam_info_msg(camera_params_left, g_cam_info_left);
        cam_info_left_pub.publish(g_cam_info_left);

        if ( data->m_depth_image[CAMERA_ID] ) {
            memcpy(g_depth.data, data->m_depth_image[CAMERA_ID], IMAGE_SIZE * 2);
            g_depth.convertTo(depth8, CV_8UC1);
            imshow("depth", depth8);
            //publish depth image
            cv_bridge::CvImage depth_16;
            g_depth.copyTo(depth_16.image);
            depth_16.header.frame_id  = "m100_guidance";
            depth_16.header.stamp = time_in_this_loop;
            depth_16.encoding	  = sensor_msgs::image_encodings::TYPE_16UC1;
            depth_image_pub.publish(depth_16.toImageMsg());
        }

        key = waitKey(1);
    }

    /* obstacle distance */
    if ( e_obstacle_distance == data_type && NULL != content )
    {
        obstacle_distance *oa = (obstacle_distance*)content;
     
	// publish obstacle distance
	sensor_msgs::LaserScan scan;
	scan.ranges.resize(CAMERA_PAIR_NUM);
	scan.header.frame_id = "m100_guidance";
	scan.header.stamp    = ros::Time::now();

	scan.angle_min = -1.57;
        scan.angle_max = 1.57;
        scan.angle_increment = 3.14 / 5;
        scan.time_increment = (1 / 40) / (5);
        scan.range_min = 0.0;
        scan.range_max = 100.0;

	for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
	scan.ranges[i] = 0.01f * oa->distance[i];
	obstacle_distance_pub.publish(scan);		
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

int main(int argc, char** argv)
{
    /* initialize ros */
    ros::init(argc, argv, "GuidanceNode");
    ros::NodeHandle my_node;
    // my_node.getParam("/left_param_file", camera_params_left);
    // my_node.getParam("/right_param_file", camera_params_right);
    // camera_params_left = "/root/Desktop/DJI2016_Challenge/src/Guidance-SDK-ROS/calibration_files/camera_params_left.yaml";
    camera_params_left = "/home/tohka/DJI2016_Challenge/src/Guidance-SDK-ROS/calibration_files/camera_params_left.yaml";
    depth_image_pub	   = my_node.advertise<sensor_msgs::Image>("image",1);
    cam_info_left_pub      = my_node.advertise<sensor_msgs::CameraInfo>("camera_info",1);
    // cam_info_right_pub  = my_node.advertise<sensor_msgs::CameraInfo>("/guidance/right/camera_info",1);
    obstacle_distance_pub  = my_node.advertise<sensor_msgs::LaserScan>("scan",1);

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
