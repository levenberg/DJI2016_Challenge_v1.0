#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "DJI_guidance.h"
#include "DJI_utility.h"

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic
#include <sensor_msgs/CameraInfo.h> // camera info message. Contains cam params
#include "yaml-cpp/yaml.h" // use to parse YAML calibration file
#include <fstream> // required to parse YAML 

ros::Publisher depth_image_pub;
ros::Publisher left_image_pub;
ros::Publisher right_image_pub;
ros::Publisher imu_pub;
ros::Publisher obstacle_distance_pub;
ros::Publisher velocity_pub;
ros::Publisher ultrasonic_pub;
ros::Publisher cam_info_left_pub; // camera info msg publishers
ros::Publisher cam_info_right_pub;

using namespace cv;

#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

char        	key       = 0;
e_vbus_index	CAMERA_ID = e_vbus1;
DJI_lock        g_lock;
DJI_event       g_event;
Mat             g_greyscale_image_left(HEIGHT, WIDTH, CV_8UC1);
Mat				g_greyscale_image_right(HEIGHT, WIDTH, CV_8UC1);
Mat				g_depth(HEIGHT,WIDTH,CV_16SC1);
Mat				depth8(HEIGHT, WIDTH, CV_8UC1);

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

static int i = 0;

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

		if ( data->m_greyscale_image_left[CAMERA_ID] ){
			memcpy(g_greyscale_image_left.data, data->m_greyscale_image_left[CAMERA_ID], IMAGE_SIZE);
			imshow("left",  g_greyscale_image_left);
			char buffer[20]="00000";
			//for(int i = 0;i<25;i++)
			//{
			  sprintf(buffer,"%03d_left.jpg",i);
			  imwrite(buffer,g_greyscale_image_left);
 			 ROS_INFO("Taking a left_picture");;

			//}
					
		if ( data->m_greyscale_image_right[CAMERA_ID] ){
			memcpy(g_greyscale_image_right.data, data->m_greyscale_image_right[CAMERA_ID], IMAGE_SIZE);
			//imshow("right", g_greyscale_image_right);
			// publish right greyscale image
						char buffer[20]="00000";
			//for(int i = 0;i<25;i++)
			//{
			  sprintf(buffer,"%03d_right.jpg",i);
			  imwrite(buffer,g_greyscale_image_right);
 ROS_INFO("Taking a right_picture");;

			//}
				

 
		}

	i++;
        key = waitKey();
    }
}
    g_lock.leave();
    g_event.set_event();

    return 0;
}

#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); \
return -1;}}

int main(int argc, char** argv)
{

    /* initialize ros */
    ros::init(argc, argv, "GuidanceNode");
    ros::NodeHandle my_node;
    camera_params_left =  "/root/Documents/roswork/DJI2016_Challenge/src/Guidance-SDK-ROS/calibration_files/camera_params_left.yaml";
    camera_params_right = "/root/Documents/roswork/DJI2016_Challenge/src/Guidance-SDK-ROS/calibration_files/camera_params_right.yaml";
    my_node.getParam("/right_param_file", camera_params_right);
    depth_image_pub			= my_node.advertise<sensor_msgs::Image>("/guidance/depth_image",1);
    left_image_pub			= my_node.advertise<sensor_msgs::Image>("/guidance/left/image_raw",1);
    right_image_pub			= my_node.advertise<sensor_msgs::Image>("/guidance/right/image_raw",1);
    imu_pub  				= my_node.advertise<geometry_msgs::TransformStamped>("/guidance/imu",1);
    velocity_pub  			= my_node.advertise<geometry_msgs::Vector3Stamped>("/guidance/velocity",1);
    obstacle_distance_pub	= my_node.advertise<sensor_msgs::LaserScan>("/guidance/obstacle_distance",1);
    ultrasonic_pub			= my_node.advertise<sensor_msgs::LaserScan>("/guidance/ultrasonic",1);
    cam_info_right_pub      = my_node.advertise<sensor_msgs::CameraInfo>("/guidance/right/camera_info",1);
    cam_info_left_pub       = my_node.advertise<sensor_msgs::CameraInfo>("/guidance/left/camera_info",1);

    /* initialize guidance */
    reset_config();
    int err_code = init_transfer();
    RETURN_IF_ERR(err_code);

	int online_status[CAMERA_PAIR_NUM];
	err_code = get_online_status(online_status);
	RETURN_IF_ERR(err_code);
    std::cout<<"Sensor online status: ";
	for (int i=0; i<CAMERA_PAIR_NUM; i++)
        std::cout<<online_status[i]<<" ";
    std::cout<<std::endl;

	// get cali param
	stereo_cali cali[CAMERA_PAIR_NUM];
	err_code = get_stereo_cali(cali);
	RETURN_IF_ERR(err_code);
    std::cout<<"cu\tcv\tfocal\tbaseline\n";
	for (int i=0; i<CAMERA_PAIR_NUM; i++)
	{
        std::cout<<cali[i].cu<<"\t"<<cali[i].cv<<"\t"<<cali[i].focal<<"\t"<<cali[i].baseline<<std::endl;
	}
	
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
		if (key > 0)
			// set exposure parameters
			if(key=='j' || key=='k' || key=='m' || key=='n'){
				if(key=='j')
					if(para.m_is_auto_exposure) para.m_expected_brightness += 20;
					else para.m_exposure_time += 3;
				else if(key=='k')
					if(para.m_is_auto_exposure) para.m_expected_brightness -= 20;
					else para.m_exposure_time -= 3;
				else if(key=='m'){
					para.m_is_auto_exposure = !para.m_is_auto_exposure;
                    std::cout<<"exposure is "<<para.m_is_auto_exposure<<std::endl;
				}
				else if(key=='n'){//return to default
					para.m_expected_brightness = para.m_exposure_time = 0;
				}

                std::cout<<"Setting exposure parameters....SensorId="<<CAMERA_ID<<std::endl;
                para.m_camera_pair_index = CAMERA_ID;
				set_exposure_param(&para);
				key = 0;
			}
			else {// switch image direction
				err_code = stop_transfer();
				RETURN_IF_ERR(err_code);
				reset_config();

				if (key == 'q') break;
				if (key == 'w') CAMERA_ID = e_vbus1;
				if (key == 'd') CAMERA_ID = e_vbus2;
				if (key == 'x') CAMERA_ID = e_vbus3;
				if (key == 'a') CAMERA_ID = e_vbus4;	   
				if (key == 's') CAMERA_ID = e_vbus5;

				select_greyscale_image(CAMERA_ID, true);
				select_greyscale_image(CAMERA_ID, false);
				select_depth_image(CAMERA_ID);

				err_code = start_transfer();
				RETURN_IF_ERR(err_code);
				key = 0;
			}
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