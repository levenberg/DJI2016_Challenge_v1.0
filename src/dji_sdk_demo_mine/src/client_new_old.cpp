#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

//#include<geometry_msgs/PoseWithCovarianceStamped.h>

#include<std_msgs/Int8.h>
#include<std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include<math.h>
#include<fstream>

#define GIMBAL_USED
#define filter_N 4
#define EPS 0.0000000001
#define USE_OBDIST

#define PI 3.1415926536
#define VEL_MODE

ofstream writeF ( "/root/log.txt",ios::app );
using namespace DJI::onboardSDK;
using namespace actionlib;
static uint8_t numOfDetections = 0;
static float last_flight_x = 0.0;
static float last_flight_y = 0.0;
static float last_flight_yaw = 0.0;//not used yet
static bool using_smallTags = false;
static float flight_yaw_relative = 0.0;//not used yet
static float ob_distance[5]= {10.0};


dji_sdk::MissionWaypointTask Waypoint_mission ( double latitude_ori, double longitude_ori, float altitude_ori,double latitude_end, double longitude_end, float altitude_end );

float cmd_vel_x = 0.0, cmd_vel_y = 0.0 ,cmd_vel_z = 0.0;
float searching_height = 1.5;

void apriltag_detection_resultCallback ( const std_msgs::Int8 & num_of_detection )
{
  numOfDetections = num_of_detection.data;
  //ROS_INFO ( "%d tag(s) are detected",numOfDetections );
}
void apriltag_using_smallTags_callback ( const std_msgs::Bool & using_smallTags_ )
{
  using_smallTags = using_smallTags_.data;
}
void base_controller_callback ( const geometry_msgs::TwistConstPtr& msg )
{
  cmd_vel_x = msg->linear.x;
  cmd_vel_y = msg->linear.y;
  cmd_vel_z = msg->angular.z;
}
void obstacle_distance_callback ( const sensor_msgs::LaserScan & g_oa )
{
  for ( int i=0; i<5; i++ )
    ob_distance[i]=g_oa.ranges[i];

  //ROS_INFO("1: %f 2:%f 3:%f 4: %f 5:%f",ob_distance[0],ob_distance[1],ob_distance[2],ob_distance[3],ob_distance[4]);
}
float sum ( float a[],size_t len )
{
  float sum = 0;
  for ( int i = 0; i<len; i++ )
    {
      sum += a[i];
    }
  return sum;
}

float find_min ( float a[], size_t len )
{
  float min = a[0];

  for ( int  j = 1; j<len; j++ )
    {
      if ( a[j]<min )
        min = a[j];

    }

  return min;
}

float find_max ( float a[], size_t len )
{
  float max = a[0];

  for ( int  j = 1; j<len; j++ )
    {
      if ( a[j]>max )
        max = a[j];

    }
  return max;
}



int main ( int argc, char **argv )
{
  int main_operate_code = 0;
  int temp32;
  bool valid_flag = false;
  bool err_flag = false;
  //for test
  double v_state_x=0.0, v_state_y=0.0, vxx=0.0, vyy=0.0;

  //Some params
  int delay_count = 100;
  int max_takeoff_waitcount = 700;
  double tracking_flight_height = 7.0;
  double descending_height_delta = 0.005;
  double initial_descending_height_at_search = 10.0;
  double initial_searching_height=1.5;
  
  ros::init ( argc, argv, "sdk_client" );
  ROS_INFO ( "sdk_service_client_test" );


  ros::NodeHandle node_priv ( "~" );
  node_priv.param<int> ( "delayCount",delay_count,100 );
  node_priv.param<double> ( "initTrackingHeight",tracking_flight_height,7.0 );
  node_priv.param<double> ( "descendSpeed",descending_height_delta,0.005 );
  node_priv.param<double> ( "initDescedingHeightSearch",initial_descending_height_at_search,10.0 );
  node_priv.param<int> ( "maxTakeoffWaitCount",max_takeoff_waitcount,700 );
  node_priv.param<double>("initial_searching_height",initial_searching_height,1.5);

  ros::NodeHandle nh;
  DJIDrone* drone = new DJIDrone ( nh );

  writeF<<ros::Time::now() <<endl;
  //virtual RC test data

  ros::Publisher  mission_type_pub = nh.advertise<std_msgs::Bool> ( "dji_sdk_demo/mission_type",10 );


  ros::Subscriber apriltag_num_of_detections_sub = nh.subscribe ( "apriltag_detection/numofdetections",100,apriltag_detection_resultCallback );
  ros::Subscriber apriltag_using_smallTags = nh.subscribe ( "apriltag_detection/usingSmallTags",10,apriltag_using_smallTags_callback );
  ros::Subscriber obstacle_distance_sub = nh.subscribe ( "/guidance/obstacle_distance",10,obstacle_distance_callback );

  //SEARCHING FOR SURVIVORS sub
  ros::Subscriber sub = nh.subscribe ( "cmd_vel", 10, base_controller_callback );

  //USED_APRILTAG_TYPE
  ros::Publisher start_searching_pub = nh.advertise<std_msgs::Bool> ( "/dji_sdk_demo/start_searching",10 );


  float flying_height_control_tracking = tracking_flight_height;
  float flying_height_control_searching = initial_descending_height_at_search;

  int count = 0;
  int count_tracking = 0;
  int count_losing_tracking =0;
  ros::Rate loop_rate ( 50 );
  static bool flag = false;
// enum used_apriltag_type{tag25H9=1, tag36H11, tag16H5};
  //uint8_t used_apriltag_type = 1;

  //float flight_x_filtered = 0.0;
// std_msgs::Float64 filtered_x_msg,not_filtered_x_msg;
  float filtered_x=0.0,filtered_y=0.0;
//filtered_x_msg.data = 0.0;
  float yaw=0;
// not_filtered_x_msg.data = 0.0;

  //For filtering;
  float filter_seq_x[filter_N]= {0},filter_seq_y[filter_N]= {0};

  //For vel_MODE
  float vel_x_drone = 0.0, vel_y_drone = 0.0;

  float initial_DeltaX, initial_DeltaY,DeltaX,DeltaY,initial_speedingup_height;
  float last_x_tag, last_y_tag;
  drone->gimbal_angle_control ( 0,-900,0,20 ); //Head down at the beginning.

  int landing_state = 0;
  int state_in_mission = 0;
  int tik_count = 0;

  dji_sdk::Waypoint waypoint0;// A fixed location, should be given when in challenge //Test GuiCao
//   waypoint0.latitude = 30.5388741;  
//   waypoint0.longitude = 114.355394;
//   waypoint0.latitude = 43.209349;
//   waypoint0.longitude = -75.413010;
   waypoint0.latitude = 43.2307347;
   waypoint0.longitude = -75.41944035;    //searching area landing gps 
  waypoint0.altitude = initial_descending_height_at_search;

  dji_sdk::MissionWaypointTask waypointtask;

  double longitude_start;
  double latitude_start; 

  int takeoff_wait_count = 0;

  float start_yaw = 0.0;
  int pitch_rate = 50;
  int yaw_rate = 50;

  std_msgs::Bool start_searching;
  start_searching.data= false;
  bool flip_once = true;
  while ( ros::ok() )
    {
      ros::spinOnce();
//       std_msgs::Bool mission_type;
//       if ( drone->transparentdata.data.at ( 0 ) =='q' )
//         mission_type.data=true;       //searching for apriltags
//       else if ( drone->transparentdata.data.at ( 0 ) =='d' )
//         mission_type.data=false;     //landing
//       mission_type_pub.publish ( mission_type );
//

      for ( int i = 0; i< filter_N-1; i++ )
        {
          filter_seq_x[i] = filter_seq_x[i+1];
          filter_seq_y[i] = filter_seq_y[i+1];
        }

      filter_seq_x[filter_N-1] = drone->flight_x;
      filter_seq_y[filter_N-1] = drone->flight_y;

      filtered_x =  drone->flight_x;//( sum ( filter_seq_x,filter_N )-find_max ( filter_seq_x,filter_N )-find_min ( filter_seq_x,filter_N ) ) / ( filter_N-2 );

      filtered_y =  drone->flight_y;//( sum ( filter_seq_y,filter_N )-find_max ( filter_seq_y,filter_N )-find_min ( filter_seq_y,filter_N ) ) / ( filter_N-2 );

      start_searching_pub.publish ( start_searching );

      switch ( drone->transparentdata.data.at ( 0 ) )
        {

          /*****************************************************'Q' : IN  MISSION**********************************************/
          /********************************************************************************************************************/
        case 'q':
        {
          if ( takeoff_wait_count<max_takeoff_waitcount ) //Take off, waypoint-fly to a fixed location and then descend to 3m
            {
              writeF<<"take off process..."<<start_yaw<<endl;
              drone->attitude_control ( 0x9B,0,0,initial_descending_height_at_search,0 );
              takeoff_wait_count ++;//attitude_control(0x9B,0,0,6,0);
              start_yaw = drone->yaw_from_drone;//Record the yaw of taking off
              longitude_start = drone->global_position.longitude;
	      writeF<<"*******initial globallong "<<longitude_start<<endl;
              latitude_start = drone->global_position.latitude;
	      writeF<<"*******initial global lat"<<latitude_start<<endl;

            }
          else
            switch ( state_in_mission )
              {
                /******************************TAKE OFF **************************************/
              case 0:


                waypointtask.mission_waypoint.clear();
                waypointtask=Waypoint_mission ( latitude_start,longitude_start,initial_descending_height_at_search,waypoint0.latitude,waypoint0.longitude,waypoint0.altitude );
                drone->mission_waypoint_upload ( waypointtask );
                drone->mission_start();
                //sleep(2);
                //writeF<<"waypoint. state=0, navigation..."<<drone->waypoint_mission_push_info.current_status<<endl;
                while ( drone->transparentdata.data.at ( 0 ) != 'a'&& drone->mission_waypoint_get_speed() >0.10 )
                  ros::spinOnce();

                writeF<<"waypoint. state=0, navigation finished, descending"<<ob_distance[0]<<endl;
                state_in_mission = 1;
                break;
                /***********************ACTION OF MISSIONS*******************************************/
              case 1://
                // writeF<<"waypoint. state=1, navigation finished, start descending."<<drone->local_position.z<<endl;

#ifdef USE_OBDIST

                writeF<<"waypoint. state=1, navigation finished, start descending."<<ob_distance[0]<<endl;

             /*   if ( drone->local_position.z>6)// ob_distance[0]<1.5 )
                  {
                    flying_height_control_searching -= 0.008;
                  }
                else*/ 
	     if ( ob_distance[0]>1.7 )
                  {
                    flying_height_control_searching -= 0.008;
                    drone->gimbal_angle_control ( 0,-300,0,10,1 );
                  }

                else if ( ob_distance[0]<initial_searching_height ) //1.5m
                  state_in_mission = 2;

                 
                 drone->attitude_control ( 0x9B,0,0,flying_height_control_searching,0 );


#else

                writeF<<"waypoint. state=1, navigation finished, start descending."<<drone->local_position.z<<endl;
                if ( drone->local_position.z<1.5 ||flying_height_control_searching<1.5 ) //ob_distance[0]<1.5 )
                  {
                    flying_height_control_searching += 0.02;
                  }
                else if ( drone->local_position.z>1.6 &&flying_height_control_searching>1.6 ) //ob_distance[0]>1.8 )
                  {
                    flying_height_control_searching -= 0.02;
                    drone->gimbal_angle_control ( 0,-300,0,10,1 );
                  }
                //      else
                //        state_in_mission = 2;
                drone->attitude_control ( 0x9B,0,0,flying_height_control_searching,0 );

                if ( drone->local_position.z<1.8 )


                  state_in_mission = 2;
#endif
                break;

              case 2:

                writeF<<"waypoint. state=2, decending finished, start searching..."<<ob_distance[0]<<endl;
//                 if ( drone->local_position.z<1.5 ) //||flying_height_control_searching<1.5 ) //ob_distance[0]<1.5 )
//                   {
//                     flying_height_control_searching += 0.005;
//
//                   }
//                 else if ( drone->local_position.z>1.6 ) //&&flying_height_control_searching>1.6 ) //ob_distance[0]>1.8 )
//                   {
//                     flying_height_control_searching -= 0.005;
//                   }
                //            drone->attitude_control ( 0x93,0,0,flying_height_control_searching,90+start_yaw*57.2958 );  //jiaodu kongzhi
                ROS_INFO ( "IN Q's state 2. Ready for Searching mode." );
                start_searching.data= true;

                /*****************SEARCHING FOR SURVIVORS!**************************************************/

                //TODO: USE this to complete this system.(NOT USED YET)
                if ( start_searching.data==true )
                  {
#ifdef USE_OBDIST
                    if ( ob_distance[0]<1.5 )
                      {
                        flying_height_control_searching += 0.005;

                      }
                    else if ( ob_distance[0]>1.6)
		    {
                        flying_height_control_searching -= 0.005;  
		    }
#endif
                    drone->attitude_control ( 0x5b, cmd_vel_x, cmd_vel_y, flying_height_control_searching, cmd_vel_z * 57.3 );

                    pitch_rate = drone->gimbal.pitch>15? -50: ( drone->gimbal.pitch>-89? pitch_rate:50 );
                    yaw_rate = drone->gimbal.yaw>45? -50: ( drone->gimbal.yaw>-45?yaw_rate:50 );

                    drone->gimbal_speed_control ( 0,pitch_rate,yaw_rate ); //gimbal_angle_control ( 0, ( int ) drone->flight_gimbal_angle_pitch_inc*10,0,10,0 );
                  }
                break;
              }
          break;

        }
        /************************************************'d' FOR ENDING MISSION**********************************************************************/
        /*****************************************************************************************************************/
        case 'd':
        {

          switch ( state_in_mission )
            {
              /**********************************RETURN TO LOCATION*****************************************************/
            case 2:
              if ( takeoff_wait_count>0 )
                {


                  if ( takeoff_wait_count == 1 )
                    {

                      FILE *fp = popen ( "pkill -9 stereo_image","r" );
                      fp = popen ( "pkill -9 navigation_","r" );
                      fp = popen ( "pkill -9 move_base","r" );
                      fp = popen ( "pkill -9 amcl","r" );
                      fp = popen ( "pkill -9 map_server","r" );
                      fp = popen ( "pkill -9 tf_","r" );
                      fp = popen ( "pkill -9 static_transform_publisher","r" );
                    }


                  //  tag_type.data= tag36H11;// Imform the apriltag_detection routine to detect 36h11;
                  //  used_apriltag_pub.publish ( tag_type );

                  takeoff_wait_count --;
                  drone->attitude_control ( 0x9B,0,0,initial_descending_height_at_search,0 );
                }
              else
                {


                  waypointtask.mission_waypoint.clear();
                  waypointtask=Waypoint_mission ( drone->global_position.latitude,drone->global_position.longitude,initial_descending_height_at_search,latitude_start,longitude_start,initial_descending_height_at_search );
                  drone->mission_waypoint_upload ( waypointtask );
                  drone->mission_start();
                  sleep ( 2 );
                  //writeF<<"waypoint. state=0, navigation..."<<drone->waypoint_mission_push_info.current_status<<endl;
                  while ( drone->transparentdata.data.at ( 0 ) != 'a'&& drone->mission_waypoint_get_speed() >0.10 )
                    ros::spinOnce();


                  while ( drone->transparentdata.data.at ( 0 ) != 'a'&& drone->local_position.z>tracking_flight_height+0.2 )
                    {
                      drone->attitude_control ( 0x9B,0,0,tracking_flight_height,0 );
                      usleep ( 20000 );
                      ros::spinOnce();
                    }
                  int ttime=0;
                  while ( drone->transparentdata.data.at ( 0 ) != 'a'&& ttime++<200 )
                    {
                      drone->attitude_control ( 0x93,0,0,tracking_flight_height,start_yaw*57.2958 );

                      drone->gimbal_angle_control ( 0,-600,0,10,1 );
                      usleep ( 20000 );
                      ros::spinOnce();
                    }


                  state_in_mission = 100;// MAY need debugging

                }


              break;///////////////////////////////////////CASE 2 BREAK

              /***************************************SEARCHING FOR LANDING SPOT************************************************************/
            case 100://

              if ( numOfDetections<1 )
                {
                  pitch_rate = drone->gimbal.pitch>-30? -50: ( drone->gimbal.pitch>-89? pitch_rate:50 );
                  drone->gimbal_speed_control ( 0,pitch_rate,0 ); //Scan for target. Use pitch only.
                }

              else
                state_in_mission = 3;
              break;////////////////////////////////////CASE 100 break

              /*******************************************TRACKING FOR A WHILE****************************************************/
            case 3:

              drone->gimbal_angle_control ( 0, ( int ) drone->flight_gimbal_angle_pitch_inc*10,0,10,0 );
              if ( numOfDetections>=1 )
                {

                  last_flight_x = filtered_x;
                  last_flight_y = filtered_y;

                  if ( filtered_x>0.5 )         filtered_x=0.5;
                  else if ( filtered_x<-0.5 )   filtered_x=-0.5;
                  else                       filtered_x=filtered_x;

                  if ( filtered_y>0.5 )         filtered_y=0.5;
                  else if ( filtered_y<-0.5 )   filtered_y=-0.5;
                  else                       filtered_y=filtered_y;

                  drone->attitude_control ( 0x9B,filtered_x,filtered_y, tracking_flight_height, 0.0 );
                  count_losing_tracking = 0 ;
                }

              else if ( numOfDetections == 0 )
                {
                  count_losing_tracking ++;
                  if ( count_losing_tracking<50 ) // if lossing the tag, keep flying for a little bit of time
                    drone->attitude_control ( 0x9B,last_flight_x,last_flight_y,tracking_flight_height,0.0 );// Keep the speed for 1 seconds if loss the tag

                  else // Losing track of target
                    {
                      drone->attitude_control ( 0x9B,0,0,tracking_flight_height,0.0 );// Keep the speed for 1 seconds if loss the tag
                    }
                }
              if ( numOfDetections>=1 && filtered_x < 0.1 && filtered_y < 0.1 ) // Ready for descending
                state_in_mission = 4;

              break;////////////////////////////////////////////////CASE 3 BREAK

              /****************************************DESCENDING-LANDING************************************************************/
            case 4:
              drone->gimbal_angle_control ( 0, -900,0,10,1 );// Fix the pitch angle : right down
              if ( numOfDetections >= 1 )
                {

                  if ( drone->local_position.z > 1.35 )
                    flying_height_control_tracking -= descending_height_delta*2.2;
                  else
                    flying_height_control_tracking -= descending_height_delta;//Descending slowerly

                  drone->attitude_control ( 0x9B,filtered_x,filtered_y, flying_height_control_tracking, 0.0 );
                  if ( using_smallTags )
                    {
                      writeF << "--------------------------small tag stage---------------------------" << endl;
                    }
                }
              else
                {

                  flying_height_control_tracking -= descending_height_delta;//Descending slowerly
                  drone->attitude_control ( 0x9B,0,0, flying_height_control_tracking, 0.0 );

                }
              break;////////////////////////////////////////////CASE 4 (Landing) BREAK;
            case  5:
              break;

            default:
              break;
            }
        }
        break;//////////////////////////////////////////////CASE 'd' BREAK;


        /*****************************************************'a' for abort mission***************************************************************************/
        /********************************************************************************************************************************************/
        case 'a':

          // for ( int i = 0; i<200; i++ )
          drone->attitude_control ( 0x9B,0,0,drone->local_position.z,0 );
          // while ( !drone->release_sdk_permission_control() )

          drone->release_sdk_permission_control();
          //usleep ( 10000 );
          break;

        default:
          //  ROS_INFO ( "Waithing for command from mobile!\n" );
          break;
        }
      // ROS_INFO ( "Command code is %c",drone->transparentdata.data.at ( 0 ) );
      loop_rate.sleep();

    }
  writeF.close();
  return 0;
}

dji_sdk::MissionWaypointTask Waypoint_mission ( double latitude_ori, double longitude_ori, float altitude_ori,double latitude_end, double longitude_end, float altitude_end )
{
  dji_sdk::MissionWaypointTask waypoint_task;
  dji_sdk::MissionWaypoint 	 waypoint;
// Clear the vector of previous waypoints
  waypoint_task.mission_waypoint.clear();

  //mission waypoint upload
  waypoint_task.velocity_range = 10;
  waypoint_task.idle_velocity = 3.5;
  waypoint_task.action_on_finish = 0;
  waypoint_task.mission_exec_times = 1;
  waypoint_task.yaw_mode = 0;
  waypoint_task.trace_mode = 0;
  waypoint_task.action_on_rc_lost = 0;
  waypoint_task.gimbal_pitch_mode = 0;
  //waypoint0
  waypoint.latitude=latitude_ori;
  waypoint.longitude=longitude_ori;
  waypoint.altitude=altitude_ori;
  waypoint.damping_distance = 0;
  waypoint.target_yaw = 0;
  waypoint.target_gimbal_pitch = 0;
  waypoint.turn_mode = 0;
  waypoint.has_action = 0;
  waypoint_task.mission_waypoint.push_back ( waypoint );


  //waypoint1
  waypoint.latitude=latitude_end;
  waypoint.longitude=longitude_end;
  waypoint.altitude=altitude_end;
  waypoint.damping_distance = 0;
  waypoint.target_yaw = 0;
  waypoint.target_gimbal_pitch = 0;
  waypoint.turn_mode = 0;
  waypoint.has_action = 0;
  waypoint_task.mission_waypoint.push_back ( waypoint );

  return waypoint_task;

  //drone->mission_waypoint_upload(waypoint_task);
}
