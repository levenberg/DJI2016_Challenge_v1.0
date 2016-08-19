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

void apriltag_detection_resultCallback ( const std_msgs::Int8 & num_of_detection )
{
  numOfDetections = num_of_detection.data;
  //ROS_INFO ( "%d tag(s) are detected",numOfDetections );
}
void apriltag_using_smallTags_callback ( const std_msgs::Bool & using_smallTags_ )
{
  using_smallTags = using_smallTags_.data;
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
  double tracking_flight_height = 6.0;
  double descending_height_delta = 0.005;
  double initial_descending_height_at_search = 10.0;

  ros::init ( argc, argv, "sdk_client" );
  ROS_INFO ( "sdk_service_client_test" );


  ros::NodeHandle node_priv ( "~" );
  node_priv.param<int> ( "delayCount",delay_count,100 );
  node_priv.param<double> ( "initTrackingHeight",tracking_flight_height,6.0 );
  node_priv.param<double> ( "descendSpeed",descending_height_delta,0.005 );
  node_priv.param<double> ( "initDescedingHeightSearch",initial_descending_height_at_search,10.0 );
  node_priv.param<int> ( "maxTakeoffWaitCount",max_takeoff_waitcount,700 );


  ros::NodeHandle nh;
  DJIDrone* drone = new DJIDrone ( nh );

  writeF<<ros::Time::now() <<endl;
  //virtual RC test data

  ros::Publisher  mission_type_pub = nh.advertise<std_msgs::Bool> ( "dji_sdk_demo/mission_type",10 );


  ros::Subscriber apriltag_num_of_detections_sub = nh.subscribe ( "apriltag_detection/numofdetections",100,apriltag_detection_resultCallback );
  ros::Subscriber apriltag_using_smallTags = nh.subscribe ( "apriltag_detection/usingSmallTags",10,apriltag_using_smallTags_callback );

  //avoid collision in ascending
  ros::Subscriber obstacle_distance_sub = nh.subscribe ( "/guidance/obstacle_distance",10,obstacle_distance_callback );

  float flying_height_control_tracking = tracking_flight_height;
  float flying_height_control_searching = initial_descending_height_at_search;

  int count = 0;
  int count_tracking = 0;
  int count_losing_tracking =0;
  ros::Rate loop_rate ( 50 );
  static bool flag = false;

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

  //Waypoit flight control
  dji_sdk::WaypointList outgoing_waypointList;
  dji_sdk::WaypointList returning_waypointList;
  dji_sdk::Waypoint waypoint0;// A fixed location, should be given when in challenge //Test GuiCao
  waypoint0.latitude = 30.5398741;//30.544329;//30.00020;//
  waypoint0.longitude = 114.355394;//114.367015;//114.9995;//
  waypoint0.altitude = initial_descending_height_at_search;
  waypoint0.staytime = 10;
  waypoint0.heading = 0;

  dji_sdk::Waypoint waypoint_return;// location of returning location
  waypoint_return.latitude = 30.00010;
  waypoint_return.longitude = 114.9995;
  waypoint_return.altitude = initial_descending_height_at_search;
  waypoint_return.staytime = 1;
  waypoint_return.heading = 180;
  
  dji_sdk::Waypoint waypoint_return_1;// location of returning location

  
  double longitude_start;
  double latitude_start;
  int waypoint_fly_flip = 0;

  int takeoff_wait_count = 0;

  float start_yaw = 0.0;
  while ( ros::ok() )
    {
      ros::spinOnce();
      std_msgs::Bool mission_type;
      if ( drone->transparentdata.data.at ( 0 ) =='q' )
        mission_type.data=true;       //searching for apriltags
      else if ( drone->transparentdata.data.at ( 0 ) =='d' )
        mission_type.data=false;     //landing
      mission_type_pub.publish ( mission_type );


      for ( int i = 0; i< filter_N-1; i++ )
        {
          filter_seq_x[i] = filter_seq_x[i+1];
          filter_seq_y[i] = filter_seq_y[i+1];
        }

      filter_seq_x[filter_N-1] = drone->flight_x;
      filter_seq_y[filter_N-1] = drone->flight_y;

      filtered_x =  drone->flight_x;//( sum ( filter_seq_x,filter_N )-find_max ( filter_seq_x,filter_N )-find_min ( filter_seq_x,filter_N ) ) / ( filter_N-2 );

      filtered_y =  drone->flight_y;//( sum ( filter_seq_y,filter_N )-find_max ( filter_seq_y,filter_N )-find_min ( filter_seq_y,filter_N ) ) / ( filter_N-2 );


      switch ( drone->transparentdata.data.at ( 0 ) )
        {

          /*****************************************************'Q' : IN  MISSION**********************************************/
          /********************************************************************************************************************/
        case 'q':
        {
          if ( takeoff_wait_count<max_takeoff_waitcount ) //Take off, waypoint-fly to a fixed location and then descend to 3m
            {
	      writeF<<"take off process..."<<endl;
              drone->attitude_control ( 0x9B,0,0,initial_descending_height_at_search,0);
              takeoff_wait_count ++;//attitude_control(0x9B,0,0,6,0);
              start_yaw = drone->yaw_from_drone;//Record the yaw of taking off
              writeF<<"start_yaw: "<<start_yaw<<endl;
              longitude_start = drone->global_position.longitude;
	      latitude_start = drone->global_position.latitude;
              //writeF<<setprecision(9)<<longitude_start<<":"<<setprecision(9)<<latitude_start<<"::"<<drone->global_position.longitude<<":"<<drone->global_position.latitude<<endl;

	      
	    }
          else
            switch ( state_in_mission )
              {
                /******************************TAKE OFF **************************************/
              case 0:
                if ( 0==waypoint_fly_flip )
                  {
		    writeF<<"waypoint. state=0, navigation..."<<endl;
		    waypoint0.heading = start_yaw*57.2958;
                    //outgoing_waypointList.waypoint_list.push_back ( waypoint0 );
                    //  outgoing_waypointList.waypoint_list.push_back ( waypoint1 );
                    //drone->waypoint_navigation_send_request ( outgoing_waypointList );
		    drone->global_position_navigation_send_request (waypoint0.latitude, waypoint0.longitude, waypoint0.altitude );
                    waypoint_fly_flip = -1;
                  }
                // drone->Glo
                else if ( waypoint_fly_flip == -1 ) //( drone->waypoint_navigation_wait_for_result ( ros::Duration ( 30 ) ) ) // drone->waypoint_navigation_get_state()==SimpleClientGoalState::SUCCEEDED )
                  {
		    writeF<<"waypoint. state=0, waiting for navigation finished..."<<endl;
		    if (drone->global_position_navigation_wait_for_result() ) // Should be no problem...
                      {
                        ROS_INFO ( "IN Waypoint flight DONE." );
			//sleep ( 8 );

                        waypoint_fly_flip = 1;
                      }

                  }
                else if ( 1 == waypoint_fly_flip )
                  {
		    
                    //Descending when no obstacles below
                    if ( drone->global_position.altitude>1.8)// ob_distance[0]>1.8)
                      {
			writeF<<"waypoint. state=0, navigation finished, descending..."<<ob_distance[0]<<endl;
                        flying_height_control_searching -= 0.01;

                        drone->attitude_control ( 0x9B,0,0,flying_height_control_searching,0 );
                      }
                    else
                      {
			writeF<<"waypoint. state=0, navigation finished, descending finished"<<ob_distance[0]<<endl;
                        waypoint_fly_flip=0;
                        state_in_mission = 1;
                      }

                  }

                break;
                /***********************ACTION OF MISSIONS*******************************************/
              case 1://
		writeF<<"waypoint. state=1, navigation finished, descending finished."<<ob_distance[0]<<endl;
                sleep ( 10 );   //wait for 20s; NOTE:DEBUGING PURPOSE!
                if ( drone->global_position.altitude<1.5)//ob_distance[0]<1.5 )
                  {
                    flying_height_control_searching += 0.005;

                  }
                else if ( drone->global_position.altitude>1.8) //ob_distance[0]>1.8 )
                  {
                    flying_height_control_searching -= 0.005;
                  }
                  drone->attitude_control ( 0x9B,0,0,flying_height_control_searching,0 );

                //drone->global_position_navigation_send_request ( 22.535, 113.95, 100 );
                state_in_mission = 2;

                break;

              case 2:
                //drone->local_position_navigation_send_request(-100, -100, 100);
		writeF<<"waypoint. state=2, navigation finished, height keeping..."<<ob_distance[0]<<endl;
                if ( drone->global_position.altitude<1.5)//ob_distance[0]<1.5 )
                  {
                    flying_height_control_searching += 0.005;

                  }
                else if ( drone->global_position.altitude>1.8)//ob_distance[0]>1.8 )
                  {
                    flying_height_control_searching -= 0.005;
                  }
                drone->attitude_control ( 0x93,0,0,flying_height_control_searching,90+start_yaw*57.2958 );  //jiaodu kongzhi
                ROS_INFO ( "IN Q's state 2. Ready for Return Debugging mode." );
                break;
              default:
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
              if ( 0 == waypoint_fly_flip )
                {
                  if ( takeoff_wait_count>0 )
                    {
                      takeoff_wait_count --;
                      drone->attitude_control ( 0x9B,0,0,initial_descending_height_at_search,0 );

                    }
                  else
                    {
                      int temp_i = 0;
                      while ( temp_i ++ <200 ) //Turning around
                        {
                          // float temp_yaw = start_yaw<0  ?PI+start_yaw :-PI+start_yaw;

                          drone->attitude_control ( 0x93,0,0,initial_descending_height_at_search,start_yaw*57.2958 );
                          usleep ( 20000 );
                        }

                      /*
                       waypoint_return.heading = start_yaw*57.2958;//( start_yaw<0 ) ?180+start_yaw*57.2958:-180+start_yaw*57.2958;
                                   //  returning_waypointList.waypoint_list.empty();
                       waypoint_return.longitude =longitude_start;
                       waypoint_return.latitude = latitude_start;
                       waypoint_return.altitude = initial_descending_height_at_search;
                       waypoint_return_1 = waypoint_return;
                       waypoint_return_1.altitude = tracking_flight_height;
                                   returning_waypointList.waypoint_list.push_back( waypoint_return ); //(waypoint_return) FOR DEBUGGING PURPOSE;
                                   returning_waypointList.waypoint_list.push_back(waypoint_return_1);
                       drone->waypoint_navigation_send_request ( returning_waypointList );
                       */
                      drone->global_position_navigation_send_request ( latitude_start, longitude_start, initial_descending_height_at_search );
                      waypoint_fly_flip = 1;
                    }

                }
              else if ( drone->global_position_navigation_wait_for_result() )  //==SimpleClientGoalState::SUCCEEDED)
                {
		  drone->gimbal_angle_control ( 0,-450,0,10,1 );
                  state_in_mission = 3;
                  waypoint_fly_flip = 0;
                }

              break;///////////////////////////////////////CASE 2 BREAK

              /***************************************ADJUST ATTITUDE(not done yet)************************************************************/
            case 100://
              drone->gimbal_angle_control ( 0,-450,0,10,1 );
              state_in_mission = 3;
              ROS_INFO ( "Gimbal is at 45 degree." );
              break;


              /*******************************************TRACKING FOR A WHILE****************************************************/
            case 3:

              if ( numOfDetections>=1 )
                {

                  last_flight_x = filtered_x;
                  last_flight_y = filtered_y;
                  drone->attitude_control ( 0x9B,filtered_x,filtered_y, tracking_flight_height, 0.0 );
                }

              else if ( numOfDetections == 0 )
                {
                  count_losing_tracking ++;
                  if ( count_losing_tracking<200 ) // if lossing the tag, keep flying for a little bit of time
                    drone->attitude_control ( 0x9B,last_flight_x,last_flight_y,tracking_flight_height,0.0 );// Keep the speed for 1 seconds if loss the tag

                  else
                    {
                      //count_losing_tracking = 0;
                      drone->attitude_control ( 0x9B,0,0,tracking_flight_height,0.0 );// Keep the speed for 1 seconds if loss the tag
                    }


                }
              if ( 600 == count_tracking++ ) // Lasting for 30 seconds
                {
                  count_tracking = 0;
                  state_in_mission = 4;

                }
              break;////////////////////////////////////////////////CASE 3 BREAK

              /****************************************CATCHING UP AND DESCENDING************************************************************/
            case 4:
            {
#define ACC_HEIGHT 2.56
              if ( numOfDetections >= 1 )
                {
                  switch ( landing_state )
                    {

                      // 0 for height above 2.56m
                    case 0:
#ifdef GIMBAL_USED
                      drone->gimbal_angle_control ( 0, ( int ) drone->flight_gimbal_angle_pitch_inc*10,0,10,0 );
#endif
                      flying_height_control_tracking -= descending_height_delta*3.2;
                      drone->attitude_control ( 0x9B,filtered_x,filtered_y, flying_height_control_tracking, 0.0 );
                      if ( drone->local_position.z <= 2.5 )
                        {
                          landing_state = 1;
                        }
                      break;

                      // 1 for big tag
                    case 1:

                      if ( count_tracking == 0 )   // Execute only once, to save the initial statistics;
                        {
                          DeltaX = filtered_x;
                          DeltaY = filtered_y;
                          initial_DeltaX = drone->flight_x_tag;// Save the actual distance between the Tag(big) and M100;
                          initial_speedingup_height = drone->local_position.z;// Save the initial height of speeding up
                          writeF<<initial_DeltaX<<" "<<initial_speedingup_height<<" "<<drone->local_position.z<<endl;
                          count_tracking++;
                          drone->gimbal_angle_control ( 0, ( int ) drone->flight_gimbal_angle_pitch_inc*10,0,10,0 );
                        }
                      else
                        {
                          count_tracking = 1;
                          if ( drone->flight_x_tag > 0.5 )   //If Tag is still ahead
                            {
                              DeltaX += 0.003*drone->flight_x_tag / ( 1+drone->flight_x_tag ); //
                            }
                          else if ( drone->flight_x_tag > 0 )
                            {
                              DeltaX -= 0.003*drone->flight_x_tag / ( 1+drone->flight_x_tag ); //
                            }

                          float temp_height = ( 2* abs ( drone->flight_x_tag ) / ( abs ( drone->flight_x_tag ) +abs ( initial_DeltaX ) ) ) ;
                          temp_height = pow ( temp_height,2 ) *initial_speedingup_height;
                          flying_height_control_tracking = temp_height;
                          writeF <<"f_H_c:"<< flying_height_control_tracking<<" F_h:"<<drone->flight_height<<" L.z:"<<drone->local_position.z<<" --DeltaX:"<<DeltaX<<",filtered_x:"<<filtered_x<<endl;
                          drone->attitude_control ( 0x9B, DeltaX,filtered_y,flying_height_control_tracking,0.0 );
                          last_flight_x = DeltaX;
                          last_flight_y = filtered_y;

                          if ( using_smallTags )
                            {
                              count_tracking = 0;
                              landing_state = 2;
                              writeF << "--------------------------small tag stage---------------------------" << endl;
                            }
                          else
                            {
                              drone->gimbal_angle_control ( 0, ( int ) drone->flight_gimbal_angle_pitch_inc*10,0,10,0 );
                            }
                        }
                      break;

                    case 2:
                      if ( count_tracking == 0 )   // Execute only once, to save the initial statistics;
                        {
                          initial_DeltaX = drone->flight_x_tag;// Save the actual distance between the Tag(big) and M100;
                          initial_speedingup_height = drone->local_position.z;// Save the initial height of speeding up
                          writeF<<initial_DeltaX<<" "<<initial_speedingup_height<<" "<<drone->local_position.z<<endl;
                          count_tracking++;
                        }
                      writeF << "deltax" << DeltaX <<endl;
                      drone->attitude_control ( 0x9B, DeltaX,filtered_y,flying_height_control_tracking,0.0 );
                      if ( ++tik_count == 20 )
                        {
                          writeF << "20s has passed" << endl;
                          landing_state = 4;
                        }
                      break;
                    case 4:
                      //if ( drone->local_position.z <0.65 && abs ( drone->flight_x_tag ) <0.3 ) // If condition satisfied, quick dump;
                    {
                      writeF << "sharp" << endl;
                      ROS_INFO ( "In sharp descending." );
                      flying_height_control_tracking -= 0.10;
                      //DeltaX -= 0.2 ;
                      drone->attitude_control ( 0x9B, DeltaX-1,filtered_y,flying_height_control_tracking,0.0 );
                      writeF<<"Za"<<endl;
                      writeF <<"f_H_c:"<< flying_height_control_tracking<<" F_h:"<<drone->flight_height<<" L.z:"<<drone->local_position.z<<" --DeltaX:"<<DeltaX<<",filtered_x:"<<filtered_x<<endl;
                      last_flight_x = DeltaX;
                      last_flight_y = filtered_y;
                    }
                    break;

                    default:
                      break;
                    }
                }

              else if ( numOfDetections == 0 )
                {
                  // Keep the speed for 1 seconds if loss the tag
                  if ( landing_state == 2 )
                    {
                      drone->attitude_control ( 0x9B,last_flight_x, 0,flying_height_control_tracking,0.0 );
                    }
                  // ignore y controll
                  else
                    {
                      drone->attitude_control ( 0x9B,last_flight_x,last_flight_y,flying_height_control_tracking,0.0 );
                    }
                }


            }
            break;////////////////////////////////////////////CASE 4 (Landing) BREAK;


            case  5:
              break;



            default:
              break;
            }
        }
        break;//////////////////////////////////////////////CASE 'd' BREAK;





        /*****************************************************'S' for abort mission***************************************************************************/
        /********************************************************************************************************************************************/
        case 'S':

          break;
        default:
          //  ROS_INFO ( "Waithing for command from mobile!\n" );
          break;
        }
      ROS_INFO ( "Command code is %c",drone->transparentdata.data.at ( 0 ) );
      //count++;
      loop_rate.sleep();


    }
  writeF.close();

  return 0;
}

