#include <ros/ros.h>
#include <stdlib.h>
#include<iostream>
#include <sensor_msgs/LaserScan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static float ob_distance[5]= {10.0};

void obstacle_distance_callback ( const sensor_msgs::LaserScan & g_oa )
{
  for ( int i=0; i<5; i++ )
    ob_distance[i]=g_oa.ranges[i];
}


int main(int argc, char** argv){
  ros::init(argc, argv, "navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  ros::NodeHandle n;
  ros::Subscriber obstacle_distance_sub = n.subscribe ( "/guidance/obstacle_distance",10,obstacle_distance_callback );

  //Test filtering

  sleep(5);
  
 int x = 0;

  while (n.ok()) {
     // srand(time(NULL));
     // int x = rand() % 8;
      switch (x) {
	  case 0:
	      goal.target_pose.pose.position.x = 2.0;
	      goal.target_pose.pose.position.y = 0;
	      goal.target_pose.pose.orientation.w = 2.0;
	  //   x = 1;
	      break;
	  case 1:
	      goal.target_pose.pose.position.x = 0;
	      goal.target_pose.pose.position.y = 2.0;
	      goal.target_pose.pose.orientation.w = 1.0;
	   //   x = 2;
	      break;
	  case 2:
	      goal.target_pose.pose.position.x = -2.0;
	      goal.target_pose.pose.position.y = 0;
	      goal.target_pose.pose.orientation.w = 1.0;
	    //  x = 3;
	      break;
	  case 3:
	      goal.target_pose.pose.position.x = 0;
	      goal.target_pose.pose.position.y = -2.0;
	      goal.target_pose.pose.orientation.w = 1.0;
	   //   x = 0;
	      break;
// 	  case 4:
// 	      goal.target_pose.pose.position.x = 2.0;
// 	      goal.target_pose.pose.position.y = 7;
// 	      
// 	      goal.target_pose.pose.orientation.w = 1.0;
// 	      x++;
// 	      break;
// 	  case 5:
// 	      goal.target_pose.pose.position.x = -7;
// 	      goal.target_pose.pose.position.y = 7;
// 	      goal.target_pose.pose.orientation.w = 1.0;
// 	      x++;
// 	      break;
// 	      case 6:
// 	      goal.target_pose.pose.position.x = 7;
// 	      goal.target_pose.pose.position.y = -7;
// 	      goal.target_pose.pose.orientation.w = 1.0;
// 	      x++;
// 	      break;
// 	  case 7:
// 	      goal.target_pose.pose.position.x = -7;
// 	      goal.target_pose.pose.position.y = -7;
// 	      goal.target_pose.pose.orientation.w = 1.0;
// 	      x++;
// 	      break;
      }
      

      ROS_INFO("Sending goal");
    //  sleep(45);
      ac.sendGoal(goal);

      if(ac.waitForResult(ros::Duration(0,0)));
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
         ROS_INFO("Hooray, the base moved 1 meter forward");

      else if(ac.getState()== actionlib::SimpleClientGoalState::ABORTED)
      {	
         ROS_INFO("ABORTED");
	 x ++ ;
      }
      
      else if(ac.getState()== actionlib::SimpleClientGoalState::PENDING)
      {	
         ROS_INFO("PENDING");
	// x +=3 ;
      }
    //  std::cout<<ac.getState().toString()<<"\n";
  }

  return 0;
}
