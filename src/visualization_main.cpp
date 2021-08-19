#include "inrol_sim_peginhole/inrol_sim_peginhole.hpp"
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <string>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Bool.h"
#include <inrol_sim_peginhole_main/msg_for_joint.h>

// Pub
ros::Publisher start_bool_pub;
std_msgs::Bool msg_start_bool;
void publish();

// Sub
ros::Subscriber obj_pose_sub;
ros::Subscriber joint_angle_sub;
double xk_sim[3];
double qk_sim[4]; // x, y, z, w
double ak_sim[8];
void Callback_xk_and_qk(const geometry_msgs::PoseStamped msg_obj_pose);
void Callback_ak(const inrol_sim_peginhole_main::msg_for_joint msg_joint_angle);

int main(int argc, char** argv) {
	ros::init(argc, argv, "visualization_main");
	ros::NodeHandle n("~");

  // publish
  start_bool_pub = n.advertise<std_msgs::Bool>("/start_bool",100);

  // subscribe
	obj_pose_sub = n.subscribe("/obj_pose",20, Callback_xk_and_qk);
	joint_angle_sub = n.subscribe("/franka_joint_angle",20, Callback_ak);

  // dir
  auto asset_root = n.param<std::string>("asset_root_directory", "./");

  // initialize
  inrol_sim::vis_initialize(asset_root);
  publish();

  // simulation
  while (true)
  {
		ros::spinOnce();
    inrol_sim::vis_function(xk_sim, qk_sim, ak_sim);
    publish();
  }

  return 0;
}

void publish(){
    msg_start_bool.data = true; 
		start_bool_pub.publish(msg_start_bool);	  
}

void Callback_xk_and_qk(const geometry_msgs::PoseStamped msg_obj_pose){

		xk_sim[0] = msg_obj_pose.pose.position.x;
		xk_sim[1] = msg_obj_pose.pose.position.y;
		xk_sim[2] = msg_obj_pose.pose.position.z;
    qk_sim[0] = msg_obj_pose.pose.orientation.x;
    qk_sim[1] = msg_obj_pose.pose.orientation.y;
    qk_sim[2] = msg_obj_pose.pose.orientation.z;
    qk_sim[3] = msg_obj_pose.pose.orientation.w;
}

void Callback_ak(const inrol_sim_peginhole_main::msg_for_joint msg_joint_angle){
  for (size_t j = 0; j < 7; j++)
  {
    ak_sim[j] = msg_joint_angle.joint_angle[j];
  }
}