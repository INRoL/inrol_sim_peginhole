#include "inrol_sim_peginhole/inrol_sim_peginhole.hpp"
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <string>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Bool.h"
#include "inrol_sim_peginhole_main/msg_for_joint.h"

// Pub
ros::Publisher obj_pose_pub;
ros::Publisher contact_wrench_pub;
ros::Publisher joint_angle_pub;
geometry_msgs::PoseStamped msg_obj_pose;
geometry_msgs::WrenchStamped msg_contact_wrench;
inrol_sim_peginhole_main::msg_for_joint msg_joint_angle;
void publish();

// Sub
ros::Subscriber start_bool_sub;
bool start_bool = false;
void Callback_start_bool(const std_msgs::Bool msg_start_bool);

// Desired variables
double pd[3];
double Rd[3][3];

// Simulation variables
double xk_sim[3];
double qk_sim[4]; // x, y, z, w
double ak_sim[8];
double Fc_sim[6];


int main(int argc, char** argv) {
	ros::init(argc, argv, "simulation_main");
	ros::NodeHandle n("~");
	ros::Rate loop_rate(100);

	// publish
	obj_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/obj_pose",100);
	contact_wrench_pub = n.advertise<geometry_msgs::WrenchStamped>("/contact_wrench",100);
	joint_angle_pub = n.advertise<inrol_sim_peginhole_main::msg_for_joint>("/franka_joint_angle",100);

  // subscribe
	start_bool_sub = n.subscribe("/start_bool",20, Callback_start_bool);

  // dir
  auto asset_root = n.param<std::string>("asset_root_directory", "./");

  // initialize
  inrol_sim::initialize(asset_root);

  // wait for initialization of visualization node
	while (!start_bool) 
	{
		ros::spinOnce();
		loop_rate.sleep();
	}  

  // count des data
  int N = 0;
  std::ifstream file;
  std::string line;
  file.open((asset_root + "/des/pd.txt").c_str());
  while (getline(file, line))	N++;
  file.close();

  // load data  
  std::ifstream file1;
  std::ifstream file2;
  file1.open((asset_root + "/des/pd.txt").c_str());
  file2.open((asset_root + "/des/Rd.txt").c_str());

  // simulation
  int count = 0;
  while (true)
  {
    for (int i = 0; i < 3; i++)
    {
      file1 >> pd[i];
      for (int j = 0; j < 3; j++)
      {
        file2 >> Rd[i][j];
      }
    }

    inrol_sim::sim_function(pd, Rd, xk_sim, qk_sim, ak_sim, Fc_sim);
    publish();
		loop_rate.sleep();

    count++;
    if (count > N-10)
    {
      break;
    }
  }

  return 0;
}

void publish(){
		msg_obj_pose.pose.position.x = xk_sim[0];
		msg_obj_pose.pose.position.y = xk_sim[1];
		msg_obj_pose.pose.position.z = xk_sim[2];
		msg_obj_pose.pose.orientation.x = qk_sim[0];
		msg_obj_pose.pose.orientation.y = qk_sim[1];
		msg_obj_pose.pose.orientation.z = qk_sim[2];
		msg_obj_pose.pose.orientation.w = qk_sim[3];
		msg_obj_pose.header.stamp = ros::Time::now();

		msg_contact_wrench.wrench.force.x = Fc_sim[0];
		msg_contact_wrench.wrench.force.y = Fc_sim[1];
		msg_contact_wrench.wrench.force.z = Fc_sim[2];
		msg_contact_wrench.wrench.torque.x = Fc_sim[3];
		msg_contact_wrench.wrench.torque.y = Fc_sim[4];
		msg_contact_wrench.wrench.torque.z = Fc_sim[5];
		msg_contact_wrench.header.stamp = ros::Time::now();

		for (size_t j = 0; j < 7; j++)
		{
			msg_joint_angle.joint_angle[j] = ak_sim[j];
		}

		obj_pose_pub.publish(msg_obj_pose);						  
		contact_wrench_pub.publish(msg_contact_wrench);	  
		joint_angle_pub.publish(msg_joint_angle);	 
}

void Callback_start_bool(const std_msgs::Bool msg_start_bool)
{
	start_bool =  msg_start_bool.data;
}
