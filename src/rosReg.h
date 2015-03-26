#ifndef ROS_RLS_HPP_
#define ROS_RLS_HPP_


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cisst_msgs/vctDoubleVec.h"
#include "geometry_msgs/Pose.h"
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>

#include "plane_registration.h"

class rosReg{ 
	
    public:
  
  ros::NodeHandle nh_;

  ros::Subscriber sub_pose;
  ros::Subscriber sub_force;

  std::vector<double> force_reading;
  std::vector<double> ee_pos;
  std::vector<std::vector<double> > ee_ori;
  std::vector<double> ee_qua; // w, x, y, z
 

  PlaneRegistration* plreg;
  
  rosReg(ros::NodeHandle& nh)
    {
      nh_ = nh;
     
      sub_force = nh_.subscribe<cisst_msgs::vctDoubleVec>("/logger/MsrFT", 10, &rosReg::cb_force,this);
       sub_pose = nh_.subscribe<geometry_msgs::Pose>("/logger/MsrSE3", 10, &rosReg::cb_pose,this);

       force_reading.assign(3,0);
       ee_pos.assign(3,0);
       ee_qua.assign(4,0);
       quaternion2rotation(ee_qua, ee_ori);
       plreg = new PlaneRegistration(force_reading, ee_pos, ee_ori);
          
    }


  void cb_force(const cisst_msgs::vctDoubleVec::ConstPtr& msg){
    for (int i = 0; i < 6; i++)
      force_reading.push_back(msg->data[i]);
  }

  void cb_pose(const geometry_msgs::Pose::ConstPtr& msg){

    ee_pos.push_back(msg->position.x);
    ee_pos.push_back(msg->position.y);
    ee_pos.push_back(msg->position.z);

    ee_qua.push_back(msg->orientation.w);
    ee_qua.push_back(msg->orientation.x);
    ee_qua.push_back(msg->orientation.y);
    ee_qua.push_back(msg->orientation.z);
    
  }

  void quaternion2rotation(const std::vector<double>& quaternion,
			   std::vector<std::vector<double> >& rotation){
    std::vector<double> x(3,0);
    std::vector<double> y(3,0);
    std::vector<double> z(3,0);

    double qw = ee_qua[0], qx = ee_qua[1], qy = ee_qua[2], qz = ee_qua[3];
    
    x[0] = 1 - 2*pow(qy,2) - 2*pow(qz,2);
    x[1] = 2*qx*qy + 2*qz*qw;
    x[2] = 2*qx*qz - 2*qy*qw;

    y[0] = 2*qx*qy - 2*qz*qw;
    y[1] = 1 - 2*pow(qx,2) - 2*pow(qz,2);
    y[2] = 2*qy*qz + 2*qx*qw;

    z[0] = 2*qx*qz + 2*qy*qw;
    z[1] = 2*qy*qz - 2*qx*qw;
    z[2] = 1 - 2*pow(qx,2) - 2*pow(qy,2);

    rotation.push_back(x);
    rotation.push_back(y);
    rotation.push_back(z);
    
  }

  void run(){

    if (plreg->append_buffer(force_reading, ee_pos, ee_ori))
      std::vector<double> error = plreg->register_plane();
    //std::cout<<error[0]<<","<<error[1]<<","<<error[2]<<std::endl;
  }
       
};      

#endif


