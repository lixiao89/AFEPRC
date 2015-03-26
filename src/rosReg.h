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
     
      sub_force = nh_.subscribe<cisst_msgs::vctDoubleVec>("/logger/MsrFT", 1000, &rosReg::cb_force,this);
       sub_pose = nh_.subscribe<geometry_msgs::Pose>("/logger/MsrSE3", 1000, &rosReg::cb_pose,this);

       force_reading.assign(3,0);
       ee_pos.assign(3,0);
       ee_qua.assign(4,0);
       ee_ori.push_back(ee_pos);
       ee_ori.push_back(ee_pos);
       ee_ori.push_back(ee_pos);
       
       quaternion2rotation(ee_qua, ee_ori);
       plreg = new PlaneRegistration(force_reading, ee_pos, ee_ori);
          
    }


  void cb_force(const cisst_msgs::vctDoubleVec::ConstPtr& msg){
    for (int i = 0; i < 3; i++)
      force_reading[i] = msg->data[i];
  }

  void cb_pose(const geometry_msgs::Pose::ConstPtr& msg){

    ee_pos[0] = msg->position.x;
    ee_pos[1] = msg->position.y;
    ee_pos[2] = msg->position.z;

    ee_qua[0] = msg->orientation.w;
    ee_qua[1] = msg->orientation.x;
    ee_qua[2] = msg->orientation.y;
    ee_qua[3] = msg->orientation.z;
    
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

    rotation[0] = x;
    rotation[1] = y;
    rotation[2] = z;
    
  }

  void run(){
    // std::cout<<force_reading[0]<<" "<<force_reading[1]<<" "<<force_reading[2]<<std::endl;
     quaternion2rotation(ee_qua, ee_ori);
    if (plreg->append_buffer(force_reading, ee_pos, ee_ori))
      std::vector<double> error = plreg->register_plane();
    //std::cout<<error[0]<<","<<error[1]<<","<<error[2]<<std::endl;
  }
       
};      

#endif



