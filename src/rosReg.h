#ifndef ROS_RLS_HPP_
#define ROS_RLS_HPP_


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cisst_msgs/vctDoubleVec.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>

#include "plane_registration.h"
#include "adaptive_estimation.h"

class rosReg{ 
	
    public:
  
  ros::NodeHandle nh_;

  ros::Subscriber sub_pose;
  ros::Subscriber sub_force;
  ros::Subscriber sub_joint_states;
  
  std::vector<double> force_reading;
  std::vector<double> ee_pos;
  std::vector<std::vector<double> > ee_ori;
  std::vector<double> ee_qua; // w, x, y, z

  std::vector<double> current_joint_states;
  double Fn_now;
  double Ft_now;
  double currTime;
  double startTime;
  
  PlaneRegistration* plreg;
  AdaptiveEstimation* adest;
  
  rosReg(ros::NodeHandle& nh)
    {
      nh_ = nh;
     
      sub_force = nh_.subscribe<cisst_msgs::vctDoubleVec>("/logger/MsrFT", 1000, &rosReg::cb_force,this);
       sub_pose = nh_.subscribe<geometry_msgs::Pose>("/logger/MsrSE3", 1000, &rosReg::cb_pose,this);
       sub_joint_states = nh_.subscribe<sensor_msgs::JointState>("/pid/joint_current",1000, &rosReg::cb_joint_states,this);
       
       force_reading.assign(3,0);
       ee_pos.assign(3,0);
       ee_qua.assign(4,0);
       ee_ori.push_back(ee_pos);
       ee_ori.push_back(ee_pos);
       ee_ori.push_back(ee_pos);

       current_joint_states.assign(7,0);
       
       quaternion2rotation(ee_qua, ee_ori);
       plreg = new PlaneRegistration(force_reading, ee_pos, ee_ori);
       adest = new AdaptiveEstimation(0,0,0,current_joint_states);

       startTime = ros::Time::now().toSec();
    }


  void cb_force(const cisst_msgs::vctDoubleVec::ConstPtr& msg){
    for (int i = 0; i < 3; i++)
      force_reading[i] = msg->data[i];

    Fn_now = msg->data[2];
    Ft_now = msg->data[0];
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

  void cb_joint_states(const sensor_msgs::JointState::ConstPtr& msg){
    for(int i = 0; i < current_joint_states.size(); i++){
      current_joint_states[i] = msg->position[i];
    }
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

    // -------  Plane Registration and Correction ----------------
    // std::cout<<force_reading[0]<<" "<<force_reading[1]<<" "<<force_reading[2]<<std::endl;
     quaternion2rotation(ee_qua, ee_ori);
     if (plreg->append_buffer(force_reading, ee_pos, ee_ori)){
       std::vector<double> error1(3,0);
       error1 = plreg->register_plane();
       std::cout<<error1[0]<<","<<error1[1]<<","<<error1[2]<<std::endl;
     }

     // --------- Adaptive Estimation --------------------------
     double current_time = ros::Time::now().toSec() - startTime;
     adest->append_buff(current_time, Fn_now, Ft_now, current_joint_states);
     AdaptiveEstimation::TaskState current_task_state = adest->run_task_monitor();

     Eigen::Vector2d param;
     double Fest;
     bool is_moving_now;
     adest->get_member_var(param, Fest, is_moving_now);

     // std::cout<<param<<std::endl;
     //std::cout<<"---"<<std::endl;
  }
       
};      

#endif



