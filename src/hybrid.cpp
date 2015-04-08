#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <iostream>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames_io.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <kdl_conversions/kdl_msg.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>

#include <actionlib/client/simple_action_client.h>
#include <conman_msgs/GetBlocksAction.h>
#include <conman_msgs/SetBlocksAction.h>
#include <boost/scoped_ptr.hpp>

#include <math.h>   // PI
#include <vector>

#include "plane_registration.h"
#include "adaptive_estimation.h"

// TODO
//  - add joint pos callback
//  - test different types of ik solvers


// Steps :
//  - r: set WAM to ready state
//  - h: start hybrid controller


#define DEBUG 0

using namespace KDL;

class Control
{
public:
  enum CONTROL_TYPE
  {
    CTRL_IDLE,
    CTRL_READY,
    CTRL_HYBRID,
    CTRL_MANUAL,
    CTRL_CORRECT
  };

  Control(ros::NodeHandle* node, bool is_master, bool is_sim, std::string joy_name):
    node_(node),
    is_master_(is_master),
    is_simulation_(is_sim),
    ctrl_type_(CTRL_IDLE)
  {
    std::string ns;     // topic namespace
    if (is_simulation_) ns = "/gazebo";
    else ns = "/barrett";

    // publisher
    pub_jnt_cmd_ = node->advertise<trajectory_msgs::JointTrajectoryPoint>(
          ns + "/traj_rml/joint_traj_point_cmd", 1);

    // subscriber
    sub_joy_ = node->subscribe(joy_name, 1, &Control::cb_joy, this);
    sub_jr3_ = node->subscribe("/jr3/wrench", 1, &Control::cb_jr3, this);
    sub_key_ = node->subscribe("/hybrid/key", 1, &Control::cb_key, this);
    sub_states_ = node->subscribe(ns + "/barrett_manager/wam/joint_states", 1,
                                  &Control::cb_jnt_states, this);
    sub_pose_cmd_ = node->subscribe("/dvrk_mtm/wam_tip_command_pose", 1,
                                    &Control::cb_pose_cmd, this);


    // actionlib
    action_get_blocks_.reset(new actionlib::SimpleActionClient<conman_msgs::GetBlocksAction>(
                        ns + "/scheme/get_blocks_action", true));
    action_set_blocks_.reset(new actionlib::SimpleActionClient<conman_msgs::SetBlocksAction>(
                        ns + "/scheme/set_blocks_action", true));

    if (!action_set_blocks_->waitForServer(ros::Duration(2.0)) ||
        !action_get_blocks_->waitForServer(ros::Duration(2.0)))
    {
      ROS_ERROR_STREAM("Time out waiting for set blocks");
      actions_connected = false;
    } else {
      actions_connected = true;
    }

    // class memeber
    init_kdl_robot();


    // AFEPRC
    force_reading.assign(3,0);
    current_joint_states.assign(7,0);
    ee_pos.assign(3,0);
    ee_ori.push_back(ee_pos);
    ee_ori.push_back(ee_pos);
    ee_ori.push_back(ee_pos);

    plreg = new PlaneRegistration(force_reading, ee_pos, ee_ori);
    adest = new AdaptiveEstimation(0, 0, 0, current_joint_states);

    start_time = ros::Time::now().toSec();

    Vector forward(0.01,0,0);
    Vector backward(-0.01,0,0);

    kb_forward.p = forward;
    kb_backward.p = backward;
  }

  virtual ~Control(){}

  void update()
  {
    // do some update here
    counter_++;
      
    // MODE
    if (ctrl_type_ == CTRL_HYBRID || ctrl_type_ == CTRL_CORRECT)
    {

#if 0
      double vel_gain = 0.1;
      double rot_gain = 0.4;
      Twist cmd_twist = twist_;
      cmd_twist.vel = vel_gain * cmd_twist.vel;
      cmd_twist.rot = rot_gain * cmd_twist.rot;

      // ignore some axes
      cmd_twist.vel(0) = 0.0;  // ignore x
      cmd_twist.rot(1) = 0.0;  // ignore y
      cmd_twist.rot(2) = 0.0;  // ignore z

      cmd_twist = cmd_frame_.M.Inverse(cmd_twist);
      cmd_frame_.Integrate(cmd_twist, 50);
#endif
      
#if 1
      // force control
      double cmd_force = -4.0;
      double err_force = cmd_force - jr3_wrench_.force.z();
      double cmd_v = 0.0;

      cmd_v = -0.0001 * err_force;
      if (cmd_v > 0.0001) cmd_v = 0.0001;
      if (cmd_v < -0.0001) cmd_v = -0.0001;
      if (fabs(cmd_v) < 0.00005) cmd_v = 0.0;

      Frame force_frame;
      force_frame.p(2) = cmd_v;
      cmd_frame_ = cmd_frame_ * force_frame;

#endif

      // AFEPRC
      if (ctrl_type_ == CTRL_CORRECT){
	AFEPRC();
      }
      
      if(is_master_)
      {
        // publish tip command frame
        static tf::TransformBroadcaster br;
        tf::Transform cmd_tf;
        tf::transformKDLToTF(cmd_frame_, cmd_tf);
        br.sendTransform(tf::StampedTransform(cmd_tf, ros::Time::now(), "wam/base_link", "cutter_tip_cmd_position"));
      }
      else
      {
          
        // do ik
        ik_solver_->CartToJnt(jnt_pos_, cmd_frame_, jnt_cmd_);

        // safety chec
        bool violation = false;
        for (size_t i = 0; i < num_jnts_; i++)
        {
            if (fabs(jnt_cmd_(i) - jnt_pos_(i)) > 0.04) {
                violation = true;
                ROS_ERROR_STREAM("jnt cmd violation: " << i << "  error = " << jnt_cmd_(i) - jnt_pos_(i));
            }
        }
        
        // send to robot
        if (!violation) {
            trajectory_msgs::JointTrajectoryPoint msg_jnt_cmd;
            msg_jnt_cmd.positions.resize(num_jnts_);
            for (size_t i = 0; i < num_jnts_; i++)
            {
                msg_jnt_cmd.positions[i] = jnt_cmd_(i);
            }
            pub_jnt_cmd_.publish(msg_jnt_cmd);
        }
      }
    }
  }

private:

  void cb_joy(const sensor_msgs::Joy &msg)
  {
    using namespace KDL;
    twist_.vel(0) = msg.axes[2];
    twist_.vel(1) = msg.axes[1];
    twist_.vel(2) = -msg.axes[0];
    twist_.rot(0) = msg.axes[5];
    twist_.rot(1) = msg.axes[4];
    twist_.rot(2) = -msg.axes[3];

#if DEBUG
    std::cout << "joy: " << twist_ << std::endl;
#endif
  }

  void cb_jr3(const geometry_msgs::WrenchStamped &msg)
  {
    tf::wrenchMsgToKDL(msg.wrench, jr3_wrench_);

    force_reading[0] = jr3_wrench_.force.x();
    force_reading[1] = jr3_wrench_.force.y();
    force_reading[2] = jr3_wrench_.force.z();
  }

  void cb_key(const std_msgs::String &msg)
  {
    ROS_INFO_STREAM("Key = " << msg.data);
    if (msg.data == "r")
      ctrl_ready();
    else if (msg.data == "h")
      ctrl_hybrid();
    else if (msg.data == 'c')
      ctrl_correct();
    else if (msg.data == "m")
      ctrl_manual();
    else if (msg.data == 'i') 
      cmd_frame_ = cmd_frame_ * kb_forward;
    else if (msg.data == 'k')
      cmd_frame_ = cmd_frame_ * kb_backward;
    else
      ROS_ERROR("Unsupported Commands");
  }

  void cb_jnt_states(const sensor_msgs::JointState &msg)
  {
    if (msg.position.size() != num_jnts_) {
      ROS_ERROR_STREAM("Joint states size mismatch  " << msg.position.size());
    }

    for (size_t i = 0; i < num_jnts_; i++) {
      jnt_pos_(i) = msg.position.at(i);
      current_joint_states[i] = msg.position.at(i);
      jnt_vel_(i) = msg.velocity.at(i);
    }

#if DEBUG
    std::cout << "pos: " << jnt_pos_.data.transpose() << std::endl;
    std::cout << "vel: " << jnt_vel_.data.transpose() << std::endl;
    std::cout << std::endl;
#endif
  }

  void cb_pose_cmd(const geometry_msgs::Pose &msg)
  {
    is_new_cmd_ = false;
    KDL::Frame cmd_frame_new;
    tf::poseMsgToKDL(msg, cmd_frame_new);
       
    if (cmd_frame_new.p.Norm() > 0.10 || cmd_frame_new.M.Norm() > 0.10) {
      is_new_cmd_ = false;
    } else {
      is_new_cmd_ = true;
      cmd_frame_ = cmd_frame_ * cmd_frame_new;
    }

  }

  void init_kdl_robot()
  {
    std::string urdf;
    node_->param("/robot_description", urdf, std::string());

    urdf::Model urdf_model;
    urdf_model.initString(urdf);

    // get tree from urdf string
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromString(urdf, my_tree)) {
      ROS_ERROR("Failed to construct kdl tree");
      return;
    }
    std::string rootLink = "wam/base_link";
    std::string tipLink = "wam/cutter_tip_link";
    if (!my_tree.getChain(rootLink, tipLink, robot_))
    {
      ROS_ERROR("Failed to get chain from kdl tree, check root/rip link");
      return;
    }
    num_jnts_ = robot_.getNrOfJoints();

    // resize joint states
    jnt_pos_.resize(num_jnts_);
    jnt_vel_.resize(num_jnts_);
    jnt_cmd_.resize(num_jnts_);
    jnt_limit_min_.resize(num_jnts_);
    jnt_limit_max_.resize(num_jnts_);

    // get jnt limits from URDF model
    size_t i_jnt = 0;
    for (size_t i = 0; i < robot_.getNrOfSegments(); i++) {
      Joint jnt = robot_.getSegment(i).getJoint();
      if (jnt.getType() != Joint::None) {
        jnt_limit_min_(i_jnt) = urdf_model.joints_[jnt.getName()]->limits->lower;
        jnt_limit_max_(i_jnt) = urdf_model.joints_[jnt.getName()]->limits->upper;
        i_jnt++;
      }
    }

    // print limit for debugging
    std::cout << "min: " << jnt_limit_min_.data.transpose() << std::endl;
    std::cout << "max: " << jnt_limit_max_.data.transpose() << std::endl;

    // KDL Solvers
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_));
    ik_solver_.reset(
          new KDL::ChainIkSolverPos_LMA(
            robot_,
            1E-5,
            500,
            1E-15));
  }

  void ctrl_ready()
  {
    ctrl_type_ = CTRL_IDLE;

    // send out ready
    trajectory_msgs::JointTrajectoryPoint msg_jnt_cmd;
    msg_jnt_cmd.positions.resize(num_jnts_);
    msg_jnt_cmd.positions[0] = 0.0;
    msg_jnt_cmd.positions[1] = -M_PI_2;
    msg_jnt_cmd.positions[2] = 0.0;
    msg_jnt_cmd.positions[3] = M_PI_2;
    msg_jnt_cmd.positions[4] = 0.0;
    msg_jnt_cmd.positions[5] = -1.2;
    msg_jnt_cmd.positions[6] = 0.0;
    pub_jnt_cmd_.publish(msg_jnt_cmd);
  }


  void ctrl_hybrid()
  {
    // enter hybrid
    ctrl_type_ = CTRL_HYBRID;
    fk_solver_->JntToCart(jnt_pos_, tip_frame_);
    std::cout << "tip pose = \n" << tip_frame_ << std::endl;

    // sync cmd frame
    cmd_frame_ = tip_frame_;

    // enable pid & traj_rml
    conman_msgs::SetBlocksGoal goal;
    goal.diff = true;
    goal.enable.clear();
    goal.enable.push_back("pid");
    goal.enable.push_back("traj_rml");
    action_set_blocks_->sendGoal(goal);

    if (action_set_blocks_->waitForResult(ros::Duration(5.0))) {
      actionlib::SimpleClientGoalState state = action_set_blocks_->getState();
      ROS_INFO_STREAM(state.toString().c_str());
    } else {
      ROS_WARN_STREAM("Action did not finish before the time out");
      ctrl_type_ = CTRL_IDLE;   // revert back to idle
    }
  }

 void ctrl_correct()
  {
    // enter correction mode
    ctrl_type_ = CTRL_CORRECT;
    fk_solver_->JntToCart(jnt_pos_, tip_frame_);
    std::cout << "tip pose = \n" << tip_frame_ << std::endl;

    // sync cmd frame
    cmd_frame_ = tip_frame_;
  }

  void AFEPRC(){

    // -------  Plane Registration and Correction ----------------

    fk_solver_->JntToCart(jnt_pos_, tip_frame_);

    for (int i = 0; i < 3; i ++){
      for (int j = 0; j < 3; j++){
	std::vector<double> temp(3, 0);
	temp[j] = tip_frame_.M(j,i);
      }
      ee_ori[i] = temp;
      ee_pos[i] = tip_frame_.p(i);
    }
    if (plreg->append_buffer(force_reading, ee_pos, ee_ori)){
      std::vector<double> error1(3,0);
      error1 = plreg->register_plane();
      //  std::cout<<error1[0]<<","<<error1[1]<<","<<error1[2]<<std::endl;
    }

    // --------- Adaptive Estimation --------------------------
    double current_time = ros::Time::now().toSec() - startTime;
    double Fn_now = force_reading[2];
    double Ft_now = force_reading[0];
    adest->append_buff(current_time, Fn_now, Ft_now, current_joint_states);
    AdaptiveEstimation::TaskState current_task_state = adest->run_task_monitor();

    Eigen::Vector2d param;
    double Fest;
    bool is_moving_now;
    adest->get_member_var(param, Fest, is_moving_now);

    //std::cout<<param<<std::endl;
    //std::cout<<"---"<<std::endl;
  
    
  }
 
  void ctrl_manual()
  {
    // enter manual(gravity compensation mode)
    ctrl_type_ = CTRL_MANUAL;

    // clear cmd_frame_

    // disable pid & traj_rml
    conman_msgs::SetBlocksGoal goal;
    goal.diff = true;
    goal.disable.clear();
    goal.disable.push_back("pid");
    goal.disable.push_back("traj_rml");
    action_set_blocks_->sendGoal(goal);

    if (action_set_blocks_->waitForResult(ros::Duration(5.0))) {
      actionlib::SimpleClientGoalState state = action_set_blocks_->getState();
      ROS_INFO_STREAM(state.toString().c_str());
    } else {
      ROS_WARN_STREAM("Action did not finish before the time out");
      ctrl_type_ = CTRL_IDLE;   // revert back to idle
    }
  }

  // ros stuff
  ros::NodeHandle* node_;
  ros::Publisher pub_jnt_cmd_;
  ros::Subscriber sub_joy_;
  ros::Subscriber sub_jr3_;

  ros::Subscriber sub_key_;
  ros::Subscriber sub_states_;
  ros::Subscriber sub_pose_cmd_;

  bool is_master_;
  bool is_simulation_;
  int counter_;

  // conman
  boost::scoped_ptr<actionlib::SimpleActionClient<conman_msgs::GetBlocksAction> > action_get_blocks_;
  boost::scoped_ptr<actionlib::SimpleActionClient<conman_msgs::SetBlocksAction> > action_set_blocks_;
  bool actions_connected;

  // joint states
  JntArray jnt_pos_;   // jnt pos current
  JntArray jnt_vel_;   // jnt vel current
  JntArray jnt_cmd_;   // jnt pos command
  unsigned int num_jnts_;

  // joint limits
  JntArray jnt_limit_min_;
  JntArray jnt_limit_max_;

  // frame
  Frame tip_frame_;    // cutter tip frame
  Frame cmd_frame_;    // cmd cutter tip frame
  bool is_new_cmd_;

  // sensor
  Wrench jr3_wrench_;
  Wrench cmd_wrench_;
  Twist twist_;

  // chain
  Chain robot_;

  // KDL solver
  boost::shared_ptr<ChainFkSolverPos> fk_solver_;
  boost::shared_ptr<ChainIkSolverPos> ik_solver_;

  // ctrl mode
  CONTROL_TYPE ctrl_type_;


  // AFEPRC

  std::vector<double> force_reading;
  std::vector<double> current_joint_states;
  std::vector<double> ee_pos;
  std::vector< std::vector<double> > ee_ori;

  PlaneRegistration* plreg;
  AdaptiveEstimation* adest;

  double start_time;
  
  Frame kb_forward;
  Frame kb_backward;
};



int main(int argc, char *argv[])
{
  using namespace KDL;

  std::cout << "kdl tester" << std::endl;
  ros::init(argc, argv, "kdl_parser_node");
  ros::NodeHandle node;

  // some parameters
  bool is_master = false;
  ros::param::get("~is_master", is_master);

  bool is_simulation = true;
  ros::param::get("~is_sim", is_simulation);

  std::string joy_name = "/spacenav/joy";
  ros::param::get("~joy_name", joy_name);

  if(is_master)
    ROS_INFO("Master Mode");
  else
    ROS_INFO("Slave Mode");

  ros::Rate rate(50);   // 50 hz

  Control ctrl(&node,
               is_master,
               is_simulation,
               joy_name);

  while (ros::ok())
  {
    ros::spinOnce();
    ctrl.update();
    rate.sleep();
  }

  return 0;
}
