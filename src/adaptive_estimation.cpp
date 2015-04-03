#include "adaptive_estimation.h"

AdaptiveEstimation::AdaptiveEstimation(const double& current_time,
				       const double& current_Ft,
				       const double& current_Fn,
				       const std::vector<double>& current_joint_states):
  // -- tune --
  obs_upper_bound(100),
  obs_lower_bound(20),
  obs_th(0.2),
  obs_ws(obs_upper_bound),
  obs_increment_shrink(10),
  obs_increment_extend(2),
  tear_th(1.4),
  bunch_th(1.8),
  slide_th(1.8),
  lambda_mu(0.995),
  lambda_Fc(0.995),
  motion_th(0.001),
  // ------------
  Fn_now(current_Fn),
  Ft_now(current_Ft),
  Fest_now(0.58*abs(current_Fn) + 4),
  t_now(current_time),
  current_task_state(NORMAL),
  is_moving_now(false){

  param << 0.58, 4;
  P = Eigen::Matrix2d::Identity();

  param_success = param;
  P_success = P;

  Fn_buff.push_back(current_Fn);
  t1_buff.push_back(current_time);

  joint_states_buff.push_back(current_joint_states);
  t2_buff.push_back(current_time);
  }


void AdaptiveEstimation::append_buff(const double& current_time,
				     const double& current_Fn,
				     const double& current_Ft,
				     const std::vector<double>& current_joint_states){

  Fn_now = current_Fn;
  Ft_now = current_Ft;
  t_now = current_time;
  
  // append joint state buffer for motion detection
  joint_states_buff.push_back(current_joint_states);
  t2_buff.push_back(current_time);
  
  if(joint_states_buff.size() > 300) {
    joint_states_buff.erase(joint_states_buff.begin());
    t2_buff.erase(t2_buff.begin());
  }
  
}



bool AdaptiveEstimation::is_moving(){

  Eigen::MatrixXd vel_buff(joint_states_buff[0].size(), 100 );
  Eigen::MatrixXd avg_joint_vel(joint_states_buff[0].size(), 1);
  
  for(int i = 200; i < joint_states_buff.size(); i++)
    for(int j = 0; j < joint_states_buff[0].size(); j++){
      vel_buff(j,i-200) = ( joint_states_buff[i][j] - joint_states_buff[i-50][j] ) / ( t2_buff[i] - t2_buff[i-50] );
    }
  
  avg_joint_vel = vel_buff.rowwise().sum();

  avg_joint_vel = avg_joint_vel / 100;

  int moving = 0;
  for(int i = 0; i < joint_states_buff[0].size(); i++){
    if(avg_joint_vel(i) > motion_th) { moving = 1; }
  }

  if(moving == 0)  { return false; }
  else             { return true;  }

}


bool AdaptiveEstimation::is_observable(){

  return true;
  
}


void AdaptiveEstimation::evaluate_adaptive_estimator(Eigen::Vector2d& x, Eigen::Matrix2d& Cov){

  Eigen::Matrix2d Lambda;
  Lambda << 1/lambda_mu, 0,
            0, 1/lambda_Fc;
 
  
  Eigen::Vector2d Hk(abs(Fn_now), 1);
 
  Eigen::Vector2d K = Cov*Hk / (Hk.transpose()*Cov*Hk + 1);
 
  x = x + K*(Ft_now - Hk.transpose()*x);
 
  Cov = Lambda*(Eigen::Matrix2d::Identity() - K*Hk.transpose())*Cov*Lambda;
 
}

AdaptiveEstimation::TaskState AdaptiveEstimation::detect_anomaly(){
  
  bool bunch = false;
  bool tear = false;
  bool slide = false;


  AdaptiveEstimation::TaskState state_now;
  
  if(Fn_now < 0 && Ft_now - Fest_now > bunch_th){
    bunch = true;
    state_now = AdaptiveEstimation::BUNCHING;
  }
  else if(Fn_now > 0 && Fest_now - Ft_now > tear_th){
    tear = true;
    state_now = AdaptiveEstimation::TEARING;
  }
  else if(Fn_now < 0 && Fest_now - Ft_now > slide_th){
    slide = true;
    state_now = AdaptiveEstimation::SLIDING;
  }
  else{
    state_now = AdaptiveEstimation::NORMAL;
  }

  return state_now;
  
}


AdaptiveEstimation::TaskState AdaptiveEstimation::run_task_monitor(){

  param = param_success;
  P = P_success;

  is_moving_now = is_moving();
  // perform adaptive estimation when the following are satisfied (should probably add is_in_contact)
  if(current_task_state == AdaptiveEstimation::NORMAL && is_moving_now && is_observable()){
    evaluate_adaptive_estimator(param, P);

    if(param(0) > 0 && param(1) > 0){
      param_success = param;
      P_success = P;
    }
  }

  Fest_now = param(0)*abs(Fn_now) + param(1);

  
  if(is_moving_now) { current_task_state = detect_anomaly(); }
  
  if(current_task_state != AdaptiveEstimation::NORMAL){
    Fest_now = 0;
    P = Eigen::Matrix2d::Zero();
    param = Eigen::Vector2d::Zero();
  }

  return current_task_state;
  
}

void AdaptiveEstimation::get_member_var(Eigen::Vector2d& x, double& Fest, bool& is_moving){

  x = param;
  Fest = Fest_now;
  is_moving = is_moving_now; 
}



