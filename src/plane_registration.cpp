#include "plane_registration.h"

PlaneRegistration::PlaneRegistration( const std::vector<double>& force_reading,
					const std::vector<double>& ee_pos,
					const std::vector< std::vector<double> >& ee_ori ){

  append_buffer( force_reading, ee_pos, ee_ori);
  window_size = 100;
  v_max = 0.003/100;
  hfp = -3;
  
  error.assign(3,0);
}


bool PlaneRegistration::append_buffer(const std::vector<double>& force_reading,
				      const std::vector<double>& ee_pos,
				      const std::vector< std::vector<double> >& ee_ori){

  std::vector<double> jr3_now = force_reading;
  std::vector<double> point_now = ee_pos;
  current_xyz = ee_pos;

  std::vector<double> xdir_now = ee_ori[0];
  current_xdir = xdir_now;

  std::vector<double> ydir_now = ee_ori[1];
  current_ydir = ydir_now;

  std::vector<double> zdir_now = ee_ori[2];
  current_zdir = zdir_now;
  
  // initialize buffer
  if ( cutter_xyz_buff.size() == 0 ){
    jr3_buff.push_back(jr3_now);
    cutter_xyz_buff.push_back(point_now);
    cutter_xdir_buff.push_back(xdir_now);
    cutter_ydir_buff.push_back(ydir_now);
    cutter_zdir_buff.push_back(zdir_now);
  }
  
  
  std::vector<double> last_pos_in_buffer = cutter_xyz_buff[cutter_xyz_buff.size()-1];
  double dist = pow(last_pos_in_buffer[0] - current_xyz[0],2) +
    pow(last_pos_in_buffer[1] - current_xyz[1],2) +
    pow(last_pos_in_buffer[2] - current_xyz[2],2);
  
  dist = sqrt( dist );
  // use points only if a distance apart
  if (dist > v_max){
    jr3_buff.push_back(jr3_now);
    cutter_xyz_buff.push_back(point_now);
    cutter_xdir_buff.push_back(xdir_now);
    cutter_ydir_buff.push_back(ydir_now);
    cutter_zdir_buff.push_back(zdir_now);
  }

  if (jr3_buff.size() > window_size){    
    jr3_buff.erase(jr3_buff.begin());
    cutter_xyz_buff.erase(cutter_xyz_buff.begin());
    cutter_xdir_buff.erase(cutter_xdir_buff.begin());
    cutter_ydir_buff.erase(cutter_ydir_buff.begin());
    cutter_zdir_buff.erase(cutter_zdir_buff.begin());
    return true;
  }
  else{
    return false;
  }
}


std::vector<double> PlaneRegistration::calculate_avg_dir(const std::vector<std::vector<double> >& data_points){

 std::vector<double> avg_dir(data_points[0].size(), 0);

 
 for (int j = 50; j < data_points.size(); j++){
   double norm = 0;
   std::vector<double> diff(data_points[0].size(), 0);
   for (int i = 0; i < data_points[0].size(); i++){
     diff[i] = (data_points[j][i] - data_points[j-49][i]);
     norm += pow(data_points[j][i] - data_points[j-49][i], 2);
   }
   norm = sqrt(norm);

   for (int i = 0; i < data_points[0].size(); i++){
     // become unit vector and store
     avg_dir[i] += diff[i] / norm;
   }
 }

 double norm = 0;
 for (int i = 0; i < data_points[0].size(); i++){
   avg_dir[i] = avg_dir[i] / data_points.size();
   norm += pow(avg_dir[i],2);
 } 

 norm = sqrt(norm);
for (int i = 0; i < data_points[0].size(); i++){
   avg_dir[i] = avg_dir[i] / norm;
 } 
 
 return avg_dir;
}


std::vector<double> PlaneRegistration::register_plane(){

  std::vector<std::vector<double> > points_after_comp;
  double stiffness_est;
  
  stiffness_compensation(points_after_comp, stiffness_est);

  std::vector<double> motion_dir = calculate_avg_dir(points_after_comp);
  
  calculate_misalignment(motion_dir);
  return error;
}


void PlaneRegistration::stiffness_compensation( std::vector<std::vector<double> >& points_after_comp, double& estimated_stiffness ){

  // set first frame in the window to be stiff. comp. reference
  std::vector<double> local_z = cutter_zdir_buff[0]; 

  std::vector<double> dx, dy, dz, df;
  
  for (int i = 5; i < cutter_xyz_buff.size(); i++){
    dx.push_back(cutter_xyz_buff[i][0] - cutter_xyz_buff[i-5][0]);
    dy.push_back(cutter_xyz_buff[i][1] - cutter_xyz_buff[i-5][1]);
    dz.push_back(cutter_xyz_buff[i][2] - cutter_xyz_buff[i-5][2]);
    df.push_back(jr3_buff[i][2] - jr3_buff[i-5][2]);
  }

  std::vector<double> dist_local_z;
  for (int i = 0; i < dx.size(); i++){
    dist_local_z.push_back(dx[i]*local_z[0] + dy[i]*local_z[1] + dz[i]*local_z[2]);
  }
 
  // fit a line through (dist_local_z, df) to calculate stifness
  std::vector<std::vector<double> > stiffness_data;
  for (int i = 0; i < df.size(); i++){
    std::vector<double> temp;
    temp.push_back(dist_local_z[i]);
    temp.push_back(df[i]);

    stiffness_data.push_back(temp);
  }

  std::vector<double> dir = calculate_avg_dir(stiffness_data); 
  
  estimated_stiffness = dir[1] / dir[0];
  
  for (int i = 0; i < cutter_xyz_buff.size(); i++){
    std::vector<double> point_after_comp(3,0);
    point_after_comp[0] = cutter_xyz_buff[i][0] + (hfp - jr3_buff[i][2])*local_z[0];
    point_after_comp[1] = cutter_xyz_buff[i][1] + (hfp - jr3_buff[i][2])*local_z[1];
    point_after_comp[2] = cutter_xyz_buff[i][2] + (hfp - jr3_buff[i][2])*local_z[2];
    points_after_comp.push_back(point_after_comp);
  }
}

  // currently only implemented to calculate angle between motion_dir and cutter_xdir (rotation around y axis)
void PlaneRegistration::calculate_misalignment(const std::vector<double>& motion_dir){
  
  double angle_error = acos(motion_dir[0]*-(current_xdir[0]) + motion_dir[1]*(-current_xdir[1]) + motion_dir[2]*(-current_xdir[2]));

  std::cout<<motion_dir[0]<<" "<<motion_dir[1]<<" "<<motion_dir[2]<<std::endl;
  //std::cout<<current_xdir[0]<<" "<<current_xdir[1]<<" "<<current_xdir[2]<<std::endl;
  
  error[0] = angle_error*180/PI;
}
