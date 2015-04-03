#include <iostream>
#include <math.h>
#include <vector>
#include <Dense>

#define PI 3.1415926

#ifndef PLANE_REGISTRATION_
#define PLANE_REGISTRATION_

class PlaneRegistration{

 public:

  PlaneRegistration( const std::vector<double>& force_reading,
		     const std::vector<double>& ee_pos,
		     const std::vector<std::vector<double> >& ee_ori );
  
  ~PlaneRegistration(){}

  bool append_buffer( const std::vector<double>& force_reading,
		      const std::vector<double>& ee_pos,
		      const std::vector<std::vector<double> >& ee_ori );
  
  // return vector containing angular error around cutter x, y, z respectfully
  std::vector<double> register_plane();
  
 private:

  // calculate an average direction, returns a unit vector
  std::vector<double> calculate_avg_dir(const std::vector<std::vector<double> >& data_points);
  void stiffness_compensation( std::vector<std::vector<double> >& points_after_comp,
			        double& estimated_stiffness);

  // calulate error around cutter x, y, z
  void calculate_misalignment( const std::vector<double>& motion_dir );

  
  //************* Memeber Variables ***********************

  // buffer
  std::vector< std::vector<double> > jr3_buff;
  std::vector< std::vector<double> > cutter_xyz_buff;
  std::vector< std::vector<double> > cutter_xdir_buff;
  std::vector< std::vector<double> > cutter_ydir_buff;
  std::vector< std::vector<double> > cutter_zdir_buff; 


  // current cutter pose
  std::vector<double> current_xyz;
  std::vector<double> current_xdir;
  std::vector<double> current_ydir;
  std::vector<double> current_zdir;
  
  // minimum number of samples in each window
  double window_size;
  // max velocity of cutter motion in m/s
  double v_max;

  // nominal controlled force value for stiffness comp
  double hfp;

  // estimated error around cutter x,y,z in degrees
  std::vector<double> error;
};

#endif
