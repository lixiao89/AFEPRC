#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rosReg.h"

int main(int argc, char **argv)
{


  ros::init(argc, argv, "Register");


  ros::NodeHandle n;
  

  ros::Rate loop_rate(100);

    rosReg* reg = new rosReg(n);

    while (n.ok())
     {
        reg->run();
        ros::spinOnce();
        loop_rate.sleep();

     }
 
  return 0;

}



 


