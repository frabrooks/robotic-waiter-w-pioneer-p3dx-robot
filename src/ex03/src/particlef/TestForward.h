#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class TestForward{
	public : 
	
    ros::NodeHandle node;
    
    ros::Publisher vel_pub;
    
    void Run();
    
    void Init();

	
};
