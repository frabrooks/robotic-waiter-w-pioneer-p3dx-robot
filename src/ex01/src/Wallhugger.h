#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class Wallhugger{
	public : 
	
    ros::NodeHandle node;
    
    ros::Publisher vel_pub;
    
    ros::Subscriber laser_sub;
    
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    
    void Run();
    
    void Init();

    bool CheckValidRange(float val);

    private:

    float walkDir;
    float turnDir;
	
};
