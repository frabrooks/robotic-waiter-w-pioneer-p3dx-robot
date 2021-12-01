#include "ros/ros.h"
#include "TestForward.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Transform.h"
#include "math.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "twister");
    
    TestForward wh = TestForward();
    
    wh.Init();
    
    wh.Run();
	
    return 0;
}

static int logNo = 0;

static void LogMetadata(const sensor_msgs::LaserScan::ConstPtr& scan){
    float angMin = scan->angle_min;
    float angMax = scan->angle_max;
    float increment = scan->angle_increment;
    float delta = scan->scan_time;
    
    logNo++;
    
    // log the metadata of this scan
    std::cout << "log:"<< logNo << "|min:" << angMin << "|max:" << angMax << "|inc:" << increment << "|delta:" << delta << std::endl;
    
    float rngMin = scan -> range_min;
    float rngMax = scan -> range_max;

    std::cout << "close:"<< rngMin << "|far:"<<rngMax << std::endl;
}



void TestForward::Init(){
    vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel",100);
    
    // establish data groups
}



void TestForward::Run(){

    float walkDir = 0.2;
    float turnDir = 0.0;
    
    ros::Rate loop_rate(10);
    while(ros::ok()){
        geometry_msgs::Twist twistMsg;
      	
      	twistMsg.angular.x = 0;
      	twistMsg.angular.y = 0;
        twistMsg.angular.z = 0;
        twistMsg.linear.x = walkDir;
        twistMsg.linear.y = 0;
        twistMsg.linear.z = 0;

        
        
        vel_pub.publish(twistMsg);
	
        ros::spinOnce();
        
        loop_rate.sleep();
    }
}

