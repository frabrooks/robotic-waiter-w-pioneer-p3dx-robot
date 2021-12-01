#ifndef UTILS_H
#define UTILS_H

#include "Structures.h"
#include <cstdlib> 
#include <fstream>
#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "math.h"
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/Quaternion.h"

//namespace Utils {

    template<typename Out>
    void split(const std::string &s, char delim, Out result);
    std::vector<std::string> split(const std::string &s, char delim);

    string intToString(int value);

    std::vector<int> extract_keys(std::map<int, int> const& input_map);

    std::map<int, float> updateMap(std::map<int, float> currentMap, int index, float value);

    bool vectorContains(std::vector<int> v, int x);
    
    


static bool CheckValidRange(float rng){
    // minimum range as reported by sensor
    if(rng < 0.02f){
        return false;
    }
    
    if(rng > 5.5f){
        return false;   // almost max range as reported by sensor, to ignore any out of range values
    }
    // according to IEEE and the internet, this is legit. nan floats return true to this check... WTF?
    if(rng != rng){
        return false;
    }

    return true;
}



static geometry_msgs::Quaternion QuaternionFromEuler(double pitch, double roll, double yaw)
{
    tf::Quaternion tempq = tf::createQuaternionFromRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion quat;
    
    quat.x = tempq[0];
    quat.y = tempq[1];
    quat.z = tempq[2];
    quat.w = tempq[3];
//    printf("Q\n%f,%f,%f,%f\nQ\n",quat.x,quat.y,quat.z,quat.w);
    return quat;
}

static double YawFromQuaternion(geometry_msgs::Quaternion q)
{
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	/*
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);
*/
	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
	return atan2(siny_cosp, cosy_cosp);
}

float angDif(float a, float b){

    return fabsf
            (
                fmodf
                (
                    fabsf(a-b + M_PI),
                    2 * M_PI
                )
                - M_PI        
            );

}


#endif
