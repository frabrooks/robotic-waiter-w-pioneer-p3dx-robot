#include "ParticleF.h"
#include "Structures.h"

#include "ros/ros.h"
#include "math.h"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/Quaternion.h"

geometry_msgs::Quaternion ParticleF::QuaternionFromEuler(double pitch, double roll, double yaw)
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

double ParticleF::YawFromQuaternion(geometry_msgs::Quaternion q)
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

// returns a value between 1 and 0, based on where the value lies between the thresholds
float ClampLerp(float value, float min, float max){
    if(min >= max){
        printf("Can't clamp lerp with min > max!!!\n");
        return 1;
    }
    
    if(value > max){
        return 1;
    }
    if(value < min){
        return 0;
    }
    
    
    float rDiff = max - min;
    float vDiff = value - min;
    return vDiff/rDiff;
    
}



