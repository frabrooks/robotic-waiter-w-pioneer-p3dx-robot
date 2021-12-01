#include "ParticleF.h"
#include "Structures.h"

#include "ros/ros.h"
#include "math.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Transform.h"
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

Pose prevOdomPose;
Pose newOdomPose;



void ParticleF::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    
    float ang = YawFromQuaternion(msg->pose.pose.orientation);
    
    newOdomPose = Pose(msg->pose.pose.position.x, msg->pose.pose.position.y, ang);
    if(!hasInitialPose){
        prevOdomPose = newOdomPose;
        hasInitialPose = true;
    }
    
    printf("X: %f, Y: %f, Theta: %f\n", newOdomPose.x, newOdomPose.y, newOdomPose.ang);
 
}

float ParticleF::MotionSample(float bSquared){

    //return (float)randomGen->gaussian(0,b2);
    // how dis work?
    // rand() with no parameter.... returns any random integer.
    
    float b = sqrt(bSquared);
    float rng;
    float total = 0;
    for(int i = 0; i < 12; i++){
        rng = randFloat(2*b) - b;
        total = total + rng;
        //printf("Rng: %f, Total: %f", rng, total);
    }
    return 0.5*total;
}

Particle ParticleF::MotionUpdate(Pose *action, Particle particle){
   
    // TODO create a new particle (stack allocated pls) with the updated motion state

    
    // Get difference between global odom poses
    float g_xDiff = newOdomPose.x - prevOdomPose.x;
    float g_yDiff = newOdomPose.y - prevOdomPose.y;
    float angDiff = newOdomPose.ang - prevOdomPose.ang; // PROBLEMS WITH ANGLES    
    
    float dTrans = sqrt(g_xDiff*g_xDiff + g_yDiff*g_yDiff);
    // Tried changing to C++'s inbuilt atan2 rather than our aTan2
    float dRot1 = atan2(g_yDiff, g_xDiff) - prevOdomPose.ang;
    float dRot2 = newOdomPose.ang - prevOdomPose.ang - dRot1;
    
    Particle updatedParticle;
    Pose finalPose;
    finalPose.x    = particle.pose.x + (dTrans * cos(particle.pose.ang + dRot1));
    finalPose.y    = particle.pose.y + (dTrans * sin(particle.pose.ang + dRot1));
    finalPose.ang  = particle.pose.ang + dRot1 + dRot2;
    
    updatedParticle.pose = finalPose;
    
    //printf("p:%.1f,%.2f||%.2f\n",update.x, update.y, update.ang);
    
    return updatedParticle;
    
    
    /*
    // USE 0.02
    float alpha1 = 0.04f;
    float alpha2 = 0.04f;
    float alpha3 = 0.03f;
    float alpha4 = 0.03f;
    
    float dTransNoise = dTrans + randomGen->gaussian(0,alpha3*pow(dTrans, 2) + alpha4*pow(dRot1, 2) + alpha4*pow(dRot2, 2));
    float dRot1Noise  = dRot1; //- randomGen->gaussian(0,alpha1*pow(dRot1, 2) + alpha2*pow(dTrans, 2));
    float dRot2Noise  = dRot2; //- randomGen->gaussian(0,alpha1*pow(dRot2, 2) + alpha2*pow(dTrans, 2));
    */
    
    
}
