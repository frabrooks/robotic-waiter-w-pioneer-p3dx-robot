
void Mover::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    
    float ang = YawFromQuaternion(msg->pose.pose.orientation);
    
    newOdomPose = Pose(msg->pose.pose.position.x, msg->pose.pose.position.y, ang);
    if(!hasInitialPose){
        prevOdomPose = newOdomPose;
        hasInitialPose = true;
    }
    
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
    
    //printf("X: %f, Y: %f, Theta: %f\n", newOdomPose.x, newOdomPose.y, newOdomPose.ang);
}
