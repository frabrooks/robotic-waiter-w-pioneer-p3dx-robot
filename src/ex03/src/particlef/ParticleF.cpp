#include "ParticleF.h"
#include "Structures.h"

#define RATE 10
//#define CALIBRATE

#include "PF_Utils.cpp"

#include "PF_Motion.cpp"
#include "PF_Sensor.cpp"
#include "PF_Sample.cpp"


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "math.h"
#include "ros_random.h"
#include <visualization_msgs/Marker.h>

#include <cstdlib>
#include <ctime>





// Insert code here

int main (int argc, char **argv){
    
    ros::init(argc,argv,"particlef");
    
    ParticleF filter = ParticleF();
    
    filter.InitFilter();
    filter.InitSensor();
    
    filter.Run();

    return 0;
}

void ParticleF::PrintParticle(Particle part){
    printf("p:%.2f,%.2f,%2f||%.2f\n",part.pose.x, part.pose.y, part.pose.ang, part.weight);
}

ParticleF::ParticleF(){
    // default variables
    // not good... ros doesn't understand the 'nullptr' keyword... oh noes
    currentState = NULL;
    newState = NULL;
    hasMap = false;
    hasParticles = false;
    hasInitialPose = false;
    waitMsg = "wait msg";
    
    newOdomPose = Pose(0,0,0);
    prevOdomPose = Pose(0,0,0);
    
    laser_rngMax = 0;
    laser_reliable_raw = 0;
    comb_reliable_raw = 0;
    laser_trust = 0;
    
    latestMotion.x = 0;
    latestMotion.y = 0;
    latestMotion.ang = 0;
    maxParticles = 1500;
        
   // init random number generator
    randomGen = new random_numbers::RandomNumberGenerator();

    
    std::cout << "Particle Filter Created" << std::endl;
}

ParticleF::~ParticleF(){
    delete randomGen;
    if(currentState != NULL){
        delete currentState;
    }
    if(newState != NULL){
        delete newState;
    }
    if(laserReadings != NULL)
    {
      delete laserReadings;
    }
}

void ParticleF::Run(){
    ros::Rate loop_rate(RATE);
    
    while(ros::ok()){
    
        // wait for initialisation to occur
	    if(!IsReady()){
	        // notify why we're not ready
	        std::cout << waitMsg << std::endl;
		    // LOOP
	        ros::spinOnce();
	        loop_rate.sleep();
		    continue;
	    }

        // LOOP
        ros::spinOnce();
        loop_rate.sleep();
        
        // do the filtering
        StepFilter(currentState);
        
        // LOOP
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool ParticleF::IsReady(){

    if(!hasMap){
        LoadMapFromService();
        return false;
    }

    if(!hasParticles){
	CreateInitialParticles();
        return false;
    }
    
    waitMsg = "Ready.";
    return true;
}

void ParticleF::QuickResample(Particle* p){
    for(int i=0;i<maxParticles;i++){
        newState[i] = p[i];
    }
}

void ParticleF::StepFilter(Particle *currentParticles){
    
    
    float sqrDist = SqrDist(prevOdomPose,newOdomPose);
    const float THRESH = 0.05;
    
    float dAng = angDif(newOdomPose.ang,prevOdomPose.ang);
    const float ANG_THRESH = M_PI/36;
    
    if(sqrDist < THRESH*THRESH && dAng < ANG_THRESH){
        // da bestest fake movement!!!
        //newOdomPose.x += 0.033;
        PublishState();
        return;
    }
    
    for(int i=0;i<maxParticles;i++){

        // get current particle
        Particle current = currentParticles[i];
        
        //PrintParticle(current);
        
        // do motion update
        Particle p = MotionUpdate(&latestMotion,current);
        
        //PrintParticle(p);
        
        // do sensor update
        Particle newP = SensorUpdate(laserReadings,p);
        //p.weight = 1;
        
        // store the new particle
        currentParticles[i] = newP;
        
        //PrintParticle(currentParticles[i]);
        //printf("><><><><><\n");
    }
    
    //printf("OLD>>>X: %f, Y: %f, Theta: %f\n", newOdomPose.x, newOdomPose.y, newOdomPose.ang);
    // reset the previous pose
    prevOdomPose = newOdomPose;
    
    // estimate pose before resampling
    predictedParticle = EstimatePose();
    
    // fill the "newState" array
    Resample(currentParticles);
    //QuickResample(currentParticles);
    
    //printf("-----\n");
    // update the state
    PublishState();
    // update the state
    //PublishState();
    
    // swap arrays over
    Particle *temp = newState;      // remember newState pointer
    newState = currentState;        // move currentState pointer into newState
    currentState = temp;            // copy temp pointer into currentState
    
}



void ParticleF::LoadMapFromService(void){
    ros::ServiceClient client = node.serviceClient<nav_msgs::GetMap>("static_map");
    
    if(!client.exists()){
    	waitMsg = "Waiting for map... Service offline.";
        return;
    }
    
    nav_msgs::GetMap srv;    

    if (client.call(srv))
    {
        ROS_INFO("Received Map");
        occupancyGrid = srv.response.map;
        waitMsg = "Got map.";
        hasMap = true;
    }
    else
    {
        ROS_ERROR("Failed to call service");
    }

}

void ParticleF::InitSensor( void )
{

  laserReadings = new SensorInfo[725]; // Might vary not sure
  for(int i=0;i<725;i++){
  
      SensorInfo si;
      si.distance = 0;
      si.angle = i;
      laserReadings[i] = si;
  }

}

void ParticleF::CreateInitialParticles(void){
    
    if(!hasMap){
        waitMsg = "Need map before creating particles.";
        return;
    }
    
    if(hasParticles){
        return;
    }
    
    // work out the min/max practical bounds of the map
    // e.g. a 4k image with loads of empty space is no good for sampling in
    
    // http://docs.ros.org/api/nav_msgs/html/msg/MapMetaData.html
    // http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
    
    int width = (int)occupancyGrid.info.width;      // base type is uint32_t... i doubt we'll have 2^32 pixels per row though...
    int height = (int)occupancyGrid.info.height;
    
    float resolution = occupancyGrid.info.resolution;
    
    int total = width*height;
        
    // cache the valid grid cells for super speedy random generation
    for(int i=0;i<total;i++){
        if(occupancyGrid.data[i] == 0){
            float x = i % width;
            float y = i / width;
            cachedValidCells.push_back(Pose(x*resolution,y*resolution,0));
        }
    }
    
    map_minX = 0;
    map_minY = 0;
    map_maxX = width;
    map_maxY = height;

    currentState = new Particle[maxParticles];
    newState = new Particle[maxParticles];
    
    #ifdef CALIBRATE
    float angle = -(M_PI * 0.5f + 0.125);
    Particle part;
    part.pose = Pose(14.9f,19.426f,angle);
    //part.pose = Pose(0,0,angle);
    part.weight = 1;
    currentState[0] = part;
    newState[0] = part;



    #else
    
    for(int i=0;i<maxParticles;i++){
        
        Particle part;
        // find a valid particle
        part = CachedRandomParticle();
        //printf("PPPP%f",part.weight);
        currentState[i] = part;
        newState[i] = part;
    }
    #endif
    
    PublishState();
    
    hasParticles = true;
    ROS_INFO("Created particle cloud");
    waitMsg = "Got Particles.";
}

void ParticleF::InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data)
{
    if(!IsReady()){
        printf("Can't set initial pose, filter not ready");
        return;
    }
    Pose startPose;
    startPose.x = data->pose.pose.position.x;
    startPose.y = data->pose.pose.position.y;
    startPose.ang = YawFromQuaternion(data->pose.pose.orientation);
    printf("Received pose: (%.1f,%.1f,%.1f)\n", startPose.x, startPose.y, startPose.ang);
    
    const float VARIANCE = 0.75;
    const float ANG_VARIANCE = 0.5; 
    
    // now resample the particles around this pose
    
    for(int i=0;i<maxParticles;i++){
        Pose p = startPose;
        p.x += randomGen->gaussian(0,VARIANCE);
        p.y += randomGen->gaussian(0,VARIANCE);
        p.ang += randomGen->gaussian(0,ANG_VARIANCE);
        currentState[i].pose = p;
        newState[i] = currentState[i];
        
    }
    
    PublishState();

    
}

void ParticleF::InitFilter(void){   

 
    // subscribe to the laser scanner
    laser_sub = node.subscribe<sensor_msgs::LaserScan>("scan", 1000, &ParticleF::LaserCallback, this);
    
    // subscribe motion listener to geometry messages
    motion_sub = node.subscribe<nav_msgs::Odometry>("odom", 1, &ParticleF::OdomCallback, this);
    
    // subscribe to setting an initial pose
    initPose_sub = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1, &ParticleF::InitialPoseCallback, this);
    
    // publish a pose array
    particles_pub = node.advertise<geometry_msgs::PoseArray>("posearray", 1);
    
    // publish where we are for rviz
    amclPose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1);
    // publish our pose for our other programs
    // TODO
    estPose_pub = node.advertise<ex03::ParticleMsg>("pose_estimate", 1);
    
    
    // publish lines
    marker_pub = node.advertise<visualization_msgs::Marker>("debug_particles", 10);
}

void ParticleF::PublishState(void){
    // TODO update the ROS Topics with our output
    
    //printf("TRUST=%.2f||LASER=%.2f|COMB=%.2f\n",laser_trust, laser_reliable_raw, comb_reliable_raw);
    tf::Quaternion q;
    geometry_msgs::TransformStamped staticMsg;
    staticMsg.header.stamp = ros::Time::now();
    staticMsg.header.frame_id = "world";
    staticMsg.child_frame_id = "map";
    
    staticMsg.transform.translation.x = 0;
    staticMsg.transform.translation.y = 0;
    staticMsg.transform.translation.z = 0;
    q.setRPY(0, 0, 0);
    staticMsg.transform.rotation.x = q.x();
    staticMsg.transform.rotation.y = q.y();
    staticMsg.transform.rotation.z = q.z();
    staticMsg.transform.rotation.w = q.w();
    
    static_map_broadcaster.sendTransform(staticMsg);
    
    
    //mapTransform.setOrigin(tf::Vector3(-prevOdomPose.x, -prevOdomPose.y, 0.0) );
    //q.setRPY(0, 0, -prevOdomPose.ang);
    //mapTransform.setRotation(q);
    //map_broadcaster.sendTransform(tf::StampedTransform(mapTransform, ros::Time::now(), "odom", "map"));
    //printf("PUBLISHED\n");
    
    /*
    visualization_msgs::Marker lineList;
    lineList.header.stamp = ros::Time::now();
    lineList.header.frame_id = "/map";
    lineList.action = visualization_msgs::Marker::ADD;
    lineList.type = visualization_msgs::Marker::LINE_LIST;
    lineList.scale.x = 0.05;
    lineList.color.b = 1.0;
    lineList.color.a = 1.0;*/
    
    geometry_msgs::PoseArray poseArray;
    
    poseArray.header.stamp = ros::Time::now();
    poseArray.header.frame_id = "/map";
    
    for(int i=0;i<maxParticles;i++){
        Particle part = currentState[i];
        
        geometry_msgs::Pose pose;
        //PrintParticle(part);
        pose.position.x = part.pose.x;
        pose.position.y = part.pose.y;
        pose.position.z = 0;
        
        pose.orientation = QuaternionFromEuler(0,0,part.pose.ang);
        poseArray.poses.push_back(pose);
        /*
        geometry_msgs::Point pnt;
        pnt.x = part.pose.x;
        pnt.y = part.pose.y;
        pnt.z = 0;
        lineList.points.push_back(pnt);
        
        float DIST = 0.3;
        
        pnt.x += cos(part.pose.ang) * DIST;
        pnt.y += sin(part.pose.ang) * DIST;
        lineList.points.push_back(pnt);*/
        
    }
    //printf("------------\n");
    particles_pub.publish(poseArray);
    //marker_pub.publish(lineList);
    
    geometry_msgs::PoseWithCovarianceStamped stamped;
    stamped.header.stamp = ros::Time::now();
    stamped.header.frame_id = "/map";

    const float SENSOR_X_OFFSET = -0.135;
    float dx = cos(predictedParticle.pose.ang) * SENSOR_X_OFFSET;
    float dy = sin(predictedParticle.pose.ang) * SENSOR_X_OFFSET;
    
    stamped.pose.pose.position.x = predictedParticle.pose.x+dx;
    stamped.pose.pose.position.y = predictedParticle.pose.y+dy;
    stamped.pose.pose.position.z = 0;
    
    stamped.pose.pose.orientation = QuaternionFromEuler(0,0,predictedParticle.pose.ang);
    
    for(int i=0;i<36;i++){
        stamped.pose.covariance[i] = 0;
    }
    
    
    amclPose_pub.publish(stamped);
    
    ex03::ParticleMsg particleMsg;
    particleMsg.header.stamp = ros::Time::now();
    particleMsg.header.frame_id = "/map";
    
    // translate this particle back slightly due to the sensor offset
    
    particleMsg.x = predictedParticle.pose.x + dx;
    particleMsg.y = predictedParticle.pose.y + dy;
    particleMsg.angle = predictedParticle.pose.ang;
    particleMsg.weight = predictedParticle.weight;
    //printf("%.2f%% WEIGHT\n",predictedParticle.weight);
    estPose_pub.publish(particleMsg);
    
    
    
    
}

Particle ParticleF::EstimatePose( void ){
    // decide where the robot is based on the currentState
    
    int maxClusters = 3;
    
    int clusterRadius = 2;
    
    float angleRange = 1.5707f;
    
    // If any cluster surpases this threshold then return that cluster 
    // as the most likely estimate immediately
    float certaintyBreak = 0.6f;
    

    int numOfClusters = 0;
    float weightSum = 0;
    
    Particle *mostLikelyParticle = NULL;
    float highestWeight = 0.0f;
    
    for(int i=0;i<maxParticles;i++){
        
        Particle * p = &currentState[i];
        
        weightSum += p->weight;
        
        if ( p->weight >= highestWeight){
            highestWeight = p->weight;
            mostLikelyParticle = p;
        }
    }

    Cluster * clusters = new Cluster[maxClusters];
    
    Particle * mostLikelyUnprocessedParticle = mostLikelyParticle;
    
    while (numOfClusters < maxClusters){
    
        if (mostLikelyUnprocessedParticle == NULL){
        
            highestWeight = 0.0f;
            for(int i=0;i<maxParticles;i++){
                // Look for most likely particle of unprocessed particles
                Particle * p = &currentState[i];
                if ( p->weight >= highestWeight){
                    highestWeight = p->weight;
                    mostLikelyUnprocessedParticle = p;
                }
            }
        }
        
        // If false then all particles have negative weight (i.e have all been processed)
        if (mostLikelyUnprocessedParticle != NULL){
        
            // most likely remaing particle becomes first in new cluster
            Cluster * c = &clusters[numOfClusters];
            c->weightSum = mostLikelyUnprocessedParticle->weight;
            
            // Calculate weighted sum
            Particle temp = *mostLikelyUnprocessedParticle;
            
            c->sumPose.x = temp.pose.x * temp.weight;
            c->sumPose.y = temp.pose.y * temp.weight;
            c->sumPose.ang = temp.pose.ang * temp.weight;
            
            numOfClusters += 1;
            
            // temporarily negate weight to mark particle as processed
            mostLikelyUnprocessedParticle->weight *= -1.0f;
            
            
            // loop to add all 'within range' particles to this cluster
            bool doAnotherPass = false;
            do{
                doAnotherPass = false;
                for(int i=0;i<maxParticles;i++){
                    
                    Particle * p = &currentState[i];
                    
                    if(p->weight > 0){
                        int xd = abs(p->pose.x - (c->sumPose.x / c->weightSum));
                        int yd = abs(p->pose.y - (c->sumPose.y / c->weightSum));
                    
                        if(sqrt(pow(xd, 2) + pow(yd, 2)) < clusterRadius){
                        
                            float angD = angDif(p->pose.ang, (c->sumPose.ang / c->weightSum));
                            
                            if (angD < angleRange){
                                
                                // Particle is sufficiently close so add to this cluster
                                c->weightSum += p->weight;
                                
                                // Calc weighted sum so we can calc weighted average by
                                // dividing by weightSum of cluster
                                c->sumPose.x += p->pose.x * p->weight;
                                c->sumPose.y += p->pose.y * p->weight;
                                c->sumPose.ang += p->pose.ang * p->weight;
                                
                                // temporarily negate weight to mark particle as processed
                                p->weight *= -1.0f;
                                
                                // when we add a new particle the center of the cluster 
                                // will change which could allow more particles to be absorbed.
                                // Only stop when we loop through all particles without growing
                                doAnotherPass = true;
                                
                            }
                        }
                    }                 
                }
            
            } while (doAnotherPass);
            
            // If this cluster is likely enough then we don't need to bother calculating another
            float certainty = c->weightSum / weightSum;

            if (certainty > certaintyBreak){

                break;
            }
            
            mostLikelyUnprocessedParticle = NULL;
        
        } else {
        
            // All particles processed
            break;
        
        }
        
    
    }
    
    Cluster * mostLikelyCluster = &clusters[0];
    
    for(int i=1; i < numOfClusters; i++){
    
        Cluster * c = &clusters[i];
        
        if(c->weightSum > mostLikelyCluster->weightSum){
            mostLikelyCluster = c;
        }
    
    }
    
    Particle returnParticle = Particle();
    // return weighted average of cluster
    returnParticle.pose.x = mostLikelyCluster->sumPose.x / mostLikelyCluster->weightSum;
    returnParticle.pose.y = mostLikelyCluster->sumPose.y / mostLikelyCluster->weightSum;
    returnParticle.pose.ang = mostLikelyCluster->sumPose.ang / mostLikelyCluster->weightSum;
    returnParticle.weight = mostLikelyCluster->weightSum / weightSum;
    
    delete clusters;
    
    
    for(int i=0;i<maxParticles;i++){
        
        Particle * p = &currentState[i];
        
        if(p->weight < 0){
            p->weight *= -1.0f;
        }
        
    }
    
    return returnParticle;
}





float ParticleF::randFloat( float max )
{
  // Random start point between 0 & ws
  srand (static_cast <unsigned> (time(0))); // seeding
  float rng = static_cast <float> (rand()) / static_cast <float> (RAND_MAX / max);
  
  return rng;
}

Particle ParticleF::CachedRandomParticle(){
    int index = randomGen->uniformInteger(0,cachedValidCells.size()-1);
    
    Particle part;
    part.pose = cachedValidCells[index];
    part.pose.ang = (float)randomGen->uniformReal(0,2 * M_PI);
    part.weight = 0;
    return part;
}

Particle ParticleF::randomParticle( int xmin, int ymin, int xmax, int ymax)
{    
    Particle part;
    Pose pose;

    pose.x = randomGen->uniformInteger(xmin,xmax);
    pose.y = randomGen->uniformInteger(ymin,ymax);
    pose.ang = randomGen->uniformInteger(0,2 * M_PI);

    part.weight = 0;
    part.pose = pose;

    return part;
}

bool ParticleF::IsValidPose(Pose p){
    // check bounds
    if(p.x < map_minX || p.x >= map_maxX){
        return false;
    }
    
    if(p.y < map_minY || p.y >= map_maxY){
        return false;
    }
    
    if(!hasMap){
        return false;
    }
    
    int x = round(p.x);
    int y = round(p.y);
    
    int index = x+(y*map_maxX);
    //printf("index:%d\n",index);
    return occupancyGrid.data[index] == 0;
}
