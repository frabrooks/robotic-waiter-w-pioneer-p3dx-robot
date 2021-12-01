#ifndef PARTICLEF_H
#define PARTICLEF_H

#include "Structures.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "ros_random.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>


class ParticleF {
    
    public:
    
    // PARTICLE FILTERING STUFF
    
    std::vector<Pose> cachedValidCells;
    int maxParticles;
    int map_minX;
    int map_maxX;
    int map_minY;
    int map_maxY;
    
    // no params = random seed
    random_numbers::RandomNumberGenerator *randomGen;

    bool IsValidPose(Pose p);
    float randFloat(float max);
    Particle randomParticle( int xmin, int ymin, int xmax, int ymax );
    Particle CachedRandomParticle();
    
    Particle *currentState;
    Particle *newState;
    SensorInfo *laserReadings;

    double map_calc_range(Pose origin, nav_msgs::OccupancyGrid* occupancyGrid);
    Pose latestMotion;
    
    void InitFilter(void);
    void InitSensor(void);
    
    Pose GetAction(void);
    
    void StepFilter(Particle *currentParticles);
    
    Particle MotionUpdate(Pose *action, Particle part);
    float MotionSample(float variance);
    
    Particle SensorUpdate(SensorInfo *info, Particle part);    
    
    void Resample(Particle *currentParticles);
    
    nav_msgs::OccupancyGrid occupancyGrid;

    // OUTPUT
    Particle EstimatePose(void);
    
    void PublishState(void);

    // INITIALISATION
    private:
        bool IsReady(void);
        
        void LoadMapFromService(void);
    	void CreateInitialParticles(void);        
        bool hasMap;
        bool hasParticles;
        bool hasInitialPose;
        
        char const *waitMsg;
        
        void PrintParticle(Particle part);
        
        Particle predictedParticle;

        void QuickResample(Particle* p);
    public:
    geometry_msgs::Quaternion QuaternionFromEuler(double pitch, double roll, double yaw);
    double YawFromQuaternion(geometry_msgs::Quaternion quat);
    
    Pose NoisyPose(Pose pose, double posScalar, double angScalar);

    // MAIN CONTROLLING STUFF
    
    ros::NodeHandle node;
    
    // broadcasters
    tf::TransformBroadcaster map_broadcaster;
    tf::Transform mapTransform;
    
    // publishers
    ros::Publisher vel_pub;
    ros::Publisher particles_pub;
    ros::Publisher estPose_pub;
    ros::Publisher marker_pub;
    
    // subscribers
    ros::Subscriber laser_sub;
    ros::Subscriber motion_sub;
    
    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    void Run();
    
    ParticleF();    // constructor
    ~ParticleF();   // destructor

};

#endif
