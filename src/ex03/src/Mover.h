#ifndef MOVER_H
#define MOVER_H

#include "Structures.h"
#include "Utils.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GridCells.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "ex03/ParticleMsg.h"
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <list>
#include <algorithm>
#include <queue>


class Mover {
    
    private:
    
    // Parameters
    const static int HISTORY_LENGTH = 5;
    const static float MAX_SPEED = 0.4f;
    
    const static float GLOBAL_SPEED = 0.3f;
    const static float LOCAL_SPEED = 0.15f;
    
    const static float LASER_X_OFFSET = 0.135;
    
    
    const static float OBSTACLE_SLOW_DIST = 1.0f;
    const static float OBSTACLE_DETECT_DIST = 3.5f;

    const static float CLOSE_DIST = 0.6;
    const static float FAR_DIST = 0.9;

    const static float GOAL_REACH_DISTANCE = 0.2f;
    const static float FINAL_GOAL_REACH_DISTANCE = 0.3f;
    
    const static float FIND_CERTAINTY_REQUIRED = 0.80;
    const static float LOST_CERTAINTY_REQUIRED = 0.50;
    

    const static float ROBOT_RADIUS = 0.25;
    const static float LOCAL_CAST_RADIUS = 0.27;
    const static float GLOBAL_CAST_RADIUS = 0.35;    
    const static float INFLATE_RADIUS = 0.35f;
    const static float LOOKAHEAD_DISTANCE = 5.0;
    const static float LOCAL_LOOKAHEAD_DISTANCE = 0.5;
    const static int8_t WALKABLE_COL = 65;
    
    
    const static int GLOBAL_PATH = 0;
    const static int LOCAL_PATH = 1;
    const static int LOCAL_AVOID = 2;
    int followMode;
    
    float moveProbe_radius;
    
    float speed;
    float angle;
    bool hasGoal;
    bool localised;
    bool doneMotion;
    
    // Runtime controls
    float obstacleSlowdown;
    
    std::vector<Pose> path;
    
    Pose localGoalPose;
    Pose localPose;
    Pose finalGoalPose;
    Pose avoidancePose;
    float pose_certainty;
    std::list<float> certainty_history;
    
    SensorInfo *laserReadings;
    float laser_MAXRANGE;
    int laserStartIndex,laserEndIndex;
    
    std::vector<Obstacle> obstacles;
    std::vector<float> cleared;
    
    std::vector<int> openCells;
    std::vector<int> nextCells;
    
    
    
    
    

    
    
    // Other
    ros::NodeHandle node;
    
    // broadcasters
    tf::TransformBroadcaster move_broadcaster;
    tf::Transform moveTransform;
    
    
    ros::Publisher vel_pub;
    ros::Publisher obstacles_pub;
    ros::Publisher localPath_pub;
    ros::Publisher costmap_pub;
    ros::Publisher marker_pub;
    ros::Subscriber laser_sub;
    
    ros::Subscriber navGoal_sub;
    ros::Subscriber path_sub;
    
    ros::Subscriber initPose_sub;
    ros::Subscriber currentPose_sub;
    
    // LOCAL PATHING
    nav_msgs::OccupancyGrid occupancyGrid;

    int costmap_width;
    int costmap_height;
    float costmap_resolution;
    int costmap_size;
    
    MapCost *costmap;
    int localPathEnd;
    Pose localPathEndPose;

    // buckets for obstacle searching/storing + discarding
    ObstacleBucket *obstacleBuckets;
    int buckets_XWidth;
    int buckets_YWidth;
    // init variables
    bool hasMap;
    bool hasCostmap;
    
    
    Inflation *inflation;
    int inflate_width;
    int inflate_size;
    
    public:
    void Init(void);
    void Run(void);
    void LocateObstacles(void);
    void Wallhug(void);

    void LineMovement(void);

    
    // LOCAL PATHING
    void InitCostmap(void);
    
    void HardResetCostmap(void);
    void RecalcCostmap(void);
    void InflateAllMap(void);
    void AddInflationAroundObstacle(int obstacleIndex);
    void FindInflationAroundEmpty(int emptyIndex);
    
    Pose WorldPoseFromLocalDeltas(Pose worldBase, float localAngle, float localDistance);
    void PublishCostmap(float radius, Pose pose);
    void MarkObstacleRay(const Pose &origin, float angle, float distance, bool finalCellObstacle, bool clearInflation);
    bool RaycastInflated(const Pose &origin, float angle, float distance);

    void MarkObstacles(void);
    void MarkObstacle(const Pose &worldPose, const Obstacle &localObstacle);
    
    void ClearObstacles(void);
    void ClearObstacle(const Pose &worldBase,float angle);
    
    int FindGoalCell(int startIndex, int fallback);
    std::vector<Pose> DijkstrasCostmap(Pose origin);
    std::vector<Pose> FillCostmap(Pose origin);
    Pose FindNearestOpenCell(Pose origin);

    void GlobalPathMovement(void);
    void LocalPathMovement(void);
    void SimpleMovement(bool beCareful);
        
    float CalculateTurnAngle(Pose origin, Pose target);
    bool FrontObstructed(float dangerDist, float robotHalfWidth);
    bool CheckConeObstructed(float distToTarget, float angleToTarget, float angleTolerance);
    Obstacle ClosestObstacleWithinAngle(float minAngle, float maxAngle);
    
    Obstacle *CircleCastAgainstObstacles(float localAngle, float localDistance, float radius);
    Obstacle *CircleOverlapObstacles(float localAngle, float localDistance, float radius);
    SensorInfo FindAvoidanceAngle(float startAngle,float endAngle,int steps,float dist);
    
    float ClampAngle(float angle);
    Pose ComputeWorldAvoidancePose(Pose worldBase, float localAngle, float localDistance);
    
    // Publishers
    void PublishNavState(void);
    
    // Callbacks
    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool CheckValidRange(float rng, float min, float max);
    void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data);
    void NavGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& data);
    void NewPathCallback(const nav_msgs::Path::ConstPtr& data);
    void CurrentPoseCallback(const ex03::ParticleMsg::ConstPtr& data);
    float ComputeCertainty(void);


};

#endif
