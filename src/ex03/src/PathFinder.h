#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GridCells.h"
#include "std_msgs/Bool.h"
#include <visualization_msgs/Marker.h>
#include "AStar.cpp"
#include "GraphBuilder.cpp"
#include "Structures.h"

class PathFinder {
    
    public:

    geometry_msgs::Pose estimate;
    geometry_msgs::Pose goal;  
    nav_msgs::GridCells obstacles;
    
    ros::NodeHandle node;
    ros::Subscriber poseEstimateSub;
    ros::Subscriber goalSub;
    ros::Subscriber obsSub;
    ros::Publisher pathPub;
    ros::Publisher graphPub;
    ros::Publisher finishedPub;

    void Init(void);
    void Run(void);

    void poseEstimateCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void obsCallback(const nav_msgs::GridCells::ConstPtr& msg);
    
    visualization_msgs::Marker initGraphPoints(Graph g);
    
    visualization_msgs::Marker initGraphEdges(Graph g);
    
    nav_msgs::Path convertVertexPathToNavPath(Graph g, vector<int> vertexPath);
    
    nav_msgs::Path pathFind(Graph g, geometry_msgs::Pose estimate, geometry_msgs::Pose goal);

};
