#include "PathFinder.h"
#include "Utils.h"
#include "std_msgs/String.h"
#include <math.h>


// This node listens for the robot's pose estimate and goal destination, and continually publishes a path (list of poses) between the two

// Callback to listen for pose estimate of current location
void PathFinder::poseEstimateCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    estimate = msg->pose.pose;
}

// Callback to listen for goal pose
void PathFinder::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal = msg->pose;
}

// Callback to listen for obstacles
void PathFinder::obsCallback(const nav_msgs::GridCells::ConstPtr& msg){
    obstacles = *msg;
}

float distanceBetween(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
    return sqrt( pow((p1.position.x - p2.position.x), 2) + pow((p1.position.y - p2.position.y), 2) );
}

// Method to convert a vertex path (list of vertex ids in graph) to a navigable path (list of poses)
nav_msgs::Path PathFinder::convertVertexPathToNavPath(Graph g, vector<int> vertexPath) {
    nav_msgs::Path path;
    
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "/map";
    
    path.poses[vertexPath.size()];
    for(int i = 0; i < vertexPath.size(); i++) {
        Vertex v = g.getVertex(vertexPath[i]);
        geometry_msgs::PoseStamped p;
        
        p.header.stamp = ros::Time::now();
        p.header.frame_id = "/map";
        
        p.pose.position.x = v.location.x;
        p.pose.position.y = v.location.y;
        p.pose.position.z = 0;
        
        float ang;
        // If inspecting last vertex, use angle from last time
        if (i == vertexPath.size() - 1) {
            // Check that it is not the only element so that we don't seg fault
            if (i > 0) {
                ang = path.poses[i - 1].pose.orientation.w;
            }
        } 
        // Else calculate angle of bearing
        else {
            Vertex w = g.getVertex(vertexPath[i + 1]);
            float dX = w.location.x - v.location.x;
            float dY = w.location.y - v.location.y;
            ang = atan2((double) dY, (double) dX);
        }
        p.pose.orientation = QuaternionFromEuler(0, 0, ang);        
        
        path.poses.push_back(p);
    }
    return path;
}

vector<int> processObs(nav_msgs::GridCells obstacles, Graph graph){
    vector<int> nodes;
    
    for(int i = 0; i < obstacles.cells.size(); i++) {
        // IS THIS WHERE THE SEG FAULT IS??
        nodes.push_back(graph.findClosestVertexTo(obstacles.cells[i]));
        //printf("%d\n",  graph.findClosestVertexTo(obstacles.cells[i]));
    }
    
    return nodes;
}

nav_msgs::Path PathFinder::pathFind(Graph g, geometry_msgs::Pose estimate, geometry_msgs::Pose goal) {
    int estimateVertex = g.findClosestVertexTo(estimate);
    int goalVertex = g.findClosestVertexTo(goal);
    vector<int> obstructedNodes = processObs(obstacles, g);
    
    g.updateObstructed(obstructedNodes);
    printf("Marked graph nodes as (un)obstructed\n");
    
    vector<int> vertexPath = findVertexPath(g, estimateVertex, goalVertex);
    printf("found vertex path");
    return convertVertexPathToNavPath(g, vertexPath);
}

void PathFinder::Init(void){
    estimate.position.x = 2;
    estimate.position.y = 3;
    estimate.position.z = 0;
    
    goal.position.x = 14;
    goal.position.y = 12;
    goal.position.z = 0;
}

// Method to return a Marker containing the points of the graph, allowing them to be visualised
visualization_msgs::Marker PathFinder::initGraphPoints(Graph g) {
    visualization_msgs::Marker graphPoints;
    graphPoints.header.stamp = ros::Time::now();
    graphPoints.header.frame_id = "/map";       
    graphPoints.action = visualization_msgs::Marker::ADD;
    graphPoints.pose.orientation.w = 1.0;
    graphPoints.color.b = 1.0;
    graphPoints.color.a = 1.0;
    graphPoints.type = visualization_msgs::Marker::POINTS;
    graphPoints.id = 0;
    graphPoints.scale.x = 0.2;
    graphPoints.scale.y = 0.2;      
            
    map<int, Vertex*> vmap = g.vertices;
    map<int, Vertex*>::iterator ivItr;
    for(ivItr = vmap.begin(); ivItr != vmap.end(); ivItr++) {    
        Vertex v = *(ivItr->second);
        geometry_msgs::Point pnt;
        pnt.x = v.location.x;
        pnt.y = v.location.y;
        pnt.z = 0;
        graphPoints.points.push_back(pnt);
    }
    return graphPoints;
}

// Method to return a Marker containing the edges of the graph, allowing them to be visualised
visualization_msgs::Marker PathFinder::initGraphEdges(Graph g) {
    visualization_msgs::Marker graphEdges;
    graphEdges.header.stamp = ros::Time::now();
    graphEdges.header.frame_id = "/map";       
    graphEdges.action = visualization_msgs::Marker::ADD;
    graphEdges.pose.orientation.w = 1.0;
    graphEdges.color.b =  1.0;
    graphEdges.color.a = 1.0;
    graphEdges.type = visualization_msgs::Marker::LINE_LIST;
    graphEdges.id = 1;
    graphEdges.scale.x = 0.1;
            
    map<int, Vertex*> vmap = g.vertices;
    map<int, Vertex*>::iterator ivItr;
    for(ivItr = vmap.begin(); ivItr != vmap.end(); ivItr++) {    
        Vertex v = *(ivItr->second);
        geometry_msgs::Point pnt;
        pnt.x = v.location.x;
        pnt.y = v.location.y;
        pnt.z = 0;
        
        // Iterate over edges of vertex to display them
        std::map<float, Vertex*> adj = v.adj;
        map<float, Vertex*>::iterator fvItr;
        for(fvItr = adj.begin(); fvItr != adj.end(); fvItr++) {   
            graphEdges.points.push_back(pnt);
            Vertex w = *(fvItr->second);
            geometry_msgs::Point pnt2;
            pnt2.x = w.location.x;
            pnt2.y = w.location.y;
            pnt2.z = 0;
            graphEdges.points.push_back(pnt2);
        }
    }
    return graphEdges;
}

void PathFinder::Run() {
    
    poseEstimateSub = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1000, &PathFinder::poseEstimateCallback, this);
    
    goalSub = node.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000, &PathFinder::goalCallback, this);

    obsSub = node.subscribe<nav_msgs::GridCells>("local_obstacles", 1000, &PathFinder::obsCallback, this);

    pathPub = node.advertise<nav_msgs::Path>("path", 1);   

    graphPub = node.advertise<visualization_msgs::Marker>("map_marker", 1);
    
    finishedPub = node.advertise<std_msgs::Bool>("job/advance", 1);

    ros::Rate loop_rate(10);
    
    Graph g = generateGraph();
    
    visualization_msgs::Marker graphPoints = initGraphPoints(g);
    visualization_msgs::Marker graphEdges = initGraphEdges(g);
    
    while (ros::ok()) {
        graphPub.publish(graphPoints);
        graphPub.publish(graphEdges);
        printf("Finding a path from (%f, %f) to (%f, %f)...\n", estimate.position.x, estimate.position.y, goal.position.x, goal.position.y);
        nav_msgs::Path path = pathFind(g, estimate, goal);
        // Print path
        /*for(int i = 0; i < path.poses.size(); i++) {
            printf("(%f, %f)", path.poses[i].pose.position.x, path.poses[i].pose.position.y);
        }*/
        printf("\n");
        // Now publish to topic
        pathPub.publish(path);
        
        std_msgs::Bool finished;
        // Threshold in metres for radius to goal to be considered "at goal"
        const float radius = 1.0f;
        if (distanceBetween(estimate, goal) <= radius) {
            finished.data = true;
            finishedPub.publish(finished);
        } else {
            finished.data = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");

    PathFinder pathfinder = PathFinder();
    pathfinder.Init();
    pathfinder.Run();
    
    return 0;
}
