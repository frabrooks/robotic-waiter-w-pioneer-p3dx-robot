#include <cstdlib> 
#include <fstream>
#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <vector>
#include "Structures.h"
#include "AStar.cpp"
#include "nav_msgs/GetMap.h"

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
//#include "Utils.h"

/// At the moment, this file is just a test for reading in the Triangle map from a file and building a list of Triangles
/// This functionality will be taken and used in pathfinding as a graph can be built from the Triangles

// These two methods are used to split a string on a certain character as apparently C++ doesn't have an inbuilt function for this
template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}
std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

// Build graph from triangles
Graph buildGraph(std::vector<Triangle> triangleList, std::vector<Point> mapPoints) {
    Graph g;
    
    // Initial pass over triangles to add their centres to the graph
    for (int i = 0; i < triangleList.size(); i++) {
        Triangle t = triangleList[i];

        g.addVertex(i, t.getCentre());
    }
    
    // Pass over triangles again to compare them with every other one and look for adjacencies
    for (int i = 0; i < triangleList.size(); i++) {
        Triangle t1 = triangleList[i];

        for (int j = 0; j < triangleList.size(); j++) {
            Triangle t2 = triangleList[j];
        
            // Rule out comparing triangle with itself
            if (i != j) {
                // Compare vertices
                if (t1.sharesEdgeWith(t2)) {
                    g.addEdge(i, j, t1.distanceTo(t2));
                }
            }
        }
    }
    
    return g;
}

void publishPath(Graph g, std::vector<int> path, ros::NodeHandle node, std::vector<Triangle> triList) {
    ros::Publisher map_marker_pub, path_marker_pub, tri_marker_pub;
    map_marker_pub = node.advertise<visualization_msgs::Marker>("map_marker", 1);   
    path_marker_pub = node.advertise<visualization_msgs::Marker>("path_marker", 1);
    tri_marker_pub = node.advertise<visualization_msgs::Marker>("tri_marker", 1);
    
    ros::Rate r(30);
    while(ros::ok()){
        visualization_msgs::Marker mapPoints, mapEdges;
        mapPoints.header.stamp = mapEdges.header.stamp = ros::Time::now();
        mapPoints.header.frame_id = mapEdges.header.frame_id = "/map";       
        mapPoints.action = mapEdges.action = visualization_msgs::Marker::ADD;
        mapPoints.pose.orientation.w = mapEdges.pose.orientation.w = 1.0;
        mapPoints.color.b = mapEdges.color.b =  1.0;
        mapPoints.color.a = mapEdges.color.a = 1.0;
        // Sort out map points
        mapPoints.type = visualization_msgs::Marker::POINTS;
        mapPoints.id = 0;
        mapPoints.scale.x = 0.2;
        mapPoints.scale.y = 0.2;      
        // Sort out map edges
        mapEdges.type = visualization_msgs::Marker::LINE_LIST;
        mapEdges.id = 1;
        mapEdges.scale.x = 0.1;
                
        map<int, Vertex*> vmap = g.vertices;
        map<int, Vertex*>::iterator ivItr;
        for(ivItr = vmap.begin(); ivItr != vmap.end(); ivItr++) {    
            Vertex v = *(ivItr->second);
            geometry_msgs::Point pnt;
            pnt.x = v.location.x;
            pnt.y = v.location.y;
            pnt.z = 0;
            mapPoints.points.push_back(pnt);
            
            // Iterate over edges of vertex to display them
            std::map<float, Vertex*> adj = v.adj;
            map<float, Vertex*>::iterator fvItr;
            for(fvItr = adj.begin(); fvItr != adj.end(); fvItr++) {   
                mapEdges.points.push_back(pnt);
                Vertex w = *(fvItr->second);
                geometry_msgs::Point pnt2;
                pnt2.x = w.location.x;
                pnt2.y = w.location.y;
                pnt2.z = 0;
                mapEdges.points.push_back(pnt2);
            }
        }
        

        // Sort out path points
        visualization_msgs::Marker pathPoints, pathStrip;
        pathPoints.header.stamp =  pathStrip.header.stamp = ros::Time::now();
        pathPoints.header.frame_id = pathStrip.header.frame_id = "/map";
        pathPoints.type = visualization_msgs::Marker::POINTS;
        pathPoints.action = pathStrip.action = visualization_msgs::Marker::ADD;
        pathPoints.pose.orientation.w = pathStrip.pose.orientation.w = 1;
        pathPoints.id = 2;
        pathPoints.scale.x = 0.2;
        pathPoints.scale.y = 0.2;
        pathPoints.color.r = pathStrip.color.r = 1.0;
        pathPoints.color.a = pathStrip.color.a =  1.0;       
        
        // Sort out path strip
        pathStrip.type = visualization_msgs::Marker::LINE_STRIP;
        pathStrip.id = 3;
        pathStrip.scale.x = 0.2;       
       
        vector<int>::iterator iItr;
        for(iItr = path.begin(); iItr != path.end(); iItr++) {    
            Vertex v = g.getVertex(*iItr);            
            geometry_msgs::Point pnt;
            pnt.x = v.location.x;
            pnt.y = v.location.y;
            //printf("x:%f, y:%f\n", itr->second->location.x, itr->second->location.y);
            pnt.z = 0;
            pathPoints.points.push_back(pnt);
            pathStrip.points.push_back(pnt);
            
        }
        
        visualization_msgs::Marker triPoints, triEdges;
        triPoints.header.stamp = triEdges.header.stamp = ros::Time::now();
        triPoints.header.frame_id = triEdges.header.frame_id = "/map";       
        triPoints.action = triEdges.action = visualization_msgs::Marker::ADD;
        triPoints.pose.orientation.w = triEdges.pose.orientation.w = 1.0;
        triPoints.color.g = triEdges.color.g =  1.0;
        triPoints.color.a = triEdges.color.a = 1.0;
        // Sort out map points
        triPoints.type = visualization_msgs::Marker::POINTS;
        triPoints.id = 0;
        triPoints.scale.x = 0.2;
        triPoints.scale.y = 0.2;      
        // Sort out map edges
        triEdges.type = visualization_msgs::Marker::LINE_LIST;
        triEdges.id = 1;
        triEdges.scale.x = 0.1;
                
        std::vector<Triangle>::iterator tItr;
        for(tItr = triList.begin(); tItr != triList.end(); tItr++) {    
            Triangle t = *tItr;
            geometry_msgs::Point pnt1;
            pnt1.x = t.p1.x;
            pnt1.y = t.p1.y;
            pnt1.z = 0;
            geometry_msgs::Point pnt2;
            pnt2.x = t.p2.x;
            pnt2.y = t.p2.y;
            pnt2.z = 0;
            geometry_msgs::Point pnt3;
            pnt3.x = t.p3.x;
            pnt3.y = t.p3.y;
            pnt3.z = 0;
            triPoints.points.push_back(pnt1);
            triPoints.points.push_back(pnt2);
            triPoints.points.push_back(pnt3);
            
            triEdges.points.push_back(pnt1);
            triEdges.points.push_back(pnt2);
            triEdges.points.push_back(pnt2);
            triEdges.points.push_back(pnt3);
            triEdges.points.push_back(pnt3);
            triEdges.points.push_back(pnt1);
        }
        
        tri_marker_pub.publish(triPoints);
        tri_marker_pub.publish(triEdges);
        map_marker_pub.publish(mapPoints);
        map_marker_pub.publish(mapEdges);
        path_marker_pub.publish(pathPoints);
        path_marker_pub.publish(pathStrip);        
        
        ros::spinOnce();
        
        r.sleep();
    }
}

int main (int argc, char **argv){
    ros::init(argc, argv, "talker");
    ros::NodeHandle node;
	// Get file
    std::ifstream infile("./bags/triangle_map_3.obj");

    if (infile.fail()) { std::cout << "FUCK: can't read triangles" << '\n'; } else { std::cout << "BLOODY NICE ONE: successfully read triangles" << '\n'; }
    
    //LoadMapFromService(node);
    
    std::string line;
    std::vector<Point> mapPoints;
    std::vector<Triangle> triangleList;
    while (std::getline(infile, line))
    {
		// Make new input stream
        std::istringstream stream(line);
       	    
	    // If line represents a vertex
	    if (line[0] == 'v') {
	        // Split line on space character
	        std::vector<std::string> values = split(line, ' ');
	        Point p;
			
			char *ending;
			// First entry in file is x
	        p.x = std::strtof(values[1].c_str(), &ending);
	        // Use z from file as y of our point
	        p.y = std::strtof(values[3].c_str(), &ending);    
	      	// Scale point from Blender coords to our actual coords
	        p.scalePoint();
	        
	        //printf("X: %f, Y: %f\n", p.x, p.y);
	      
			// Add to list
	        mapPoints.push_back(p);
	    }
        
		// If line represents a face
	    if (line[0] == 'f') {
	        // Split line on space character
	        std::vector<std::string> values = split(line, ' ');
	        Triangle r;
	        
			// Lookup points in points list
			// Subtract 1 when finding index of point in list
			// as points are indexed from 1 and array from 0
	        r.p1 = mapPoints[atoi(values[1].c_str()) - 1];
	        r.p2 = mapPoints[atoi(values[2].c_str()) - 1];
	        r.p3 = mapPoints[atoi(values[3].c_str()) - 1];
	        
			// Add to list
	        triangleList.push_back(r);
	    }	   
    }

    printf("Building graph now...\n");
    // Now make graph, given triangles and points
    Graph g = buildGraph(triangleList, mapPoints);
    printf("Built graph from triangles!\n");
        
    // Print testing graph
        /*float minx = 0;
        float miny = 0;
	    map<int, Vertex *>::iterator itr;
	    for (itr = g.vertices.begin(); itr != g.vertices.end(); itr++ ) {
            Vertex v = *(itr->second);
            if(v.location.x > minx){
                minx = v.location.x;
            }
            if(v.location.y > miny){
                miny = v.location.y;
            }
            //printf("ID: %d - X: %f, Y: %f\n", v.name, v.location.x, v.location.y);
        }
    printf("%f", minx);
    printf("%f", miny);*/
	
	//printf("number of nodes = %d\n", g.vertices.size());

	std::vector<int> path = findVertexPath(g, 300, 50);
	
	for(std::vector<int>::iterator it = path.begin(); it != path.end(); ++it) {
        printf("%d\n", *it);
    }
    
    publishPath(g, path, node, triangleList);
    
    return 0;
}
