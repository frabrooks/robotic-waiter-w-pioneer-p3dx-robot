#include "GraphBuilder.h"
#include <fstream>
#include <iostream>

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
Graph convertTrianglesToGraph(std::vector<Triangle> triangleList, std::vector<Point> mapPoints) {
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

Graph generateGraph() {
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
    Graph g = convertTrianglesToGraph(triangleList, mapPoints);
    printf("Built graph from triangles!\n");
    return g;
}
