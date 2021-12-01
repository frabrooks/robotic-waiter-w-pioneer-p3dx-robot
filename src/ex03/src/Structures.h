#ifndef STRUCTURES_H
#define STRUCTURES_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <map>
#include "math.h"
#include <vector>

using namespace std;

bool vectorContains(std::vector<int> v, int x) {
    if(std::find(v.begin(), v.end(), x) != v.end()) {
        return true;
    } else {
        return false;
    }
}

// Used for pathfinding
struct Point {
	public:
	float x;
	float y;
	
	// Calculate Euclidean distance between this point and another
	float distanceTo(Point p2) {
	    return sqrt( pow((x - p2.x), 2) + pow((y - p2.y), 2) );
	}
	
	float distanceTo(geometry_msgs::Point p2) {
	    return sqrt( pow((x - p2.x), 2) + pow((y - p2.y), 2) );
	}
	
	void scalePoint() {
        // Map properties
        // Retrieved from inspecting map properties in RViz
        float resolution = 0.05;
        int mapWidth = 404;
        int mapHeight = 537;

        // From file
        float inputMinX = 0;
        float inputMaxX = 6.018620;
        float inputMinY = -8;
        float inputMaxY = 0;
        
        // Resultant
        float outputMinX = 0;
        float outputMaxX = mapWidth * resolution;
        float outputMinY = 0;
        float outputMaxY = mapHeight * resolution;
        
        float inputRangeX = inputMaxX - inputMinX;
        float inputRangeY = inputMaxY - inputMinY;
        float outputRangeX = outputMaxX - outputMinX;
        float outputRangeY = outputMaxY - outputMinY;
        
        float resultX = outputRangeX * (this->x / inputRangeX);
        // Multiply by -1 to remove negative
        float resultY = -1 * outputRangeY * (this->y / inputRangeY);
        
        this->x = resultX;
        this->y = resultY;
    }
};

bool operator==(const Point& lhs, const Point& rhs) {
        return (lhs.x == rhs.x && lhs.y == rhs.y);
}

struct Triangle {
	public:
	Point p1;
	Point p2;
	Point p3;
	
	Point getCentre() {
		Point centre;
		centre.x = (p1.x + p2.x + p3.x) / 3;
		centre.y = (p1.y + p2.y + p3.y) / 3;
		return centre;
	}
	
	float sign (Point v1, Point v2, Point v3){
	    return (v1.x - v3.x) * (v2.y - v3.y) - (v2.x - v3.x) * (v1.y - v3.y);
	}
	
	bool pointIsWithin(Point p) {
		float d1, d2, d3;
		bool neg, pos;
		
		d1 = sign(p, p1, p2);
		d2 = sign(p, p2, p3);
 		d3 = sign(p, p3, p1);
		
		neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
		pos = (d1 < 0) || (d2 > 0) || (d3 > 0);
		
		return !(neg && pos);
	}
	
	bool sharesEdgeWith(Triangle t2) {
	    int sharedVertices = 0;
	    // Compare all vertices
	    if (p1 == t2.p1 || p1 == t2.p2 || p1 == t2.p3) {
	        sharedVertices++;
	    }
	    if (p2 == t2.p1 || p2 == t2.p2 || p2 == t2.p3) {
	        sharedVertices++;
	    }
	    if (p3 == t2.p1 || p3 == t2.p2 || p3 == t2.p3) {
	        sharedVertices++;
	    }
	    
	    if (sharedVertices == 2) {
	        return true;
	    } else {
	        return false;
	    }
	}
	
	// Calculate Euclidean distance between centre of this triangle and another
	float distanceTo(Triangle t2) {
	    //Point c1 = getCentre();
	    //Point c2 = t2.getCentre();
	    //
	    //return sqrt( pow((c1.x - c2.x), 2) + pow((c1.y - c2.y), 2) );
	    
	    return getCentre().distanceTo(t2.getCentre());
	}
};


/// GRAPH IMPLEMENTATION FROM https://stackoverflow.com/a/15697480
// Vertex for graph
struct Vertex { 
    int id; // id of vertex is its numeric id
    bool obstructed;
    Point location; // location in space of Vertex
    
    //typedef pair<float, Vertex*> ve;
    //vector<ve> adj; // cost of edge, destination Vertex
    std::map<float, Vertex*> adj;
    
    
    // Constructor
    Vertex(int s, bool obstructed, Point p) : id(s), obstructed(obstructed), location(p) {}
    
    float distanceTo(Vertex v2) {
        return location.distanceTo(v2.location);
    }
    
    float distanceTo(geometry_msgs::Point p) {
        return location.distanceTo(p);
    }
    
    float distanceTo(geometry_msgs::Pose p) {
        return location.distanceTo(p.position);
    }
};
// Graph for pathfinding
class Graph
{
public:
    typedef map<int, Vertex*> vmap;
    vmap vertices;
    Vertex getVertex(const int&);
    void addVertex(const int&, const Point);
    void addEdge(const int& from, const int& to, float cost);
    int findClosestVertexTo(geometry_msgs::Pose pose);
    int findClosestVertexTo(geometry_msgs::Point point);
    void updateObstructed(vector<int> obstructedNodes);
};

Vertex Graph::getVertex(const int& id) {
    return *(vertices.find(id)->second);
}

void Graph::addVertex(const int& id, const Point location)
{
    vmap::iterator itr = vertices.find(id);
    if (itr == vertices.end())
    {
        Vertex *v;
        v = new Vertex(id, false, location);
        vertices[id] = v;
        return;
    }
    printf("Vertex already exists!\n");
}

void Graph::addEdge(const int& from, const int& to, float cost)
{
    Vertex *f = (vertices.find(from)->second);
    Vertex *t = (vertices.find(to)->second);
    pair<float, Vertex *> edge = make_pair(cost, t);
    f->adj.insert(edge);
}

int Graph::findClosestVertexTo(geometry_msgs::Point point) {
    int closest;
    float closestDist = std::numeric_limits<float>::max();
    
    // Loop over all vertices
    vmap::iterator itr;
    for(itr = vertices.begin(); itr != vertices.end(); itr++) {
        // Record min distance
        float dist = itr->second->distanceTo(point);
        if (dist < closestDist) {
            closestDist = dist;
            closest = itr->first;
        }
    }    
    
    // Return vertex id of vertex with shortest distance
    return closest;
}

int Graph::findClosestVertexTo(geometry_msgs::Pose pose) {
    int closest;
    float closestDist = std::numeric_limits<float>::max();
    
    // Loop over all vertices
    vmap::iterator itr;
    for(itr = vertices.begin(); itr != vertices.end(); itr++) {
        // Record min distance
        float dist = itr->second->distanceTo(pose);
        if (dist < closestDist) {
            closestDist = dist;
            closest = itr->first;
        }
    }    
    
    // Return vertex id of vertex with shortest distance
    return closest;
}

void Graph::updateObstructed(vector<int> obstructedNodes) {
    vmap::iterator itr;
    for (itr = vertices.begin(); itr != vertices.end(); itr++) {
        if (vectorContains(obstructedNodes, itr->first)) {
            itr->second->obstructed = true;
        } else {
            itr->second->obstructed = false;
        }
    }
}

enum RobotMoveMode {
    Wallhug,
    Path
};

struct Pose {
	public:
	float x;
	float y;
	float ang;
	
	Pose(){
	    x = 0;
	    y = 0;
	    ang = 0;
	}
	
	Pose(float nx, float ny, float nang)
    {
        x= nx;
        y = ny;
        ang = nang;
    }
};

float SqrDist(Pose a, Pose b){
    float x = (a.x-b.x);
    float y = (a.y-b.y);
    return (x*x)+(y*y);
}


struct SensorInfo {
  float distance;
  float angle;
  bool valid;
};

struct Obstacle {
    float distance;
    float angle;
    float localX;
    float localY;
    float worldX;
    float worldY;
  
    Obstacle(){
        distance = 0;
        angle = 0;
        localX = 0;
        localY = 0;
        worldX = 0;
        worldY = 0;
    }
};

struct Vector2 {
    float x;
    float y;
    
    Vector2(float _x, float _y){
        x = _x;
        y = _y;
    }
    
    
    Vector2 operator+(const Vector2& v) const
    {
        return Vector2(x + v.x, y + v.y);
    }
    
    Vector2 operator-(const Vector2& v) const
    {
        return Vector2(x - v.x, y - v.y);
    }
    
    Vector2 operator*(const float f) const
    {
        return Vector2(x*f, y*f);
    }
    
    static float Dot(Vector2 a, Vector2 b) {
        return (a.x * b.x) + (a.y * b.y);  
    }
    
    float SqrMagnitude(void) const{
        return (x*x) + (y*y);
    } 
    
    float Magnitude(void){
        return sqrt(SqrMagnitude());
    }
};


struct MapCost{
    int x;
    int y;
    float worldX;
    float worldY;
    float fixedCost;
    float dynamicCost;
    float inflationCost;
    int parent; // for local pathing
    float dijkstrasCost;
    bool obstructed;
    bool trueObstacle;
    bool inflated;
    bool visited;
    
    int8_t GetColor(void) const{
        if(obstructed || inflated){
            return 127;
        }
        float val = (fixedCost+dynamicCost+inflationCost);
        if(val > 127){
            return 127;
        }
        return (int8_t) val;
    }
};

struct Inflation{
    int indexOffset;
    int costFromCenter;
    bool inflate;
    
    Inflation(){
        indexOffset = 0;
        inflate = false;
        costFromCenter = 0;
    }
    
    Inflation(int index, bool infl, int cost){
        indexOffset = index;
        inflate = infl;
        costFromCenter = cost;
    }
};

struct ObstacleBucket {
    std::vector<Obstacle> obstacles;
};
#endif
