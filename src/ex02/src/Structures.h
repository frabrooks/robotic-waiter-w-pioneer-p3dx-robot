#ifndef STRUCTURES_H
#define STRUCTURES_H

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

struct Particle {
	public:
	Pose pose;	// motion update
	float weight;	// sensor update
};

struct MotionInfo {
	public:
	Pose previous; // where we thought we were
	Pose predicted; // where we should be in internal coords
};

struct SensorInfo {

  float distance;
  float angle;

};


struct Cluster{

    float weightSum;
    
    Pose sumPose;
};

#endif
