#include "ParticleF.h"
#include "Structures.h"

#include "geometry_msgs/Twist.h"
#include "math.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define MAP_VALID(i, j) ((i >= 0) && (i < map_width) && (j >= 0) && (j < map_height))
#define MAP_INDEX(i, j) ((i) + (j) * map_width)
#define GET_VAL(map_seq, i, j) (map_seq[MAP_INDEX(i,j)])

#define ABSERROR
//#define VOTE


const int LASER_SAMPLE = 50;
// Number of laser points sampled. Adjust so there's a good range but good performance
const float ERROR = 0.4; // + - <number>% error okay

void ParticleF::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{

    float expectedHits = (scan->angle_max - scan->angle_min)/scan->angle_increment;
    for (int i = 0; i < expectedHits; i++)
    {
        SensorInfo si;
        si.distance = scan->ranges[i];
        si.angle = scan->angle_min + i * scan->angle_increment;

        laserReadings[i] = si;
    }

}

Particle ParticleF::SensorUpdate(SensorInfo *info, Particle particle) {


    // is particle in the valid map
    float res = occupancyGrid.info.resolution;
    float x = particle.pose.x / res;
    float y = particle.pose.y / res;
    float ang = particle.pose.ang;
    Pose p = Pose(x, y, ang);
    if (!IsValidPose(p))
    {
        // teleport this somewhere else
        particle = CachedRandomParticle();
    }
#ifdef VOTE
    particle.weight = 1;
#endif
    int lrSize = 725;

    float teethSpacing_f = lrSize / LASER_SAMPLE;
    int teethSpacing = static_cast<int>(teethSpacing_f);
    int comb[LASER_SAMPLE];
    int start = rand() % teethSpacing;

    for (int i = 0; i < LASER_SAMPLE; i++)
    {
        comb[i] = start + teethSpacing * i;
        //printf("%d|",comb[i]);
    }
    //printf("\n");
    
    float maxError = 1;
    float error = 1;
    int total = 0;
    for (int i = 0; i <LASER_SAMPLE; i++)
    {
        SensorInfo si = laserReadings[comb[i]]; // Testing distance-angle
        // measured distance
        float md = si.distance;
        // map -> global pose: scaled pose
        Pose sp;
        sp.x = particle.pose.x / res;
        sp.y = particle.pose.y / res;
        sp.ang = particle.pose.ang + si.angle;
        // calculated distance
        double cd = (
                map_calc_range(sp, &occupancyGrid)
                );

        //printf("cd: %f md: %f \n", cd, md);

        if (md == md && cd == cd)
        {
#ifdef ABSERROR
            error += abs(md-cd);
            if(md > cd){
                maxError += md;
            }else{
                maxError += cd;
            }
#endif  
#ifdef VOTE
            if (cd < md * (1.0 + ERROR) && cd > md * (1.0 - ERROR))
            {
                particle.weight += 1;
            }
            total++;
#endif
        }

    }
#ifdef ABSERROR
    float wx = particle.weight;
    particle.weight = 0.3 * particle.weight;
    //printf("%f|",wx);
    particle.weight += (1.0f - (error / maxError));
    if(particle.weight < 0){
        particle.weight = 0;
    }
#endif
#ifdef VOTE
    int cnt = (int)(LASER_SAMPLE / particle.weight);
    float wx = cnt;
    particle.weight = (5 / log(wx*wx + 1.3));
#endif

    return particle;
}

double ParticleF::map_calc_range(Pose origin, nav_msgs::OccupancyGrid* occupancyGrid) {

    int map_width = occupancyGrid->info.width;
    int map_height = occupancyGrid->info.height;
    double map_origin_x = occupancyGrid->info.origin.position.x;
    double map_origin_y = occupancyGrid->info.origin.position.y;
    double map_resolution = occupancyGrid->info.resolution;
    double max_range = 250.0d;

    //printf("mw %d mh %d mox %f moy %f mr %f maxr %f\n", map_width, map_height, map_origin_x, map_origin_y, map_resolution, max_range);

    // Bresenham raytracing
    int x0,x1,y0,y1;
    int x,y;
    int xstep, ystep;
    char steep;
    int tmp;
    int deltax, deltay, error, deltaerr;

    x0 = origin.x; //(floor((origin.x - map_origin_x) / map_resolution + 0.5) + map_width / 2);
    y0 = origin.y; //(floor((origin.y - map_origin_y) / map_resolution + 0.5) + map_height / 2);

    x1 = origin.x + cos(origin.ang) * max_range; //(floor(((origin.x + max_range * cos(origin.ang)) - map_origin_x) / map_resolution + 0.5) + map_width / 2);
    y1 = origin.y + sin(origin.ang) * max_range; //(floor(((origin.y + max_range * sin(origin.ang)) - map_origin_y) / map_resolution + 0.5) + map_height / 2);

    //printf("x0 %d y0 %d\n", x0, y0);
    //printf("x1 %d y1 %d\n", x1, y1);

    if(abs(y1-y0) > abs(x1-x0)) {
        steep = 1;
    } else {
        steep = 0;
    }

    if(steep)
    {
        tmp = x0;
        x0 = y0;
        y0 = tmp;

        tmp = x1;
        x1 = y1;
        y1 = tmp;
    }

    deltax = abs(x1-x0);
    deltay = abs(y1-y0);
    error = 0;
    deltaerr = deltay;

    x = x0;
    y = y0;

    if(x0 < x1)
        xstep = 1;
    else
        xstep = -1;
    if(y0 < y1)
        ystep = 1;
    else
        ystep = -1;

    if(steep)
    {
        if(!MAP_VALID(y,x) || ( GET_VAL(occupancyGrid->data,y,x) < 0 ) || (GET_VAL(occupancyGrid->data,y,x) > 65))
            return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map_resolution;
    }
    else
    {
        if(!MAP_VALID(x,y) || ( GET_VAL(occupancyGrid->data,x,y) < 0 ) || (GET_VAL(occupancyGrid->data,x,y) > 65))
            return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map_resolution;
    }

    while(x != (x1 + xstep * 1))
    {
        x += xstep;
        error += deltaerr;
        if(2*error >= deltax)
        {
            y += ystep;
            error -= deltax;
        }

        if (steep)
        {
            if(!MAP_VALID(y,x) || ( GET_VAL(occupancyGrid->data,y,x) < 0 ) || (GET_VAL(occupancyGrid->data,y,x) > 65) )
                return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map_resolution;
        }
        else
        {
            if(!MAP_VALID(x,y) || ( GET_VAL(occupancyGrid->data,x,y) < 0 ) || (GET_VAL(occupancyGrid->data,x,y) > 65) )
                return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map_resolution;
        }
    }
    return max_range;
}
