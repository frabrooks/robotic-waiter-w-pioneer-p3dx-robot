#include "ParticleF.h"
#include "Structures.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include <iostream>

#define POS_NOISE 0.1
#define ANG_NOISE 0.25


Particle randomParticle( void )
{
    Particle p;
    return p;
}

float wsum( Particle* array, int n )
{
    float ws = 0;
    for (int i = 0; i < n; i++){
        ws += array[n].weight;
    }
    return ws;
}

Pose ParticleF::NoisyPose(Pose pose, double posScalar, double angScalar){
    pose.x += randomGen->gaussian01() * posScalar;
    pose.y += randomGen->gaussian01() * posScalar;
    pose.ang += randomGen->gaussian01() * angScalar;
    return pose;
}

void ParticleF::Resample( Particle* sample )
{

    int sample_size = maxParticles;

    // number of samples to keep
    // keep X% of the samples if we fully trust our laser
    double keepPercent = 0.95 + (0.05 * ClampLerp(predictedParticle.weight,0.65,0.9));
    
    const int SAMPLE_KEEP = (int) (keepPercent * sample_size);
    
    float tw = 0;
    for( int i = 0; i < sample_size; i++ )
    {
        // total weight
        tw = sample[i].weight + tw;
    }

    // weight step for threshold
    float ws = tw / (float) SAMPLE_KEEP;

    float rng = randFloat(ws);

    // Forming selection comb
    float* comb = new float[SAMPLE_KEEP];
    for( int i = 0; i < SAMPLE_KEEP; i++ )
    {
        comb[i] = ws * i + rng;
        //printf("%f .. ",comb[i]);
    }
    //printf("\n");

    // we overwrite the new state with
    Particle* keep = newState;

    //printf("%p=%p\n",keep,sample);

    int j = 0;
    float total = sample[j].weight;
    //printf("%d sk||", SAMPLE_KEEP);
    for(int i=0;i<SAMPLE_KEEP;i++){
        while(total < comb[i] /*&& j < SAMPLE_KEEP*/){
            j++;    // move next
            total += sample[j].weight; // increment total
            //printf("%d:t%f|",j,total);
        }
        //printf("\npicked [%d]",j);
        //PrintParticle(sample[j]);
        sample[j].pose = NoisyPose(sample[j].pose,POS_NOISE/RATE,ANG_NOISE/RATE);
        keep[i] =  sample[j];
    }

    for( int i = SAMPLE_KEEP; i < sample_size; i++ )
    {
        // TODO add random values?
        keep[i] = CachedRandomParticle();
        keep[i].weight = 1;
    }

    delete comb;
}

