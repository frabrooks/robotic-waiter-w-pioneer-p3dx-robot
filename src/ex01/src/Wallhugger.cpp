#include "ros/ros.h"
#include "Wallhugger.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "math.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "twister");

    std::cout << "this is a test" << std::endl;
    
    Wallhugger wh = Wallhugger();
    
    wh.Init();
    
    wh.Run();
	
    return 0;
}

static int logNo = 0;

static void LogMetadata(const sensor_msgs::LaserScan::ConstPtr& scan){
    float angMin = scan->angle_min;
    float angMax = scan->angle_max;
    float increment = scan->angle_increment;
    float delta = scan->scan_time;
    
    logNo++;
    
    // log the metadata of this scan
    std::cout << "log:"<< logNo << "|min:" << angMin << "|max:" << angMax << "|inc:" << increment << "|delta:" << delta << std::endl;
    
    float rngMin = scan -> range_min;
    float rngMax = scan -> range_max;

    std::cout << "close:"<< rngMin << "|far:"<<rngMax << std::endl;
}



void Wallhugger::Init(){
    vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel",100);
    
    laser_sub = node.subscribe<sensor_msgs::LaserScan>("base_scan", 1000, &Wallhugger::laserCallback, this);
    
    walkDir = 0;
    turnDir = 0;
    
    // establish data groups
}


void Wallhugger::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){

    //LogMetadata(scan);  

    float expectedHits = (scan->angle_max - scan->angle_min)/scan->angle_increment;
    
//    std::cout << "hits:" << expectedHits << std::endl;   
    
    // where each segment is roughly a third of the total hits
    float rAvg = 0;
    float fAvg = 0;
    float lAvg = 0;
    
    float rTotal = 0;
    float fTotal = 0;
    float lTotal = 0;
    
    int rCnt = 0;
    int fCnt = 0;
    int lCnt = 0;
    
    float hi = scan->range_min;
    float low = scan->range_max;
 
    float fHi = hi;
    float fLo = low;
    float lHi = hi;
    float lLo = low;
    float rHi = hi;
    float rLo = low;
 
    for(int i=0;i<expectedHits;i++){
        float val = scan->ranges[i];
        if(!CheckValidRange(val)){
            continue;
        }
        
        
        if(i < 242){
            rTotal += val;
            rCnt++;
            if(val > rHi){
                rHi = val;
            }
            else if(val < rLo){
                rLo = val;
            }
        }
        else if(i < 482){
            fTotal += val;
            fCnt ++;
            if(val > fHi){
                fHi = val;
            }
            else if(val < fLo){
                fLo = val;
            }
        }
        else{
            lTotal += val;
            lCnt ++;
            if(val > lHi){
                lHi = val;
            }
            else if(val < lLo){
                lLo = val;
            }
        }
        
        if(val > hi){
            hi = val;
        }
        
        if(val < low){
            low = val;
        }
    }
    
    rCnt = std::max(rCnt,1);
    fCnt = std::max(fCnt,1);
    lCnt = std::max(lCnt,1);
    
    rAvg = rTotal / rCnt;
    fAvg = fTotal / fCnt;
    lAvg = lTotal / lCnt;
    
    //printf("hi-low:%.2f-%.2f|L:%.2f|F:%.2f|R:%.2f\n",hi,low,lAvg,fAvg,rAvg);
    
    printf("LHi:%.2f|LLo:%.2f||FHi:%.2f|FLo:%.2f||RHi:%.2f|RLo:%.2f\n",lHi,lLo,fHi,fLo,rHi,rLo);
    
    const float speed = 0.1f;
    const float turnSpeed = 0.5f;
       
    turnDir = 0;
    walkDir = speed;
    
    float close = 0.8f;
    float danger = 0.6f;
    
    
    if(fLo < danger){
        walkDir = -speed/2;// DANGER DANGER... Reverse very slowly
        
        turnDir = turnSpeed;    // steer towards the most open area
        if(rLo > lLo){
            turnDir = -turnSpeed;
        }
    }
    else if(fLo < close){
        walkDir = 0;    // stop
        turnDir = turnSpeed;    // steer towards the most open area
        if(rLo > lLo){
            turnDir = -turnSpeed;
        }
    }
    else{
        if(lAvg < close){
            turnDir -= turnSpeed;   // steer away
        }
    
        if(rAvg < close){
            turnDir += turnSpeed;    // steer away
        }
    }
    
}

bool Wallhugger::CheckValidRange(float rng){
    // minimum range as reported by sensor
    if(rng < 0.02f){
        return false;
    }
    
    if(rng > 5.5f){
        return false;   // almost max range as reported by sensor, to ignore any out of range values
    }
    // according to IEEE and the internet, this is legit. nan floats return true to this check... WTF?
    if(rng != rng){
        return false;
    }

    return true;
}


void Wallhugger::Run(){
    
    ros::Rate loop_rate(10);
    
    while(ros::ok()){
        geometry_msgs::Twist twistMsg;
      
        twistMsg.angular.z = turnDir;
        twistMsg.linear.x = walkDir;
        
        vel_pub.publish(twistMsg);
        
        ros::spinOnce();
        
        loop_rate.sleep();
    }
}

