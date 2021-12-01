#include "Mover.h"

#include "nav_msgs/GetMap.h"
#include "math.h"
#include "Mover_Obstacles.cpp"
#include "Mover_Costmap.cpp"

char const *waitMsg = "";
bool hasMap;

int main(int argc, char **argv){

    ros::init(argc,argv,"ex3_mover");
    
    Mover mover = Mover();
    
    mover.Init();
    
    mover.Run();
    
    return 0;
}

void Mover::Init(void){
    
    speed = 0;
    angle = 0;
    obstacleSlowdown = 1;
    localPose = Pose();
    localGoalPose = Pose();
    finalGoalPose = Pose();
    
    laserReadings = new SensorInfo[725];
    
    for(int i=0;i<725;i++){
        laserReadings[i].distance = 0;
        laserReadings[i].angle = 0;
        laserReadings[i].valid = false;
    }

    laserStartIndex = 0;
    laserEndIndex = 725;
    localPathEnd = 0;
    followMode = GLOBAL_PATH;

    obstacles.reserve(725);
    cleared.reserve(725);
    
    vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    obstacles_pub = node.advertise<nav_msgs::GridCells>("local_obstacles", 1);
    
    costmap_pub = node.advertise<nav_msgs::GridCells>("global_costmap", 1);
    
    // publish lines
    marker_pub = node.advertise<visualization_msgs::Marker>("local_goal", 1);
    
    localPath_pub = node.advertise<visualization_msgs::Marker>("local_path", 1);
    
    laser_sub = node.subscribe<sensor_msgs::LaserScan>("scan", 100, &Mover::LaserCallback, this);
    
    navGoal_sub = node.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal",1,&Mover::NavGoalCallback, this);
    
    path_sub = node.subscribe<nav_msgs::Path>("path",1,&Mover::NewPathCallback,this);
    
    initPose_sub = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1, &Mover::InitialPoseCallback, this);
    
    currentPose_sub = node.subscribe<ex03::ParticleMsg>("pose_estimate",1, &Mover::CurrentPoseCallback, this);
    
    hasMap = false;
    hasCostmap = false;
    
}

/*
*   This function only wraps around once
*/
float Mover::ClampAngle(float angle){
    if(angle < -M_PI){
        return angle + 2*M_PI;
    }
    
    if(angle > M_PI){
        return angle - 2*M_PI;
    }
    return angle;
}

float Mover::CalculateTurnAngle(Pose origin, Pose target){

    // work out which way to turn so that we face our target position
    float dx = target.x - origin.x;
    float dy = target.y - origin.y;
    float angToTarget = atan2(dy,dx);
    float angleDelta = angToTarget - origin.ang;
    angleDelta = ClampAngle(angleDelta);
    
    // this is the absolute angle required to turn
    return angleDelta;
}


void Mover::Wallhug(){

    // don't slow down because of obstacles... we're localising.
    obstacleSlowdown = 1;

    Obstacle rightObst = ClosestObstacleWithinAngle(-M_PI,-0.5f);

    bool frontObstructed = FrontObstructed(CLOSE_DIST,ROBOT_RADIUS);
    
    
    // early out if obstacle
    if(frontObstructed){
        printf("LOCALISING:Obstructed\n");
        speed = 0;
        angle = 0.5f;
        return;
    }
    
    if(rightObst.distance < CLOSE_DIST){
        printf("LOCALISING:Too Close\n");
        speed = 0.2f;
        angle = 0.5f;
    }
    else if(rightObst.distance > FAR_DIST){
        printf("LOCALISING:Too Far\n");
        speed = 0.3f;
        angle = -0.5f;
    }
    else{
        printf("LOCALISING:Normal\n");
        speed = MAX_SPEED;
        angle = 0.0f;
    }
}

void Mover::LineMovement(){
   if(!hasGoal){
        printf("READY!:%.2f%%No Goal Set\n",obstacleSlowdown);
        speed = 0.0f;
        angle = 0.0f;
        return;
    }
    
    float distToTarget = SqrDist(localPose,localGoalPose);
    float closeDistSqr = GOAL_REACH_DISTANCE * GOAL_REACH_DISTANCE;
    
    if(distToTarget < closeDistSqr){
        hasGoal = false;
        speed = 0.0f;
        angle = 0.0f;
        printf("!!!! GOAL REACHED !!!!\n");
        return;
    }
    
    
    
    float angleToTarget = CalculateTurnAngle(localPose,localGoalPose);

    bool frontObstructed = FrontObstructed(CLOSE_DIST,ROBOT_RADIUS);

    float absAngle = fabsf(angleToTarget);

    angle = angleToTarget * 0.5f;
    
    if(absAngle < 0.5){
        speed = 0.3f;
    }
    else{
        speed = 0.0f;
        printf("READY!:%.2f%%Turning...\n",obstacleSlowdown);
        return;
    }
    

    
    if(frontObstructed){
        angle = 0.0f;
        speed = 0.0f;
        printf("READY!:%.2f%%Blocked!\n",obstacleSlowdown);
    }
    else{
        printf("READY!:%.2f%%Following...\n",obstacleSlowdown);
    }

}

void Mover::SimpleMovement(bool beCareful){
    
    float distToFinal = SqrDist(localPose,finalGoalPose);
    float closeFinalDistSqr = FINAL_GOAL_REACH_DISTANCE * FINAL_GOAL_REACH_DISTANCE;
    
    if(distToFinal < closeFinalDistSqr){
        speed = 0.0f;
        angle = 0.0f;
        printf("!!!! GOAL REACHED !!!!\n");
        doneMotion = true;
        return;
    }
        
    float distToTarget = SqrDist(localPose,localGoalPose);
    float closeDistSqr = GOAL_REACH_DISTANCE * GOAL_REACH_DISTANCE;
    
    if(distToTarget < closeDistSqr){
        speed = 0.0f;
        angle = 0.0f;
        printf("LOCAL GOAL REACHED\n");
        doneMotion = true;
        return;
    }
    doneMotion = false;
    
    float angleToTarget = CalculateTurnAngle(localPose,localGoalPose);
    
    float absAngle = fabsf(angleToTarget);

    angle = angleToTarget;

    float angleThresh = 0.5f;
    
    speed = GLOBAL_SPEED;
    
    if(absAngle > angleThresh){
        speed = 0.0f;
        printf("READY!:%.2f%% Turning...\n",obstacleSlowdown);
        return;
    }
    
    float estimatedSpeed = speed * obstacleSlowdown;
    
    bool frontObstructed = FrontObstructed(0.1+(estimatedSpeed),ROBOT_RADIUS);
    
    if(frontObstructed){
        speed = -0.1f;
        printf("READY!:%.2f%% FRONT Blocked!\n",obstacleSlowdown);
    }
    else{
        printf("READY!:%.2f%% Following...\n",obstacleSlowdown);
    }

}


Pose Mover::ComputeWorldAvoidancePose(Pose worldBase, float localAngle, float localDistance){
    
    // default to base pose
    Pose result = worldBase;   
    
    float avoidDist = ROBOT_RADIUS - localDistance;
    
    // work out global offsets
    float worldAng = ClampAngle(worldBase.ang + localAngle);
    float world_dx = cos(worldAng)*avoidDist;
    float world_dy = sin(worldAng)*avoidDist;
    
    // calculate new world pose
    result.x = worldBase.x + world_dx;
    result.y = worldBase.y + world_dy;
    result.ang = worldAng;
    
    return result;
}

SensorInfo Mover::FindAvoidanceAngle(float startAngle,float endAngle,int steps,float distance){
    
    float diff = ClampAngle(endAngle - startAngle);
    SensorInfo si;
    si.distance = distance;
    for(int i=0;i<steps;i++){
        float scanAngle = startAngle + diff*i;
        // first check for valid map pose
        
        Obstacle *hit = CircleCastAgainstObstacles(scanAngle,distance,GLOBAL_CAST_RADIUS);
        
        // free circle cast! let's do this
        if(hit == NULL){
            si.angle = scanAngle;
            si.valid = true;
            return si;
        }
        
    }
    si.angle = startAngle;
    si.valid = false;
    return si;
}

void Mover::GlobalPathMovement(){

    if(path.size() == 0){
        printf("NO PATH\n");
        localGoalPose = localPose;
        SimpleMovement(false);
        return;
    }
    
    // got a list of positions to move to... try path skipping
    int failureCount = 0;
    int targetIndex = 0;
    int lastClearIndex = -1;
    
    SensorInfo rightOpen;
    rightOpen.angle = localPose.ang;
    rightOpen.distance = 99999;
    rightOpen.valid = false;
    SensorInfo leftOpen = rightOpen;
    
    for(targetIndex=0;targetIndex < path.size();targetIndex++){
        
        
        Pose pathPose = path[targetIndex];
        
        float angle = CalculateTurnAngle(localPose, pathPose);
        float dist = sqrt(SqrDist(pathPose,localPose));
        
        if(dist > LOOKAHEAD_DISTANCE){
            break;
        }
        
        Obstacle *obst = CircleCastAgainstObstacles(angle,dist,GLOBAL_CAST_RADIUS);
        
        if(obst == NULL){
        
            float worldAngle = ClampAngle(localPose.ang + angle);
            if(!RaycastInflated(localPose,worldAngle,dist)){
                lastClearIndex = targetIndex;
            }
        }
        /*
        else{
            // find left and right furthest valid points
            SensorInfo newLeftAvoid = FindAvoidanceAngle(angle,angle+2,40,dist);
            SensorInfo newRightAvoid = FindAvoidanceAngle(angle,angle-2,40,dist);
            
            if(newLeftAvoid.valid){
                leftOpen = newLeftAvoid;
            }
            if(newRightAvoid.valid){
                rightOpen = newRightAvoid;
            }
        }*/
        
    }
    
    localPathEnd = targetIndex-1;

    
    bool blocked = lastClearIndex == 0 && path.size() > 1;
    
    if(lastClearIndex == -1 || blocked){
        printf(".GLOBAL PATH OBSTRUCTED.\n");
        followMode = LOCAL_PATH;
        localGoalPose = path[0];
        localPathEndPose = path[localPathEnd];
        lastClearIndex = 0;  
    }
    else{
        printf("PATH INDEX=%d\n",lastClearIndex);
        // find our local goal pose
        localGoalPose = path[lastClearIndex];
    }

    SimpleMovement(false);

}

void Mover::LocalPathMovement(){

    // Try to switch back to global once we can see our target
    float localAngle = CalculateTurnAngle(localPose, localPathEndPose);
    float localDist = sqrt(SqrDist(localPose,localPathEndPose));
    float worldAngle = ClampAngle(localPose.ang + localAngle);
    //printf("%.3f,%.3f\n",localAngle,localDist);
    
    if(!RaycastInflated(localPose,worldAngle,localDist)){
        followMode = GLOBAL_PATH;
    }

    vector<Pose> localPath = DijkstrasCostmap(localPose);
    
    int count = -1;
    for(int i=localPath.size()-1;i>=0;i--){
        Pose pathPose = localPath[i];
        
        float localAngle = CalculateTurnAngle(localPose, pathPose);
        float localDist = sqrt(SqrDist(pathPose,localPose));
        float worldAngle = ClampAngle(localPose.ang + localAngle);
        //printf("%.3f,%.3f\n",localAngle,localDist);

        
        if(localDist > LOCAL_LOOKAHEAD_DISTANCE){
            break;
        }
        
        if(!RaycastInflated(localPose,worldAngle,localDist)){
            localGoalPose = pathPose;
            count++;
        }


    }
    
    if(count >= 0){
        printf("Following local Path @%d\n",count);
    }
    else{
        printf(".LOCAL PATH OBSTRUCTED.\n");
        
        if(localPath.size() == 0){
            followMode = GLOBAL_PATH;
            printf("NO LOCAL PATH FOUND.... Retrying Global.\n");
        }
        

        
        HardResetCostmap();
        localGoalPose = FindNearestOpenCell(localPose);
    }
    
    SimpleMovement(true);
}

void Mover::Run(void){
    
    ros::Rate loop_rate(20);
    
    while(ros::ok()){
        
        
        // We need a map!
        if(!hasCostmap){
            InitCostmap();
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        
        LocateObstacles();
        
        printf("C:%.2f%%\n",pose_certainty);
        // do simple movement if we're certain where we are
        if(pose_certainty > FIND_CERTAINTY_REQUIRED){
            ClearObstacles();
            MarkObstacles();
            RecalcCostmap();
            localised = true;
        }
        else if(pose_certainty < LOST_CERTAINTY_REQUIRED){
            localised = false;
        }
        
        if(localised){
            switch(followMode){
                case GLOBAL_PATH:
                    GlobalPathMovement();
                break;
                
                case LOCAL_PATH:
                    LocalPathMovement();
                break;
            }
        }
        else {
            Wallhug();
        }

        geometry_msgs::Twist twistMsg = geometry_msgs::Twist();
      	
      	if(angle > 0.5){
      	    angle = 0.5;
      	}
      	else if(angle < -0.5){
      	    angle = -0.5;
      	}
        twistMsg.angular.z = angle;
        twistMsg.linear.x = speed * obstacleSlowdown;
        //printf("MOTION:%.1f,%.1f,%.1f\n", twistMsg.angular.z, twistMsg.linear.x, obstacleSlowdown);
        
        // assuming constant velocity, predict motion
        //PredictMotion(twistMsg);
        
        vel_pub.publish(twistMsg);
        
        PublishNavState();
        
        ros::spinOnce();
        
        loop_rate.sleep();
    }
}

void Mover::PublishNavState(void)
{

    moveTransform.setOrigin(tf::Vector3(localPose.x, localPose.y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, localPose.ang);
    moveTransform.setRotation(q);
    move_broadcaster.sendTransform(tf::StampedTransform(moveTransform, ros::Time::now(), "map", "mover"));

    
    
    visualization_msgs::Marker lineList;
    lineList.header.stamp = ros::Time::now();
    lineList.header.frame_id = "/map";
    lineList.action = visualization_msgs::Marker::ADD;
    lineList.type = visualization_msgs::Marker::LINE_LIST;
    lineList.scale.x = 0.05;
    lineList.color.b = 1.0;
    lineList.color.a = 1.0;
    
    geometry_msgs::Point startPnt;
    startPnt.x = localPose.x;
    startPnt.y = localPose.y;
    startPnt.z = 0;
    lineList.points.push_back(startPnt);

    geometry_msgs::Point endPnt;
    endPnt.x = localGoalPose.x;
    endPnt.y = localGoalPose.y;
    endPnt.z = 0;
    lineList.points.push_back(endPnt);
    
    marker_pub.publish(lineList);
    
    PublishCostmap(5,localPose);
    
}

void Mover::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    float maxRange = scan->range_max;
    laser_MAXRANGE = maxRange;
    float minRange = scan->range_min;
    
    float expectedHits = (scan->angle_max - scan->angle_min)/scan->angle_increment;
    for (int i = 0; i < expectedHits; i++)
    {
        SensorInfo si;
        si.distance = scan->ranges[i];
        si.valid = CheckValidRange(si.distance,minRange,maxRange);
        float correctionDist = si.distance;
        if(!si.valid){
            correctionDist = maxRange;
        }
        float angle = scan->angle_min + i * scan->angle_increment;
        float localX = (sin(angle) * correctionDist);
        float localY = (cos(angle) * correctionDist)+LASER_X_OFFSET;

        si.angle = atan2(localX,localY);
        si.distance = localX / sin(si.angle);

        laserReadings[i] = si;
    }

}

bool Mover::CheckValidRange(float rng, float min, float max){
    // minimum range as reported by sensor
    if(rng < min){
        return false;
    }
    
    if(rng > max){
        return false;   // almost max range as reported by sensor, to ignore any out of range values
    }
    // according to IEEE and the internet, this is legit. nan floats return true to this check... WTF?
    if(rng != rng){
        return false;
    }

    return true;
}

void Mover::InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data)
{

    localPose.x = data->pose.pose.position.x;
    localPose.y = data->pose.pose.position.y;
    localPose.ang = YawFromQuaternion(data->pose.pose.orientation);
    printf("Received pose: (%.1f,%.1f,%.1f)\n", localPose.x, localPose.y, localPose.ang);
}

float Mover::ComputeCertainty(void){

    float total = 0;
    for (std::list<float>::iterator it=certainty_history.begin(); it != certainty_history.end(); ++it){
        total += *it;
    }
    return total / certainty_history.size();
}

void Mover::CurrentPoseCallback(const ex03::ParticleMsg::ConstPtr& data)
{
    localPose.x = data->x;
    localPose.y = data->y;
    localPose.ang = data->angle;
    certainty_history.push_back(data->weight);
    if(certainty_history.size() > HISTORY_LENGTH){
        certainty_history.pop_front();
    }
    
    pose_certainty = ComputeCertainty();
    //printf("Received pose: (%.1f,%.1f,%.1f, %.2f%%)\n", localPose.x, localPose.y, localPose.ang, pose_certainty);
}

void Mover::NavGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& data)
{

    localGoalPose.x = data->pose.position.x;
    localGoalPose.y = data->pose.position.y;
    localGoalPose.ang = YawFromQuaternion(data->pose.orientation);
    printf("Received nav goal: (%.1f,%.1f,%.1f)\n", localGoalPose.x, localGoalPose.y, localGoalPose.ang);
    hasGoal = true;
    
}

void Mover::NewPathCallback(const nav_msgs::Path::ConstPtr& data){
    Pose oldEnd = localPose;
    if(path.size() > 0){
        oldEnd = path.back();
    }
    path.clear();
    int size = data->poses.size();
    
    geometry_msgs::PoseStamped gPose;
    Pose pPose;
    for(int i=0;i<size;i++){
        gPose = data->poses[i];
        pPose.x = gPose.pose.position.x;
        pPose.y = gPose.pose.position.y;
        pPose.ang = YawFromQuaternion(gPose.pose.orientation);
        path.push_back(pPose);
    }
    
    // kick back into global pathing if we've got a new goal    
    if(SqrDist(path.back(), oldEnd) > 0.1){
        followMode = GLOBAL_PATH;
        finalGoalPose = path.back();
    }

}
