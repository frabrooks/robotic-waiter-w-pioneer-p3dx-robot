void Mover::LocateObstacles(void){

    const int WINDOW_SIZE = 3;
    
    int closeScans = 0;
    float totalScans = 1;

    cleared.clear();
    obstacles.clear();

    
    for(int i=laserStartIndex;i<laserEndIndex;i++){
    
    
        float distance = 0;
        float hitAngle = 0;
        float missAngle = 0;
        int hitCnt = 0;
        
        Obstacle obstacle;
        
        
        SensorInfo info = laserReadings[i];

        if(!info.valid){
            cleared.push_back(info.angle);
        }
        else{
            obstacle.distance = info.distance;
            obstacle.angle = info.angle;
            obstacle.localX = cos(obstacle.angle)*obstacle.distance;
            obstacle.localY = sin(obstacle.angle)*obstacle.distance;
            
            float worldAngle = ClampAngle(localPose.ang +obstacle.angle); 
            obstacle.worldX = localPose.x + (cos(worldAngle)*obstacle.distance);
            obstacle.worldY = localPose.y + (sin(worldAngle)*obstacle.distance);
    
            if(obstacle.distance < OBSTACLE_SLOW_DIST){
                closeScans++;
            }
        
            if(obstacle.distance < OBSTACLE_DETECT_DIST){
                obstacles.push_back(obstacle);
            }
            
            totalScans++;
        }
        
        
    }
    
    // compute speed scalars based on obstacle close-ness
    obstacleSlowdown = 1-(closeScans/totalScans);

}

bool Mover::FrontObstructed(float dangerDist, float robotHalfWidth){
    for(int i=0;i<obstacles.size();i++){
        float d = fabsf(obstacles[i].localY);
        // skip obstacles that are behind
        if(obstacles[i].localX <= 0){
            continue;
        }
        // skip obstacles that are too far away
        if(obstacles[i].localX > dangerDist){
            continue;
        }
        
        if(d<robotHalfWidth){
            return true;
        }
    }
    return false;
}

bool Mover::CheckConeObstructed(float distToTarget, float angleToTarget, float angleTolerance){
    for(int i=0;i<obstacles.size();i++){
        float angleDelta = ClampAngle(obstacles[i].angle - angleToTarget);
        angleDelta = fabsf(angleDelta);
        // skip if the obstacle isn't in the right angle
        if(angleDelta - angleTolerance > angleTolerance){
            continue;
        }
        
        if(obstacles[i].distance < distToTarget){
            return true;
        }
        
    }
    return false;
}

Obstacle *Mover::CircleOverlapObstacles(float localAngle, float localDistance, float radius){

    const float targetX = localDistance * cos(localAngle);
    const float targetY = localDistance * sin(localAngle);
    const float sqrRadius = radius * radius;

    for(int i=0;i<obstacles.size();i++){
        Obstacle obs = obstacles[i];
        float dx = targetX - obs.localX;
        float dy = targetY - obs.localY;
        float sqrDist = (dx*dx)+(dy*dy);
        if(sqrDist < sqrRadius){
            return &(obstacles[i]);
        }
    }
    return NULL;
}

// returns NULL if no obstacle is in that location, or a pointer to the closest obstacle if an obstacle blocks it
Obstacle *Mover::CircleCastAgainstObstacles(float localAngle, float localDistance, float radius){
    
    //printf("casting %.4f,%.4f\n",localAngle,localDistance);
    
    float targetX = localDistance * cos(localAngle);
    float targetY = localDistance * sin(localAngle);

    float closeDist = radius * radius;

    Obstacle *closest = NULL;

    // do a distance to line segment calculation
    
    // a = origin
    // b = target
    // p = obstacle
    // a2b = a to b
    Vector2 a = Vector2(0,0);
    Vector2 b = Vector2(targetX,targetY);

    Vector2 a2b = b - a;
    const float a2bSqrMag = a2b.SqrMagnitude();

    for(int i=0;i<obstacles.size();i++){
        
        // compute P
        Vector2 p = Vector2(obstacles[i].localX,obstacles[i].localY);
        Vector2 a2p = p - a;
        
        // compute how far Q is along AB
        float dot = Vector2::Dot(a2b,a2p);
        float percentDist = dot / a2bSqrMag;
        
        // find Q along our line
        Vector2 q = a2b * percentDist;
        if(percentDist < 0){
            // skip if behind us
            q = a;
        }
        else if(percentDist > 1){
            q = b;
        }
        
        // compute distance Q to P
        Vector2 p2q = p - q;
        float sqrDistToObst = p2q.SqrMagnitude();
        

        
        // check distance against radius
        if(sqrDistToObst < closeDist){
           //printf("p=%.3f,%.3f || q =%.3f,%.3f || d^=%.3f\n",p.x,p.y,q.x,q.y,sqrDistToObst);
           //printf("OMG OBSTACLE %.2f,%.2f\n",obstacles[i].localX,obstacles[i].localY);
           closeDist = sqrDistToObst;
           closest = &obstacles[i];
        }    
    }
    
    return closest;
}

Obstacle Mover::ClosestObstacleWithinAngle(float angleMin, float angleMax){
    float closestDist = 99999;
    Obstacle closest = Obstacle();
    for(int i=0;i<obstacles.size();i++){
        if(obstacles[i].angle < angleMin){
            continue;
        }
        if(obstacles[i].angle > angleMax){
            continue;
        }
        if(obstacles[i].distance < closestDist){
            closestDist = obstacles[i].distance;
            closest = obstacles[i];
        }
    }
    return closest;
}
