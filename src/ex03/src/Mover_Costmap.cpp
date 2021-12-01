void Mover::InitCostmap(void){
    ros::ServiceClient client = node.serviceClient<nav_msgs::GetMap>("static_map");
    
    if(!client.exists()){
    	printf("No Map Server... Service offline.\n");
        return;
    }
    
    nav_msgs::GetMap srv;    

    if (client.call(srv))
    {

        occupancyGrid = srv.response.map;
        printf("Received Map\n");
        hasMap = true;
    }
    else
    {
        ROS_ERROR("Failed to call service");
    }
    
    if(!hasMap){
        return;
    }
    
    printf("Creating Costmap...");
    costmap_width = occupancyGrid.info.width;
    costmap_height = occupancyGrid.info.height;
    costmap_resolution = occupancyGrid.info.resolution;
    
    //obstacleBuckets = new ObstacleBucket[buckets_XWidth * buckets_YWidth];
    
    costmap_size = costmap_width*costmap_height;
    
    costmap = new MapCost[costmap_size];
    
    for(int i=0;i<costmap_size;i++){
        MapCost cost;
        
        cost.x = i % costmap_width;
        cost.y = (i / costmap_width);
        cost.worldX = cost.x * costmap_resolution;
        cost.worldY = cost.y * costmap_resolution;
        cost.fixedCost = occupancyGrid.data[i];
        cost.dynamicCost = 0;
        cost.inflationCost = 0;
        cost.obstructed = false;
        cost.trueObstacle = false;
        cost.inflated = false;
        cost.visited = false;
        costmap[i] = cost;
    }
    
    
    inflate_width = (4*INFLATE_RADIUS/costmap_resolution)+1;
    int midX = ((inflate_width/2));
    int midY = midX;
    
    inflate_size = inflate_width * inflate_width;
    // compute inflation matrix
    inflation = new Inflation[inflate_size];
    for(int i=0;i<inflate_size;i++){
        
        int x = i % inflate_width;
        int y = i / inflate_width;
        
        const int dx = midX-x;
        const int dy = midY-y;
        
        int costmapCentreOffset = dx + (dy*costmap_width);
        
        
        float dist = sqrt((dx*dx)+(dy*dy)) * costmap_resolution;
        
        float occupancy = (2*INFLATE_RADIUS) - dist;
        if(occupancy < 0){
            occupancy = 0;
        }
        
        int cost = (occupancy * 127);
        
        
        inflation[i] = Inflation(costmapCentreOffset,cost >= 65,cost);
        if(x == 0){
            printf("\n| ");
        }
        printf("%d | ",inflation[i].indexOffset);
        
    }
    
    
    hasCostmap = true;
}


void Mover::ClearObstacles(void){
    for(int i=0;i<cleared.size();i++){
        ClearObstacle(localPose,cleared[i]);       
    }
}

void Mover::ClearObstacle(const Pose &worldBase,float localAngle){
    
    float realAngle = ClampAngle(worldBase.ang + localAngle);
    MarkObstacleRay(worldBase,realAngle,laser_MAXRANGE/2,false,false);
    
}

void Mover::MarkObstacles(void){
    for(int i=0;i<obstacles.size();i++){
        MarkObstacle(localPose,obstacles[i]);
    }
}

void Mover::MarkObstacle(const Pose &worldBase, const Obstacle &localObstacle){

    float realAngle = ClampAngle(worldBase.ang + localObstacle.angle);
    MarkObstacleRay(worldBase,realAngle,localObstacle.distance,true,false);
}

#define MAP_VALID(i, j) ((i >= 0) && (i < map_width) && (j >= 0) && (j < map_height))
#define MAP_INDEX(i, j) ((i) + (j) * map_width)
#define GET_VAL(map_seq, i, j) (map_seq[MAP_INDEX(i,j)])

void Mover::MarkObstacleRay(const Pose &origin, float angle, float distance, bool finalCellObstructed, bool clearInflation) {

    int map_width = costmap_width;
    int map_height = costmap_height;
    double map_resolution = costmap_resolution;
    double max_range = distance/map_resolution;

    //printf("mw %d mh %d mox %f moy %f mr %f maxr %f\n", map_width, map_height, map_origin_x, map_origin_y, map_resolution, max_range);

    // Bresenham raytracing
    int x0,x1,y0,y1;
    int x,y;
    int xstep, ystep;
    char steep;
    int tmp;
    int deltax, deltay, error, deltaerr;

    x0 = origin.x/map_resolution; //(floor((origin.x - map_origin_x) / map_resolution + 0.5) + map_width / 2);
    y0 = origin.y/map_resolution; //(floor((origin.y - map_origin_y) / map_resolution + 0.5) + map_height / 2);

    x1 = x0 + cos(angle) * max_range; //(floor(((origin.x + max_range * cos(origin.ang)) - map_origin_x) / map_resolution + 0.5) + map_width / 2);
    y1 = y0 + sin(angle) * max_range; //(floor(((origin.y + max_range * sin(origin.ang)) - map_origin_y) / map_resolution + 0.5) + map_height / 2);

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
            if(MAP_VALID(y,x)){
                int index = MAP_INDEX(y,x);
                costmap[index].obstructed = false;
                if(clearInflation){
                    costmap[index].inflated = false;
                }
            }
        }
        else
        {
            if(MAP_VALID(x,y)){
                int index = MAP_INDEX(x,y);
                costmap[index].obstructed = false;
                if(clearInflation){
                    costmap[index].inflated = false;
                }
            }
        }
    }
    
    if (steep){
        if(MAP_VALID(y,x)){
            int index = MAP_INDEX(y,x);
            costmap[index].obstructed = finalCellObstructed;
            if(finalCellObstructed){
                costmap[index].trueObstacle = true;
            }
            if(clearInflation){
                costmap[index].inflated = false;
            }
        }
    }
    else
    {
        if(MAP_VALID(x,y)){
            int index = MAP_INDEX(x,y);
            costmap[index].obstructed = finalCellObstructed;
            if(finalCellObstructed){
                costmap[index].trueObstacle = true;
            }
            if(clearInflation){
                costmap[index].inflated = false;
            }
        }
    }
}

bool Mover::RaycastInflated(const Pose &origin, float angle, float distance) {

    int map_width = costmap_width;
    int map_height = costmap_height;
    double map_resolution = costmap_resolution;
    double max_range = distance/map_resolution;

    //printf("mw %d mh %d mox %f moy %f mr %f maxr %f\n", map_width, map_height, map_origin_x, map_origin_y, map_resolution, max_range);

    // Bresenham raytracing
    int x0,x1,y0,y1;
    int x,y;
    int xstep, ystep;
    char steep;
    int tmp;
    int deltax, deltay, error, deltaerr;

    x0 = origin.x/map_resolution; //(floor((origin.x - map_origin_x) / map_resolution + 0.5) + map_width / 2);
    y0 = origin.y/map_resolution; //(floor((origin.y - map_origin_y) / map_resolution + 0.5) + map_height / 2);

    x1 = x0 + cos(angle) * max_range; //(floor(((origin.x + max_range * cos(origin.ang)) - map_origin_x) / map_resolution + 0.5) + map_width / 2);
    y1 = y0 + sin(angle) * max_range; //(floor(((origin.y + max_range * sin(origin.ang)) - map_origin_y) / map_resolution + 0.5) + map_height / 2);

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
            if(MAP_VALID(y,x)){
                int index = MAP_INDEX(y,x);
                if(costmap[index].inflated){
                    return true;
                }
            }
        }
        else
        {
            if(MAP_VALID(x,y)){
                int index = MAP_INDEX(x,y);
                if(costmap[index].inflated){
                    return true;
                }
            }
        }
    }
    
    if (steep){
        if(MAP_VALID(y,x)){
            int index = MAP_INDEX(y,x);
            if(costmap[index].inflated){
                    return true;
                }
        }
    }
    else
    {
        if(MAP_VALID(x,y)){
            int index = MAP_INDEX(x,y);
            if(costmap[index].inflated){
                    return true;
            }
        }
    }
}

/*
void Mover::AddObstacle(const Pose &worldPose, const Obstacle &localObstacle){

    // compute world obstacle pose
    Pose worldPose = WorldPoseFromLocalDeltas(worldPose,localObstacle.angle,localObstacle.distance);
    
    // round to nearest multiple of cell size
    int xPos = (int) (std::floor(worldPose.x / bucketCellSize) * bucketCellSize);
    int yPos = (int) (std::floor(worldPose.y / bucketCellSize) * bucketCellSize);
    
    int cellCoord = xPos + yPos * height;
    
}*/

Pose Mover::WorldPoseFromLocalDeltas(Pose worldBase, float localAngle, float localDistance){
    
    // default to base pose
    Pose result = worldBase;
    
    // work out global offsets
    float worldAng = ClampAngle(worldBase.ang + localAngle);
    float world_dx = cos(worldAng)*localDistance;
    float world_dy = sin(worldAng)*localDistance;
    
    // calculate new world pose
    result.x = worldBase.x + world_dx;
    result.y = worldBase.y + world_dy;
    result.ang = worldAng;
    
    return result;
}

void Mover::HardResetCostmap(void){
   for(int i=0;i<costmap_size;i++){
        costmap[i].inflated = false;
        costmap[i].obstructed = false;
        costmap[i].inflationCost = 0;
    }
    MarkObstacles();
    InflateAllMap();
}

void Mover::RecalcCostmap(void){
   for(int i=0;i<costmap_size;i++){
        costmap[i].inflated = false;
        costmap[i].inflationCost = 0;
    }
    InflateAllMap();
}
void Mover::InflateAllMap(void){
    for(int i=0;i<costmap_size;i++){
        
        MapCost cost = costmap[i];
        if(cost.fixedCost > 0){
            AddInflationAroundObstacle(i);
        }
        else if(cost.obstructed){
            AddInflationAroundObstacle(i);
        }
        else if(cost.trueObstacle){
            // mark each true obstacle once
            AddInflationAroundObstacle(i);
            costmap[i].obstructed = true;
            costmap[i].trueObstacle = false;
        }
    }
}

void Mover::AddInflationAroundObstacle(int obstacleIndex){

    for(int i=0;i<inflate_size;i++){
    
        // compute array index
        int index = inflation[i].indexOffset + obstacleIndex;
        
        // check valid
        if(index < 0 || index > costmap_size){
            continue;
        }
        
        if(inflation[i].costFromCenter > costmap[index].inflationCost){
            costmap[index].inflationCost = inflation[i].costFromCenter;
        }
        
        if(inflation[i].inflate){
            costmap[index].inflated = true;
        }
    }   
}

void Mover::FindInflationAroundEmpty(int emptyIndex){
    
    for(int i=0;i<inflate_size;i++){
    
        // compute array index
        int index = inflation[i].indexOffset + emptyIndex;
        
        // check valid
        if(index < 0 || index > costmap_size){
            continue;
        }
        
        if(costmap[index].obstructed){
            costmap[emptyIndex].inflated = true;
            return;
        }
    }
       
}

void Mover::PublishCostmap(float radius, Pose pose){


    std::vector<Pose> path = DijkstrasCostmap(localPose);
    
    if(path.size() > 0){
    
    visualization_msgs::Marker lineList;
    lineList.header.stamp = ros::Time::now();
    lineList.header.frame_id = "/map";
    lineList.action = visualization_msgs::Marker::ADD;
    lineList.type = visualization_msgs::Marker::LINE_LIST;
    lineList.scale.x = 0.05;
    lineList.color.b = 1.0;
    lineList.color.g = 1.0;
    lineList.color.a = 1.0;
    
    geometry_msgs::Point prevPnt;
    prevPnt.x = path[0].x;
    prevPnt.y = path[0].y;
    prevPnt.z = 0;


    for(int i = 1; i < path.size(); i++) {
        
        geometry_msgs::Point pnt;
        pnt.x = path[i].x;
        pnt.y = path[i].y;
        pnt.z = 0;
        lineList.points.push_back(prevPnt);
        lineList.points.push_back(pnt);
        prevPnt = pnt;
    }
    
    localPath_pub.publish(lineList);

    }

    nav_msgs::GridCells costmapCells;
    
    costmapCells.header.stamp = ros::Time::now();
    costmapCells.header.frame_id = "/map";
    costmapCells.cell_width = 0.05;
    costmapCells.cell_height = 0.05;
    
    const float sqrRadius = radius * radius;
    
    int cnt = 0;
    
    nav_msgs::GridCells obsMsg;
    
    obsMsg.header.stamp = ros::Time::now();
    obsMsg.header.frame_id = "/map";
    obsMsg.cell_width = 0.05f;
    obsMsg.cell_height = 0.05f;
    
    for(int i=0;i<costmap_size;i++){
        
        MapCost cost = costmap[i];
        
        if(!cost.inflated){
            continue;
        }

        float dx = (cost.worldX - pose.x);
        float dy = (cost.worldY - pose.y);
     
        float dist = (dx*dx)+(dy*dy);
        if(dist < sqrRadius){
            cnt++;
            
            geometry_msgs::Point point = geometry_msgs::Point();
            point.x = cost.worldX;
            point.y = cost.worldY;
            point.z = 0;
            costmapCells.cells.push_back(point);
            
            if(cost.obstructed){
                obsMsg.cells.push_back(point);
            }
        }
        
        
    }
    //printf("publishing...%d\n",costmapCells.cells.size());
    costmap_pub.publish(costmapCells);
    obstacles_pub.publish(obsMsg);
}

int Mover::FindGoalCell(int startIndex, int fallback){

    for(int i=startIndex;i<path.size();i++){
        int goalX = path[i].x / costmap_resolution;
        int goalY = path[i].y / costmap_resolution;
        int goalCell = goalX + (goalY * costmap_width);
        
        MapCost cost = costmap[goalCell];
        
        if(!cost.inflated){
            return goalCell;
        }
    }
    return fallback;
}

#define COSTMAP_VALID(i) ((i) >= 0 && (i) < costmap_size)

#define DIJKSTRAS_COST 5

// for the queue!
class costmapComparison
{
    const MapCost *map;
public:
    costmapComparison (const MapCost *cmap)
    {
        map = cmap;
    }
    
    bool operator() (const int& lhs, const int&rhs) const
    {
        return map[lhs].dijkstrasCost > map[rhs].dijkstrasCost;
    }
};


std::vector<Pose> Mover::DijkstrasCostmap(Pose origin){
    std::vector<Pose> positions;
    
    
    // holy hell dat typedef :O
    std::priority_queue<int,std::vector<int>,costmapComparison> queue (costmap);
    
    
    if(path.size() == 0){
        printf("NO GLOBAL PATH\n");
        return positions;
    }

    // reset the costmap
    for(int i=0;i<costmap_size;i++){
        costmap[i].parent = -1;
        costmap[i].visited = false;
        costmap[i].dijkstrasCost = FLT_MAX;
    }
    
    int startX = origin.x / costmap_resolution;
    int startY = origin.y / costmap_resolution;

    int startCell = startX + (startY*costmap_width);

    int goalCell= FindGoalCell(localPathEnd,startCell);
    
    if(!COSTMAP_VALID(startCell) || !COSTMAP_VALID(goalCell)){
        printf("INVALID START / GOAL CELL\n");
        return positions;
    }
    
    if(startCell == goalCell){
        printf("NO VALID GOAL CELL!\n");
        return positions;
    }
    
    costmap[startCell].dijkstrasCost = 0;
    queue.push(startCell);
    
    while(queue.size() > 0){
        
        // get next lowest item
        int cellIndex = queue.top();
        queue.pop();
        MapCost cell = costmap[cellIndex];

        // skip if we've already been here
        if(cell.visited){
            continue;
        }
        
        // mark this cell as visited
        costmap[cellIndex].visited = true;
        float distToCell = cell.dijkstrasCost;

        if(cell.GetColor() > WALKABLE_COL){
            continue;
        }
        
        // check to stop
        if(cellIndex == goalCell){
            //printf("Found Costmap Goal!\n");
            break;    
        }
            
        // get neighbours
        int left = cellIndex-1;
        int right = cellIndex+1;
        int up = cellIndex + costmap_width;
        int down = cellIndex - costmap_width;
        
        if(COSTMAP_VALID(left) && !costmap[left].visited){
            float cost = DIJKSTRAS_COST + costmap[left].inflationCost + distToCell;
            if(cost < costmap[left].dijkstrasCost){
                costmap[left].parent = cellIndex;
                costmap[left].dijkstrasCost = cost;
                queue.push(left);
            }
        }
        
        if(COSTMAP_VALID(right) && !costmap[right].visited){
            float cost = DIJKSTRAS_COST + costmap[right].inflationCost + distToCell;
            if(cost < costmap[right].dijkstrasCost){
                costmap[right].parent = cellIndex;
                costmap[right].dijkstrasCost = cost;
                queue.push(right);
            }
        }
                    
        if(COSTMAP_VALID(down) && !costmap[down].visited){
        float cost = DIJKSTRAS_COST + costmap[down].inflationCost + distToCell;
            if(cost < costmap[down].dijkstrasCost){
                costmap[down].parent = cellIndex;
                costmap[down].dijkstrasCost = cost;
                queue.push(down);
            }
        }

        
        if(COSTMAP_VALID(up) && !costmap[up].visited){
            float cost = DIJKSTRAS_COST + costmap[up].inflationCost + distToCell;
            if(cost < costmap[up].dijkstrasCost){
                costmap[up].parent = cellIndex;
                costmap[up].dijkstrasCost = cost;
                queue.push(up);
            }
        }
    
    }

    printf("Finished!\n");
    MapCost cell = costmap[goalCell];
    
    while(cell.parent != -1){
        int p = cell.parent;
        Pose pos = Pose(cell.worldX,cell.worldY,0);
        positions.push_back(pos);
        cell = costmap[p];
    }
    
    printf("positions=%lu\n",positions.size());
    
    return positions;
 
}

std::vector<Pose> Mover::FillCostmap(Pose origin){

    std::vector<Pose> positions;

    if(path.size() == 0){
        printf("NO GLOBAL PATH\n");
        return positions;
    }

    // reset the costmap
    nextCells.clear();
    for(int i=0;i<costmap_size;i++){
        costmap[i].parent = -1;
        costmap[i].visited = false;
    }
    
    int startX = origin.x / costmap_resolution;
    int startY = origin.y / costmap_resolution;

    int startCell = startX + (startY*costmap_width);

    int goalCell= FindGoalCell(localPathEnd,startCell);
    
    if(!COSTMAP_VALID(startCell) || !COSTMAP_VALID(goalCell)){
        printf("INVALID START / GOAL CELL\n");
        return positions;
    }
    
    if(startCell == goalCell){
        printf("OH GOD, WE CAN'T FIND A WAY, PANIIIIIIIC\n");
        return positions;
    }
    
    costmap[startCell].visited = true;
    nextCells.push_back(startCell);
    
    while(nextCells.size() > 0){
        openCells.clear();
        std::swap(openCells,nextCells);
        // move all neighbours
        
        for(int i=0;i<openCells.size();i++){
            int cellIndex = openCells[i];
            MapCost cell = costmap[cellIndex];

            if(cell.GetColor() > WALKABLE_COL){
                continue;
            }
            
            // check to stop
            if(cellIndex == goalCell){
                //printf("Found Costmap Goal!\n");
                break;
                
            }
            
            // get neighbours
            int left = cellIndex-1;
            int right = cellIndex+1;
            int up = cellIndex + costmap_width;
            int down = cellIndex - costmap_width;
            
            if(COSTMAP_VALID(left) && !costmap[left].visited){
                costmap[left].parent = cellIndex;
                costmap[left].visited = true;
                nextCells.push_back(left);
            }
            
            if(COSTMAP_VALID(right) && !costmap[right].visited){
                costmap[right].parent = cellIndex;
                costmap[right].visited = true;
                nextCells.push_back(right);
            }
            
            if(COSTMAP_VALID(down) && !costmap[down].visited){
                costmap[down].parent = cellIndex;
                costmap[down].visited = true;
                nextCells.push_back(down);
            }
            
            if(COSTMAP_VALID(up) && !costmap[up].visited){
                costmap[up].parent = cellIndex;
                costmap[up].visited = true;
                nextCells.push_back(up);
            }
        
        }
        
    }
    

    //printf("Finished!\n");
    MapCost cell = costmap[goalCell];
    
    while(cell.parent != -1){
        int p = cell.parent;
        Pose pos = Pose(cell.worldX,cell.worldY,0);
        positions.push_back(pos);
        cell = costmap[p];
    }
    
    //printf("positions=%lu\n",positions.size());
    
    return positions;
    
}

Pose Mover::FindNearestOpenCell(Pose origin){

    // reset the costmap
    nextCells.clear();
    for(int i=0;i<costmap_size;i++){
        costmap[i].parent = -1;
        costmap[i].visited = false;
    }
    
    int startX = origin.x / costmap_resolution;
    int startY = origin.y / costmap_resolution;

    int startCell = startX + (startY*costmap_width);
    
    if(!COSTMAP_VALID(startCell)){
        printf("INVALID START CELL\n");
        return origin;
    }
    
    costmap[startCell].visited = true;
    nextCells.push_back(startCell);
    
    while(nextCells.size() > 0){
        openCells.clear();
        std::swap(openCells,nextCells);
        // move all neighbours
        
        for(int i=0;i<openCells.size();i++){
            int cellIndex = openCells[i];
            MapCost cell = costmap[cellIndex];

            // never consider obstructed cells
            if(cell.obstructed){
                continue;
            }

            if(!cell.inflated){
            
                Pose p = Pose(cell.worldX,cell.worldY,0);
                
                float realAngle = CalculateTurnAngle(localPose,p);
                float dist = sqrt(SqrDist(p,localPose));
                
                Obstacle *obs = CircleOverlapObstacles(realAngle,dist,INFLATE_RADIUS*2);

                if(obs == NULL){
                    return p;
                }
            }
            
            
            
            // get neighbours
            int left = cellIndex-1;
            int right = cellIndex+1;
            int up = cellIndex + costmap_width;
            int down = cellIndex - costmap_width;
            
            if(COSTMAP_VALID(left) && !costmap[left].visited){
                costmap[left].parent = cellIndex;
                costmap[left].visited = true;
                nextCells.push_back(left);
            }
            
            if(COSTMAP_VALID(right) && !costmap[right].visited){
                costmap[right].parent = cellIndex;
                costmap[right].visited = true;
                nextCells.push_back(right);
            }
            
            if(COSTMAP_VALID(down) && !costmap[down].visited){
                costmap[down].parent = cellIndex;
                costmap[down].visited = true;
                nextCells.push_back(down);
            }
            
            if(COSTMAP_VALID(up) && !costmap[up].visited){
                costmap[up].parent = cellIndex;
                costmap[up].visited = true;
                nextCells.push_back(up);
            }
        
        }
        
    }
    
    return origin;
    
}
