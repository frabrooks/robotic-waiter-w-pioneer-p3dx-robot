#include "Structures.h"

#include <algorithm>
#include <cstdlib> 
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <vector>

std::vector<int> extract_keys(std::map<int, int> m) {
    //map<int, int> m;
    vector<int> v;
    for(map<int,int>::iterator it = m.begin(); it != m.end(); ++it) {
      v.push_back(it->first);
    }
    return v;
}

std::map<int, float> updateMap(std::map<int, float> currentMap, int index, float value) {
    std::map<int, float>::iterator itr = currentMap.find(index); 
    if (itr != currentMap.end()) {
        itr->second = value;
    }
    return currentMap;
}

std::map<int, int> updateMap(std::map<int, int> currentMap, int index, float value) {
    std::map<int, int>::iterator itr = currentMap.find(index); 
    if (itr != currentMap.end()) {
        itr->second = value;
    }
    return currentMap;
}

/*bool vectorContains(std::vector<int> v, int x) {
    if(std::find(v.begin(), v.end(), x) != v.end()) {
        return true;
    } else {
        return false;
    }
}*/

void print(const std::vector<int> &vec)
{
  for (std::vector<int>::const_iterator i = vec.begin(); i != vec.end(); ++i)
    std::cout << *i << ' ';
  std::cout << std::endl;
}

std::map<int, float> initGScores(Graph graph, int startId) {
    std::map<int, float> gScores;
    
    map<int, Vertex*>::iterator itr;
    for(itr = graph.vertices.begin(); itr != graph.vertices.end(); itr++) {
        float gScore;
        if (itr->first == startId) {
            gScore = 0.0f;
        } else {
            gScore = std::numeric_limits<float>::max();
        }        
        gScores.insert(std::pair<int, float>(itr->first, gScore));
    }    
    
    return gScores;
}

std::map<int, float> initFScores(Graph graph, int startId, int goalId) {
    std::map<int, float> fScores;
    
    map<int, Vertex*>::iterator itr;
    for(itr = graph.vertices.begin(); itr != graph.vertices.end(); itr++) {
        float fScore;
        if (itr->first == startId) {
            Vertex start = graph.getVertex(startId);
            Vertex goal = graph.getVertex(goalId);
            fScore = start.distanceTo(goal);
        } else {
            fScore = std::numeric_limits<float>::max();
        }
        fScores.insert(std::pair<int, float>(itr->first, fScore));
    }    
    
    return fScores;
}

int findClosestOpen(std::map<int, float> fScores, std::vector<int> open) {
    //print(open);
    int closest;
    float closestDist = std::numeric_limits<float>::max();
    
    std::vector<int>::iterator itr;
    for(itr = open.begin(); itr != open.end(); itr++) {
        float fScore = fScores.find(*itr)->second;
        
        if (fScore < closestDist) {
            closest = *itr;

            closestDist = fScore;
            //printf("closest is now: %d\n", closest);
        }
    }
    
    /*map<int, float>::iterator itr;
    int count = 0;
    for(itr = fScores.begin(); itr != fScores.end(); itr++) {
        //printf("count=%d\n", count);
        //printf("id: %d, fScore:%d\n", itr->first, itr->second);
        if (itr->second < closestDist) {
            closest = itr->first;
            
        }
        count++;
    }*/
    
    return closest;
}


std::vector<int> reconstructPath(std::map<int, int> cameFrom, int currentId, Graph graph) {
    /*std::map<int,int>::iterator itr;
    for(itr = cameFrom.begin(); itr != cameFrom.end(); itr++) {
        cout << "(" << itr->first << "," << itr->second << ") ";
    }
    printf("\n");*/

    std::vector<int> totalPath;
    totalPath.push_back(currentId);    
    
    std::vector<int> vertexIds = extract_keys(cameFrom);
    
    while(vectorContains(vertexIds, currentId)) {
        currentId = cameFrom[currentId];
        //printf("cid: %d\n", currentId);
        //totalPath.push_back(currentId);
        totalPath.insert(totalPath.begin(), currentId);
    }
    /*printf("Total path: ");
    print(totalPath);*/
    return totalPath;
    //return smoothPath(totalPath, graph);
}

std::vector<int> findVertexPath(Graph graph, int startId, int goalId){
    //printf("Finding a path from node %d to node %d...\n", startId, goalId);
    std::vector<int> open;    
    open.push_back(startId);
    std::vector<int> closed;
    std::map<int, int> cameFrom;
    
    std::map<int, float> gScores = initGScores(graph, startId);
    //printf("Initialised G-scores!\n");
    std::map<int, float> fScores = initFScores(graph, startId, goalId);
    //printf("Initialised F-scores!\n");
    
    int currentId = startId;
    
    Vertex goal = graph.getVertex(goalId);
    //print(open);
    while(!open.empty()) {
    
        currentId = findClosestOpen(fScores, open);
            //printf("Considering node %d\n",currentId);
        
        if(currentId == goalId){
            printf("Path found!\n");
            return reconstructPath(cameFrom, currentId, graph);
        }
        
        //print(open);
        open.erase(std::remove(open.begin(), open.end(), currentId), open.end());
        //print(open);        
        //printf("Removed node %d\n",currentId);
        //printf("%d\n", open[3]);
        closed.push_back(currentId);
        
        Vertex current = graph.getVertex(currentId);
        //std::vector<std::pair<float, Vertex*> > neighbours = current.adj;
        map<float, Vertex*> neighbours = current.adj;
                    
        map<float, Vertex*>::iterator itr;        
        for(itr = neighbours.begin(); itr != neighbours.end(); itr++) {
            Vertex neighbour = *(itr->second);
            int neighbourId = neighbour.id;
            
            if(vectorContains(closed, neighbourId)) {
                //printf("Node %d is closed\n", neighbourId);
                continue;
            }
                                
            float tentativeGScore = gScores.find(currentId)->second + current.distanceTo(neighbour);
            if(!vectorContains(open, neighbourId)){
                open.push_back(neighbourId);
            }
            else if(tentativeGScore >= gScores.find(neighbourId)->second){
                continue;
            }
            cameFrom[neighbourId] = currentId;
            gScores[neighbourId] = tentativeGScore;
            fScores[neighbourId] = tentativeGScore + neighbour.distanceTo(goal);   
            if (neighbour.obstructed) { 
                fScores[neighbourId] += 100;
            }
        }     
    }
    printf("WE SHOULDN'T END UP HERE");
}
