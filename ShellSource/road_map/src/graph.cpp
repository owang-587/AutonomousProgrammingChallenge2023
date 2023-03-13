#include "graph.h"
#include <math.h>
#include <queue>
#include <algorithm>

// comparison override to use the priority queue
class Compare {
public:
  bool operator()(Path a, Path b){
    if(a.distance < b.distance){
        return true;
    }
    return false;
  }
};

Path::Path(double distance, std::vector<std::string> verticesTaken, DIRECTION startDirection, DIRECTION endDirection){
  this->distance = distance;
  this->verticesTaken = verticesTaken;
  this->startDirection = startDirection;
  this->endDirection = endDirection;
}

void Graph::createVertex(std::string id, bool isGoalPoint = false) {
  vertices[id] = Vertex(isGoalPoint);
  if (isGoalPoint){
    numOfGoalPoints++;
  }
}

void Graph::addEdge(std::string startId, std::string endId, DIRECTION startDirection, double distance) {
  vertices[startId].addEdge(endId, startDirection, distance);
}


bool Graph::isVertex(std::string id) const {
  return vertices.find(id) != vertices.end();
}

bool Graph::isEdge(std::string startId, std::string endId, DIRECTION startDirection) {
  return vertices[startId].isEdge(endId);
}

int Graph::numNeighbours(std::string id)  {
  return vertices[id].getEdges().size();
}

int Graph::size() const {
  return vertices.size();
}

std::vector<std::string> Graph::getVertices() {

  std::vector<std::string> nodes;

  for (auto it = vertices.begin();
    it != vertices.end(); it++) {
      nodes.push_back(it->first);
    }

  return nodes;
}

void Graph::createSearchTree(){
  for (auto it = vertices.begin(); it != vertices.end(); it++){
    // only make a searchTree for the vertices that are goalpoints
    if (it->second.isGoalPoint){

      // initialize the search tree (not sure if you have to do this)
      searchTree[it->first];
      updateSearchTree(it->first);
    }
  }
}

void Graph::updateSearchTree(std::string startId){

  std::unordered_map<std::string, std::vector<Path> >& goalPointSearchTree = searchTree[startId]; 
  std::priority_queue<Path, std::vector<Path>, Compare> pathQueue;
  int pathsFound = 0;

  
  auto exploreEdge = [this, &goalPointSearchTree, &pathQueue, &pathsFound](Path currentPath, std::string neighbourId, Edge& edge){
    
    // check if we hit a goal point
    if (vertices[neighbourId].isGoalPoint){
      
      // make sure we didn't already find a path to this goal point by checking if the starting direction is the same
      bool repeat = false;
      for (const auto& foundPath : goalPointSearchTree[neighbourId]){
        if (currentPath.startDirection == foundPath.startDirection){
          repeat = true;
          break;
        }
      } 
      if (repeat){
        return;
      }

      // if its a new goal point path, add it to the search tree
      goalPointSearchTree[neighbourId].push_back(currentPath);
      pathsFound++;
    }


    const double NEXT_DISTANCE = edge.distance + currentPath.distance;

    Path nextPath(NEXT_DISTANCE, currentPath.verticesTaken, currentPath.startDirection, edge.startDirection);

    // add the next node we are going to to the vertex backtrace
    nextPath.verticesTaken.push_back(neighbourId);

    pathQueue.push(nextPath);
  };

  // initialize the queue with the edge data of the starting vertex
  for (const auto& connection : vertices[startId].getEdges()){
    const std::string NEIGHBOUR_ID = connection.first;
    Edge EDGE = connection.second;

    Path newPath(EDGE.distance, {startId, NEIGHBOUR_ID}, EDGE.startDirection, EDGE.startDirection);
    pathQueue.push(newPath);
  }

  // the number of directions we are exploring from is equal to the number of edges the vertex connects to
  int startingDirections = numNeighbours(startId);

  // we want a path for each goal point starting from every starting direction 
  while (pathsFound < (numOfGoalPoints-1)*startingDirections){
    // take the first value from the priority queue and remove it
    Path currentPath = pathQueue.top();
    pathQueue.pop();

    const std::string CURRENT_VERTEX = currentPath.verticesTaken.back();
    const std::string PREVIOUS_VERTEX = currentPath.verticesTaken.rbegin()[1];

    // loop through all connections from the vertex
    for (const auto& connection : vertices[CURRENT_VERTEX].getEdges()){
      const std::string NEIGHBOUR_ID = connection.first;
      Edge EDGE = connection.second;

      // stops us from going backwards
      if (NEIGHBOUR_ID != PREVIOUS_VERTEX){
        
        exploreEdge(currentPath, NEIGHBOUR_ID, EDGE);

        // this is the original without the lambda function in case the lambda function breaks during testing
        // 
        // // check if we hit a goal point
        // if (vertices[NEIGHBOUR_ID].isGoalPoint){
          
        //   // make sure we didn't already find a path to this goal point by checking if the starting direction is the same
        //   bool repeat = false;
        //   for (const auto& foundPath : goalPointSearchTree[NEIGHBOUR_ID]){
        //     if (currentPath.startDirection == foundPath.startDirection){
        //       repeat = true;
        //       break;
        //     }
        //   } 
        //   if (repeat){
        //     break;
        //   }

        //   // if its a new goal point path, add it to the search tree
        //   goalPointSearchTree[NEIGHBOUR_ID].push_back(currentPath);
        //   pathsFound++;
        // }


        // const double NEXT_DISTANCE = EDGE.distance + currentPath.distance;

        // Path nextPath(NEXT_DISTANCE, currentPath.verticesTaken, currentPath.startDirection, EDGE.startDirection);
        // // add the next node we are going to the vertex backtrace
        // nextPath.verticesTaken.push_back(NEIGHBOUR_ID);

        // pathQueue.push(nextPath);
      }
    }
  }


  
  
}