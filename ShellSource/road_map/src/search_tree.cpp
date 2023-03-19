#include <iostream>
#include <sstream>
#include <queue>
#include <map>
#include <algorithm>

#include "../include/graph.h"

using namespace std;

/***************Direction to and from string conversion**************/

string to_string(const DIRECTION& direction) {
  switch(direction) {
      case DIRECTION::UP: return "up";
      case DIRECTION::RIGHT: return "right";
      case DIRECTION::DOWN: return "down";
      case DIRECTION::LEFT: return "left";
  }
}

DIRECTION from_string(string str) {
    
  if(str == "UP") {
    return DIRECTION::UP;

  } else if(str == "RIGHT") {
    return DIRECTION::RIGHT;

  } else if(str == "DOWN") {
    return DIRECTION::DOWN;

  } else if(str == "LEFT") {
    return DIRECTION::LEFT;
    
  }

}

/***************Reading the data to create the graph****************/

DIRECTION getOpposite(DIRECTION direction){
  if(direction == DIRECTION::DOWN) {
    return DIRECTION::UP;

  } else if(direction == DIRECTION::LEFT) {
    return DIRECTION::RIGHT;

  } else if(direction == DIRECTION::UP) {
    return DIRECTION::DOWN;

  } else if(direction == DIRECTION::RIGHT) {
    return DIRECTION::LEFT;
  }
}

Graph read_city_graph_undirected(){

  Graph graph;

  string line;

  while(getline(cin, line)) {
    char opcode = line[0];
    line.erase(0, 2);

    if (opcode == 'V'){

      const string ID = line.substr(0, line.find(','));
      line.erase(0, line.find(',')+1);

      bool isGoalPost;

      istringstream(line.substr(0, line.find(','))) >> std::boolalpha >> isGoalPost;

      line.erase(0, line.find(',')+1);
      

      graph.createVertex(ID, isGoalPost);

    } else {
      const string FIRST_VERTEX = line.substr(0, line.find(','));
      line.erase(0, line.find(',')+1);

      const string SECOND_VERTEX = line.substr(0, line.find(','));
      line.erase(0, line.find(',')+1);

      const DIRECTION startingDirection = from_string(line.substr(0, line.find(',')));
      line.erase(0, line.find(',')+1);
      
      const double distance = stod(line.substr(0, line.find(',')));
      

      DIRECTION oppositeStartDirection;

      if (line.find(',') != string::npos){
        line.erase(0, line.find(',')+1);
        oppositeStartDirection = from_string(line.substr(0, line.find(',')));
      } else {
        oppositeStartDirection = getOpposite(startingDirection);
      }

      graph.addEdge(FIRST_VERTEX, SECOND_VERTEX, startingDirection, distance);
      graph.addEdge(SECOND_VERTEX, FIRST_VERTEX, oppositeStartDirection, distance);
      
    } 
  }
  return graph;
}

/********Using dijkstra to get the shortest paths connecting each node********/

vector<Path> getDijkstraPaths(Graph& graph){
  
  vector<string> goalPoints = graph.getGoalpoints();
  vector<Path> shortestPaths;
  for (int i = 0; i < goalPoints.size(); i++){
    for (int j = i+1; j < goalPoints.size(); j++){
      vector<Path> paths = graph.getShortestPaths(goalPoints[i], goalPoints[j]);
      for (auto path : paths){
        shortestPaths.push_back(path);
      }

      paths = graph.getShortestPaths(goalPoints[j], goalPoints[i]);
      for (auto path : paths){
        shortestPaths.push_back(path);
      }
    }
  }
  return shortestPaths;
}

void printPaths(vector<Path>& shortestPaths){
  for (auto path : shortestPaths){
    if (path.verticesTaken.size() > 0){
      cout << "From " << path.verticesTaken.front() 
           << " facing "  << to_string(path.startDirection) 
           << " to " << path.verticesTaken.back() 
           << " has a distance of " << path.distance << endl;

      cout << "The path taken was: ";
      for (auto vertex : path.verticesTaken){
      cout << vertex << "->";
      }
      cout << "Done" << endl;
    }
  }
}

/****************Creating the minimum spanning tree*****************/

// comparison override to use the priority queue
class Compare {
public:
  bool operator()(Path a, Path b){
    if(a.distance > b.distance){
        return true;
    }
    return false;
  }
};

map<string, vector<Path>> makeMinSpanTree(Graph& graph, string startId){
  map<string, vector<Path>> minSpanningTree;
  priority_queue<Path, vector<Path>, Compare> pathQueue; 
  vector<string> goalPoints = graph.getGoalpoints();

  minSpanningTree[startId];
  goalPoints.erase(find(goalPoints.begin(), goalPoints.end(), startId));

  // put in the intial values into the tree
  for (auto goalPoint : goalPoints){
    minSpanningTree[goalPoint];
    vector<Path> shortestPaths = graph.getShortestPaths(startId, goalPoint);
    for (auto path : shortestPaths){
      pathQueue.push(path);
    }
  }

  // implementing Prim's algorithm
  // essentially just take the shortest connection from any node already connected
  // if its a new node do the connection

  // we only want the start to connect to the closest neighbour
  while (!pathQueue.empty()){
    Path currentPath = pathQueue.top();
    pathQueue.pop();

    const string EDGE_END = currentPath.verticesTaken.back();
    auto VERTEX_INDEX = find(goalPoints.begin(), goalPoints.end(), EDGE_END);

    if(VERTEX_INDEX != goalPoints.end()){
      const string EDGE_START = currentPath.verticesTaken.front();

      minSpanningTree[EDGE_START].push_back(currentPath);

      
      reverse(currentPath.verticesTaken.begin(), currentPath.verticesTaken.end());
      Path reversePath(currentPath);
      reversePath.startDirection = getOpposite(currentPath.endDirection);
      reversePath.endDirection = getOpposite(currentPath.startDirection);

      minSpanningTree[EDGE_END].push_back(reversePath);

      goalPoints.erase(VERTEX_INDEX);
      // put in the intial values into the tree
      for (auto goalPoint : goalPoints){
        vector<Path> shortestPaths = graph.getShortestPaths(EDGE_END, goalPoint);
        for (auto path : shortestPaths){
          pathQueue.push(path);
        }
      }
    }
  }

  return minSpanningTree;
}

void printMinSpanTree(map<string, vector<Path>>& minSpanningTree){
  cout << "The minimum spanning tree is: " << endl;
  for (auto itr = minSpanningTree.begin(); itr != minSpanningTree.end(); itr++){
    cout << "From " << itr->second.front().verticesTaken.front()
         << " to " << itr->second.front().verticesTaken.back()
         << " has a distance of " << itr->second.front().distance << endl;
  }
}



int main(){
  Graph graph = read_city_graph_undirected();
  graph.createSearchTree();



  return 0;
}