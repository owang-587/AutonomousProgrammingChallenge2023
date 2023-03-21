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

map<string, vector<Path>> getDijkstraPaths(Graph& graph){
  
  map<string, vector<Path>> dijkstraPaths;
  vector<string> goalPoints = graph.getGoalpoints();
  

  for (int i = 0; i < goalPoints.size(); i++){
    vector<Path> shortestPaths;
    for (int j = 0; j < goalPoints.size(); j++){
      if (i == j){
        continue;
      }
      const vector<Path> PATHS = graph.getShortestPaths(goalPoints[i], goalPoints[j]);
      for (auto path : PATHS){
        shortestPaths.push_back(path);
      }
    }
    dijkstraPaths[goalPoints[i]] = shortestPaths;
  }

  return dijkstraPaths;
}

void printPaths(map<string, vector<Path>>& dijkstraPaths){
  for (auto itr = dijkstraPaths.begin(); itr != dijkstraPaths.end(); itr++){
    for (auto path : itr->second){
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

vector<Path> makeMinSpanTree(Graph& graph, string startId){
  vector<Path> minSpanningTree;
  priority_queue<Path, vector<Path>, Compare> pathQueue; 
  vector<string> goalPoints = graph.getGoalpoints();
  goalPoints.erase(find(goalPoints.begin(), goalPoints.end(), startId));

  // put in the intial values into the tree
  for (auto goalPoint : goalPoints){
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

      minSpanningTree.push_back(currentPath);
      
      // reverse(currentPath.verticesTaken.begin(), currentPath.verticesTaken.end());
      // Path reversePath(currentPath);
      // reversePath.startDirection = getOpposite(currentPath.endDirection);
      // reversePath.endDirection = getOpposite(currentPath.startDirection);

      // minSpanningTree[EDGE_END].push_back(reversePath);

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

void runNearestNeighbours(Graph& graph){
  const int NUM_GOALPOINTS = graph.getGoalpoints().size();

  map<string, vector<Path>> dijkstraPaths = getDijkstraPaths(graph);

  vector<string> nearestNeighbourPath = {"GP1"};

  Path nextPath;

  const string CURRENT_VERTEX = nearestNeighbourPath.back();
  const vector<Path> connections = dijkstraPaths[CURRENT_VERTEX];

  double smallestDist = __DBL_MAX__;
  double nnTotalDistance = 0;
  string closestVertex;

  // find the nearest neighbour
  for (const auto& path : connections){
    const string PATH_END_VERTEX = path.verticesTaken.back();
    const bool NOT_SEARCHED = find(nearestNeighbourPath.begin(), nearestNeighbourPath.end(), PATH_END_VERTEX) == nearestNeighbourPath.end();
    if (NOT_SEARCHED && path.distance < smallestDist){
      smallestDist = path.distance;
      closestVertex = PATH_END_VERTEX;
      nextPath = path;
    }
  }

  DIRECTION currentDirection = nextPath.endDirection;
  nnTotalDistance += nextPath.distance;
  nearestNeighbourPath.push_back(nextPath.verticesTaken.back());
  
  while (nearestNeighbourPath.size() < NUM_GOALPOINTS){
    Path nextPath;
    const string CURRENT_VERTEX = nearestNeighbourPath.back();
    const vector<Path> connections = dijkstraPaths[CURRENT_VERTEX];

    double smallestDist = __DBL_MAX__;
    string closestVertex;

    // find the nearest neighbour
    for (const auto& path : connections){
      const string PATH_END_VERTEX = path.verticesTaken.back();
      const bool NOT_SEARCHED = find(nearestNeighbourPath.begin(), nearestNeighbourPath.end(), PATH_END_VERTEX) == nearestNeighbourPath.end();
      if (NOT_SEARCHED && path.distance < smallestDist && path.startDirection == currentDirection){
        smallestDist = path.distance;
        closestVertex = PATH_END_VERTEX;
        nextPath = path;
      }
    }

    currentDirection = nextPath.endDirection;
    nnTotalDistance += nextPath.distance;
    nearestNeighbourPath.push_back(nextPath.verticesTaken.back());

  }

  for (auto path : nearestNeighbourPath){
    cout << path << ' ';
  }

  cout << endl;

  cout << "NN Distance: " << nnTotalDistance << endl;

  vector<Path> mst = makeMinSpanTree(graph, "GP1");

  double mstDistance = 0;
  for (auto path : mst){
    mstDistance += path.distance;
  }

  cout << "MST Distance: " << mstDistance << endl;

  cout << "Score: " << (nnTotalDistance/mstDistance) << endl;  
}



int main(){
  Graph graph = read_city_graph_undirected();
  graph.createSearchTree();
  

  runNearestNeighbours(graph);
  
  return 0;
}