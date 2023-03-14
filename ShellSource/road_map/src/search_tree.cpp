#include <iostream>
#include <sstream>

#include "../include/graph.h"

using namespace std;

string to_string(const DIRECTION& s) {
  switch(s) {
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
      line.erase(0, line.find(',')+1);

      graph.addEdge(FIRST_VERTEX, SECOND_VERTEX, startingDirection, distance);
      graph.addEdge(SECOND_VERTEX, FIRST_VERTEX, getOpposite(startingDirection), distance);
      
    }
    
  }

  return graph;
}

int main(){
  Graph graph = read_city_graph_undirected();

  graph.createSearchTree();

  vector<string> goalPoints = graph.getGoalpoints();

  vector<Path> shortestPaths;

  shortestPaths.push_back(graph.getShortestPath(goalPoints[0], goalPoints[1], DIRECTION::LEFT));
  shortestPaths.push_back(graph.getShortestPath(goalPoints[0], goalPoints[1], DIRECTION::RIGHT));
  shortestPaths.push_back(graph.getShortestPath(goalPoints[1], goalPoints[0], DIRECTION::LEFT));
  shortestPaths.push_back(graph.getShortestPath(goalPoints[1], goalPoints[0], DIRECTION::RIGHT));

  for (auto path : shortestPaths){
    cout << "From " << path.verticesTaken.front() 
              << " facing "  << to_string(path.startDirection) 
              << " to " << path.verticesTaken.back() 
              << " has a distance of " << path.distance << endl;

    cout << "The path taken was: ";
    for (auto vertex : path.verticesTaken){
      cout << vertex << "->";
    }
    cout << endl;
  }

  return 0;
}