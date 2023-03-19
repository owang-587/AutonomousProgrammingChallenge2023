
#ifndef _GRAPH_H_
#define _GRAPH_H_

#include "vertex.h"

// make a workingpath inside of graph.cpp
// remove the first constructor as it will be taken care of
// use the working path to carry on 
struct Path
{
  Path(double distance, std::vector<std::string> verticesTaken, DIRECTION startDirection, DIRECTION endDirection);

  double distance;
  std::vector<std::string> verticesTaken;
  DIRECTION startDirection, endDirection;
};




class Graph {
public:
  // No constructor or destructor are necessary this time.
  // A new instance will be an empty graph with no vertices.

  // creates an intersection and adds it to the map of vertices
  void createVertex(std::string id, bool isGoalPoint);

  // creates an edge and adds it to the vertex's list of edges
  // NOTE: this is a directed graph so this is a one-way connection
  void addEdge(std::string startId, std::string endId, DIRECTION startDirection, double distance);

  // returns true if and only if the id is a vertex in the graph
  bool isVertex(std::string id) const;

  // returns true if and only if the edge data passed lines up with an edge in the graph
  // will certainly return false if neither vertex is in the graph
  bool isEdge(std::string startId, std::string endId, DIRECTION startDirection) ;

  // return the number of neighbours of v
  int numNeighbours(std::string id);

  // returns the number of vertices in the graph
  int size() const;

  // return a vector of the id's of all vertices
  std::vector<std::string> getVertices();

  // only get the vertices that are goal points
  std::vector<std::string> getGoalpoints();

  // makes the entire search tree
  void createSearchTree();

  // updates the search tree for one starting vertex
  void updateSearchTree(std::string startId);

  std::vector<Path> getShortestPaths(std::string startId, std::string endId);

private:
  int numOfGoalPoints = 0;
  std::unordered_map<std::string, Vertex> vertices;

  /*
    there might be a better way to do this, but essentially first key is starting id,
    second key is end id, and the pair has the distance and path taken
  */
  std::unordered_map<
    std::string, 
    std::unordered_map<
      std::string,
      std::vector<Path>
    >
  > searchTree;

};

#endif
