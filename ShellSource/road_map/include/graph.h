
#ifndef _GRAPH_H_
#define _GRAPH_H_

#include "vertex.h"

/*
  Represents a graph using an adjacency list representation.
  Vertices are id'd using integers.
*/
class Graph {
public:
  // No constructor or destructor are necessary this time.
  // A new instance will be an empty graph with no vertices.

  // creates a vertex and adds it to the map of vertices
  void createVertex(std::string id, int x, int y, int layer);

  // creates an edge and adds it to the vertex's list of edges
  // NOTE: this is a directed graph so this is a one-way connection
  void addEdge(std::string startId, std::string endId, DIRECTION startDirection, DIRECTION endDirection, double distance);

  // returns true if and only if the id is a vertex in the graph
  bool isVertex(std::string id) const;

  // returns true if and only if the edge data passed lines up with an edge in the graph
  // will certainly return false if neither vertex is in the graph
  bool isEdge(std::string startId, std::string endId, DIRECTION startDirection) ;

  // returns a vector of the id's of the neighbours of the specified vertex
  std::vector<std::string> getNeighbours(std::string id) ;

  // return the number of neighbours of v
  int numNeighbours(std::string id) ;

  // returns the number of vertices in the graph
  int size() const;

  // return a vector with the id of all vertices
  std::vector<std::string> getVertices() ;

private:

  std::unordered_map<std::string, Vertex> vertices;
  
};

#endif _GRAPH_H_
