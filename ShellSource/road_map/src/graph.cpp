#include "graph.h"
#include <math.h>

void Graph::createVertex(std::string id, int x, int y, int layer) {
  vertices[id] = Vertex(x, y, layer);
}

void Graph::addEdge(std::string startId, std::string endId, DIRECTION startDirection, DIRECTION endDirection, double distance) {
  vertices[startId].addEdge(endId, startDirection, endDirection, distance);
}


bool Graph::isVertex(std::string id) const {
  return vertices.find(id) != vertices.end();
}

bool Graph::isEdge(std::string startId, std::string endId, DIRECTION startDirection) {
  return vertices[startId].isEdge(endId, startDirection);
}


std::vector<std::string> Graph::getNeighbours(std::string id)  {
  return vertices[id].getNeighbours();
}


int Graph::numNeighbours(std::string id)  {
  return vertices[id].getNeighbours().size();
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
