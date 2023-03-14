#include "../include/vertex.h"

#include <algorithm>


Vertex::Vertex(){}

Vertex::Vertex(bool isGoalPoint){
    this->isGoalPoint = isGoalPoint;
}

void Vertex::addEdge(std::string endId, DIRECTION startDirection, double distance){
    edges[endId] = Edge(startDirection, distance);
}

bool Vertex::isEdge(std::string endId) const{
    return edges.find(endId) != edges.end();
}

double Vertex::getEdgeDistance(std::string endId) {
    return edges[endId].distance;
}

std::unordered_map<std::string, Edge> Vertex::getEdges() const {
  return edges;
}

