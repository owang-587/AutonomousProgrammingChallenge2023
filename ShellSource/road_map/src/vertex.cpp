#include "vertex.h"

#include <algorithm>

Vertex::Vertex(float x, float y, int layer){
    this->x = x;
    this->y = y;
    this->layer = layer;
}

void Vertex::addEdge(std::string endId, DIRECTION startDirection, DIRECTION endDirection, double distance){
    if (!isEdge(endId, startDirection)){
        Edge newEdge(startDirection, endDirection, distance);
        edges[endId].push_back(newEdge);
    }
}

bool Vertex::isEdge(std::string endId, DIRECTION startDirection) const{
    return find(edges.begin(), edges.end(), endId) != edges.end();
}

double Vertex::getEdgeDistance(std::string endId, DIRECTION startDirection){
    if (!isEdge(endId, startDirection)){
        // probably should have an early return here but idk
    }

    for (auto edge : edges[endId]){
        if (edge.startDirection == startDirection){
            return edge.distance;
        }
    }
}

std::vector<std::string> Vertex::getNeighbours(){

  std::vector<std::string> neighbours;

  for (auto it = edges.begin();
    it != edges.end(); it++) {
      neighbours.push_back(it->first);
    }

  return neighbours;
}