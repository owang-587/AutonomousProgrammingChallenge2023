#ifndef _VERTEX_H
#define _VERTEX_H_

#include <unordered_map>
#include <vector>
#include <string>

enum class DIRECTION{
    UP,
    RIGHT,
    DOWN,
    LEFT
};

struct Edge{
    Edge(DIRECTION startDirection, DIRECTION endDirection, double distance){
        this->startDirection = startDirection;
        this->endDirection = endDirection;
        this->distance = distance;
    }
    DIRECTION startDirection, endDirection;
    double distance;
};


class Vertex {
public:
    // creates an instance of the vertex
    Vertex(float x, float y, int layer);

    // adds an edge to the vertex with the given information
    void addEdge(std::string endId, DIRECTION startDirection, DIRECTION endDirection, double distance);

    // returns true if the edge is a valid connection from this vertex
    bool isEdge(std::string endId, DIRECTION startDirection) const;

    // returns the distance of the specified edge
    double getEdgeDistance(std::string endId, DIRECTION startDirection);

    // returns a vector of the id's of all the neighbours of this vertex
    std::vector<std::string> getNeighbours();
    

private:
    float x, y;
    int layer;
    std::unordered_map<std::string, std::vector<Edge>> edges;
};

#endif _VERTEX_H_