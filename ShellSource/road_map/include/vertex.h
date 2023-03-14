#ifndef _VERTEX_H
#define _VERTEX_H_

#include <string>
#include <unordered_map>
#include <vector>


enum class DIRECTION{
    UP,
    RIGHT,
    DOWN,
    LEFT
};

struct Edge{
    Edge(){}
    Edge(DIRECTION startDirection, double distance){
        this->startDirection = startDirection;
        this->distance = distance;
    }

    DIRECTION startDirection;
    double distance;
};


class Vertex {
public:
    bool isGoalPoint;
    Vertex();
    // creates an instance of the vertex
    Vertex(bool isGoalPoint);

    // adds an edge to the vertex with the given information
    void addEdge(std::string endId, DIRECTION startDirection, double distance);

    // returns true if the edge is a valid connection from this vertex
    bool isEdge(std::string endId) const;

    // returns the distance of the specified edge
    double getEdgeDistance(std::string endId) ;

    // returns an unordered map of the neighbours and edges connecting them
    std::unordered_map<std::string, Edge> getEdges() const ;

private:
    
    std::unordered_map<std::string, Edge> edges;
    
};

#endif