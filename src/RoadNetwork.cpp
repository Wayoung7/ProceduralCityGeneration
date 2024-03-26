#include "RoadNetwork.h"
#include <iostream>

RoadVertex *RoadNetwork::addRoadVertex(Vec2 pos, RoadVertex *parent)
{
    RoadVertex *newVertex = new RoadVertex(pos);
    newVertex->addNeighbor(parent);
    parent->addNeighbor(newVertex);
    roadVertices.emplace_back(newVertex);
    return newVertex;
}

RoadVertex *RoadNetwork::addRoadVertex(Vec2 pos)
{
    RoadVertex *newVertex = new RoadVertex(pos);
    roadVertices.emplace_back(newVertex);
    return newVertex;
}

RoadVertex *RoadNetwork::nearestVertex(Vec2 pos)
{
    int min = __INT_MAX__;
    RoadVertex *near = nullptr;
    for (RoadVertex *v : roadVertices)
    {
        if (pos.distanceSquared(v->getPos()) < min)
        {
            min = pos.distanceSquared(v->getPos());
            near = v;
        }
    }
    return near;
}

void RoadNetwork::addEdge(RoadVertex *src, RoadVertex *dest)
{
    src->addNeighbor(dest);
    dest->addNeighbor(src);
}

int RoadNetwork::verticesCount()
{
    return roadVertices.size();
}

void RoadNetwork::showVertices()
{
    for (RoadVertex *v : roadVertices)
    {
        std::cout << v->getPos().x << ", " << v->getPos().y << std::endl;
    }
}

RoadNetwork::~RoadNetwork()
{
    for (RoadVertex *v : roadVertices)
    {
        delete v;
    }
}
