#include "RoadVertex.h"

RoadVertex::RoadVertex(Vec2 _pos)
{
    pos = _pos;
}

RoadVertex::RoadVertex(Vec2 _pos, RoadVertex *_parent)
{
    pos = _pos;
    neighbors.emplace_back(_parent);
}

prvVec RoadVertex::getNeighbors()
{
    return neighbors;
}

void RoadVertex::addNeighbor(RoadVertex *neighbor)
{
    neighbors.emplace_back(neighbor);
}
