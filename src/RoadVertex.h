#pragma once

#include "Math.h"
#include <vector>

class RoadVertex;

typedef std::vector<RoadVertex *> prvVec;

class RoadVertex
{
  private:
    Vec2 pos;
    prvVec neighbors;

  public:
    RoadVertex()
    {
    }
    RoadVertex(Vec2 _pos);
    RoadVertex(Vec2 _pos, RoadVertex *_parent);
    prvVec getNeighbors();
    void addNeighbor(RoadVertex *neighbor);
    inline Vec2 getPos()
    {
        return pos;
    }
    inline void setPos(Vec2 _pos)
    {
        pos = _pos;
    }
    inline int neighborCount()
    {
        return neighbors.size();
    }
};
