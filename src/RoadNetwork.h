#pragma once

#include "Math.h"
#include "RoadVertex.h"

class RoadNetwork
{
  private:
    prvVec roadVertices;

  public:
    RoadVertex *addRoadVertex(Vec2 pos, RoadVertex *parent);
    RoadVertex *addRoadVertex(Vec2 pos);
    RoadVertex *nearestVertex(Vec2 pos);
    void addEdge(RoadVertex *src, RoadVertex *dest);
    int verticesCount();
    void showVertices();
    inline auto begin() const
    {
        return roadVertices.begin();
    }
    inline auto end() const
    {
        return roadVertices.end();
    }
    ~RoadNetwork();
};
