#pragma once

#include "RoadNetwork.h"
#include "RoadVertex.h"
#include <queue>

class Generator
{
  private:
    std::queue<RoadVertex> queue;

  public:
    void init(RoadNetwork &rn);
    void step(RoadNetwork &rn);
};
