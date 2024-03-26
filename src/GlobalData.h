#pragma once

#include "RoadVertex.h"
#include <vector>

class GlobalData
{
  public:
    int m_roadSegs;
    bool isRunning = true;

    static GlobalData &getInstance()
    {
        static GlobalData instance;
        return instance;
    }
    GlobalData(GlobalData const &) = delete;
    void operator=(GlobalData const &) = delete;

  private:
    GlobalData()
    {
    }
};
