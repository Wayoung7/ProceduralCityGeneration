#pragma once

#include <vector>

class GlobalData
{
  public:
    int m_roadSegs;
    bool isRunning = true;
    bool isFinished = false;

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
