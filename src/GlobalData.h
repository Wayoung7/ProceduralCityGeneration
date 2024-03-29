#pragma once

class GlobalData
{
  public:
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
