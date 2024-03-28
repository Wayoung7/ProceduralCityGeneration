#pragma once

#include "Math.h"

class GlobalConfig
{
  public:
    float minSegLen;
    float maxSegLen;
    int segLimit;
    int maxRoadCross;
    int windowWidth;
    int windowHeight;
    int normalSegLen;
    int highwaySegLen;
    float branchAngleDev;
    float straightAngleDev;
    float normalBranchProb;
    float highwayBranchProb;
    float maxSnapDist;
    float minIntersectionDev;
    int normalDelay;

    inline bool inside(Vec2 p_pos) const
    {
        return p_pos.x >= 0 && p_pos.x < windowWidth && p_pos.y >= 0 && p_pos.y < windowHeight;
    }

    static GlobalConfig &getInstance()
    {
        static GlobalConfig instance;
        return instance;
    }

    GlobalConfig(GlobalConfig const &) = delete;
    void operator=(GlobalConfig const &) = delete;

  private:
    GlobalConfig()
        : segLimit(29999), windowWidth(1920), windowHeight(1080), normalSegLen(26), highwaySegLen(31),
          branchAngleDev(0.11f), straightAngleDev(0.15f), normalBranchProb(0.35f), highwayBranchProb(0.06f),
          maxSnapDist(4), minIntersectionDev(0.55f), normalDelay(2)
    {
    }
};
