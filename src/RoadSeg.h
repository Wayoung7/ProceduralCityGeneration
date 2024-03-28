#pragma once

#include "Math.h"
#include <optional>
#include <vector>

enum RoadSegType
{
    NormalRoad,
    HighWay,
};

class RoadSeg
{
  private:
    Vec2 st;
    Vec2 ed;
    int t;
    RoadSegType type;
    bool isSevered = false;

    std::vector<RoadSeg *> stNeighbors;
    std::vector<RoadSeg *> edNeighbors;

  public:
    RoadSeg(Vec2 _st, Vec2 _ed, int _t, RoadSegType _type);
    RoadSeg(Vec2 _st, float dir, float len, int _t, RoadSegType _type);
    void addStNeighbor(RoadSeg *stNeighbor);
    void addStNeighbors(std::vector<RoadSeg *> _stNeighbors);
    void addEdNeighbor(RoadSeg *edNeighbor);
    void addEdNeighbors(std::vector<RoadSeg *> _edNeighbors);
    std::optional<Vec2> intersectWith(const RoadSeg &other) const;
    float getDir() const;
    inline float getLen() const
    {
        return st.distance(ed);
    }
    inline int getT() const
    {
        return t;
    }
    inline Vec2 getSt() const
    {
        return st;
    }
    inline void setSt(Vec2 _st)
    {
        st = _st;
    }
    inline Vec2 getEd() const
    {
        return ed;
    }
    inline void setEd(Vec2 _ed)
    {
        ed = _ed;
    }
    inline RoadSegType getType() const
    {
        return type;
    }
    inline void setType(RoadSegType _type)
    {
        type = _type;
    }
    inline std::vector<RoadSeg *> getStNeighbors() const
    {
        return stNeighbors;
    }
    inline std::vector<RoadSeg *> getEdNeighbors() const
    {
        return edNeighbors;
    }
    inline void clearStNeighbors()
    {
        stNeighbors = std::vector<RoadSeg *>();
    }
    inline void clearEdNeighbors()
    {
        edNeighbors = std::vector<RoadSeg *>();
    }
    inline void setSevered(bool _severed)
    {
        isSevered = _severed;
    }
    inline bool getSevered() const
    {
        return isSevered;
    }
    inline bool stNeighborHas(RoadSeg *input)
    {
        for (RoadSeg *seg : stNeighbors)
        {
            if (input == seg)
            {
                return true;
            }
        }
        return false;
    }
    inline bool edNeighborHas(RoadSeg *input)
    {
        for (RoadSeg *seg : edNeighbors)
        {
            if (input == seg)
            {
                return true;
            }
        }
        return false;
    }
};
