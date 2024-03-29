#pragma once

#include "RoadSeg.h"
#include <SFML/Graphics.hpp>
#include <functional>
#include <list>
#include <optional>

enum ActionType
{
    NullAction,
    IntersectionAction,
    SnapAction,
    ExtendAction,
};

class CityGen
{
  private:
    std::map<std::pair<int, int>, std::vector<RoadSeg *>> roads;
    std::list<RoadSeg> queue;
    int gridCellWidth;
    int segCount;

    void addRoad(RoadSeg *road);
    std::pair<int, int> calCell(const Vec2 &point);
    std::vector<RoadSeg *> queryNearbyRoads(const Vec2 &point);
    int roadsCount() const;
    RoadSeg PopSmallestInQueue();
    RoadSeg *localConstrain(RoadSeg &cur);
    RoadSeg *handleIntersectionAction(RoadSeg &cur, RoadSeg *otherSeg, Vec2 intersection);
    RoadSeg *handleSnapAction(RoadSeg &cur, RoadSeg *otherSeg, Vec2 point);
    RoadSeg *handleExtendAction(RoadSeg &cur, RoadSeg *otherSeg, Vec2 intersection);
    std::vector<RoadSeg> globalGoal(RoadSeg *cur) const;
    RoadSeg *splitThroughSeg(Vec2 intersection, RoadSeg &cur, RoadSeg *seg);
    std::vector<RoadSeg *> filterRoads(std::vector<RoadSeg *> src, std::function<bool(RoadSeg *)> func) const;

  public:
    void init();
    void step();
    void reset();
    void stop();
    inline auto begin() const
    {
        return roads.begin();
    }
    inline auto end() const
    {
        return roads.end();
    }
    inline int getSegCoung() const
    {
        return segCount;
    }
    ~CityGen();
};
