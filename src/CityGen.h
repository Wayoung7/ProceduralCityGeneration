#pragma once

#include "RoadSeg.h"
#include <SFML/Graphics.hpp>
#include <functional>
#include <list>
#include <optional>
#include <vector>

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
    std::vector<RoadSeg *> roads;
    std::list<RoadSeg> queue;

  public:
    void init();
    void step();
    void reset();
    void stop();
    RoadSeg PopSmallestInQueue();
    RoadSeg *localConstrain(RoadSeg &cur);
    // void cutThroughSeg(Vec2 intersection, RoadSeg cur, RoadSeg *seg);
    RoadSeg *handleIntersectionAction(RoadSeg &cur, RoadSeg *otherSeg, Vec2 intersection);
    RoadSeg *handleSnapAction(RoadSeg &cur, RoadSeg *otherSeg, Vec2 point);
    RoadSeg *handleExtendAction(RoadSeg &cur, RoadSeg *otherSeg, Vec2 intersection);
    std::vector<RoadSeg> globalGoal(RoadSeg *cur);
    RoadSeg *splitThroughSeg(Vec2 intersection, RoadSeg &cur, RoadSeg *seg);
    inline auto begin() const
    {
        return roads.begin();
    }
    inline auto end() const
    {
        return roads.end();
    }
    std::vector<RoadSeg *> filterRoads(std::function<bool(RoadSeg *)> func);
    ~CityGen();
};

// class Action
// {
//   public:
//     RoadSeg *seg;
//     Vec2 point;
//     virtual bool apply(RoadSeg *)
//     {
//     }
//     Action(RoadSeg *_seg, Vec2 _point) : seg(_seg), point(_point)
//     {
//     }
// };

// class IntersectionAction : public Action
// {
//   public:
//     bool apply(RoadSeg *) override;
//     IntersectionAction(RoadSeg *_seg, Vec2 _point) : Action(_seg, _point)
//     {
//     }
// };

// class SnapAction : public Action
// {
//   public:
//     bool apply(RoadSeg *) override
//     {
//     }
//     SnapAction(RoadSeg *_seg, Vec2 _point) : Action(_seg, _point)
//     {
//     }
// };

// class ExtendAction : public Action
// {
//   public:
//     bool apply(RoadSeg *) override
//     {
//     }
//     ExtendAction(RoadSeg *_seg, Vec2 _point) : Action(_seg, _point)
//     {
//     }
// };
