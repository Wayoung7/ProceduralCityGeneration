#pragma once

#include <bits/stdc++.h>

using namespace std;

class RoadSegmentProperty
{
  public:
    pair<int, int> x;
    pair<int, int> y;
    int level;
    RoadSegmentProperty(pair<int, int> _x, pair<int, int> _y, int _level)
    {
        x = _x;
        y = _y;
        level = _level;
    }
    RoadSegmentProperty();
    ~RoadSegmentProperty();
};

inline RoadSegmentProperty::RoadSegmentProperty()
{
}

RoadSegmentProperty::~RoadSegmentProperty()
{
}

class WaitingSeg
{
  public:
    float t;
    RoadSegmentProperty r;
    WaitingSeg(float _t, RoadSegmentProperty _r)
    {
        t = _t;
        r = _r;
    }
};
