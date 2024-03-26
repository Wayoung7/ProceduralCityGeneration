#include "RoadSeg.h"
#include "GlobalConfig.h"

RoadSeg::RoadSeg(Vec2 _st, Vec2 _ed, int _t, RoadSegType _type) : st(_st), ed(_ed), t(_t), type(_type)
{
}

RoadSeg::RoadSeg(Vec2 _st, float dir, float len, int _t, RoadSegType _type) : st(_st), ed(_st), t(_t), type(_type)
{
    if (_type == HighWay)
    {
        ed = _st + Vec2::fromAngle(dir) * GlobalConfig::getInstance().highwaySegLen;
    }
    else
    {
        ed = _st + Vec2::fromAngle(dir) * GlobalConfig::getInstance().normalSegLen;
    }
}

void RoadSeg::addStNeighbor(RoadSeg *stNeighbor)
{
    stNeighbors.push_back(stNeighbor);
}

void RoadSeg::addStNeighbors(std::vector<RoadSeg *> _stNeighbors)
{
    stNeighbors.insert(stNeighbors.end(), _stNeighbors.begin(), _stNeighbors.end());
}

void RoadSeg::addEdNeighbors(std::vector<RoadSeg *> _edNeighbors)
{
    edNeighbors.insert(edNeighbors.end(), _edNeighbors.begin(), _edNeighbors.end());
}

void RoadSeg::addEdNeighbor(RoadSeg *edNeighbor)
{
    edNeighbors.push_back(edNeighbor);
}

std::optional<Vec2> RoadSeg::intersectWith(const RoadSeg &other)
{
    std::optional<Vec2> res;

    float x1 = st.x;
    float y1 = st.y;
    float x2 = ed.x;
    float y2 = ed.y;
    float x3 = other.st.x;
    float y3 = other.st.y;
    float x4 = other.ed.x;
    float y4 = other.ed.y;
    float den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
    float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;
    if (0.f - EPS < u && u < 1.f + EPS && t > 0.f)
    {
        Vec2 r(x1 + t * (x2 - x1), y1 + t * (y2 - y1));

        if (!r.isEqualApprox(st) && !r.isEqualApprox(ed) && !r.isEqualApprox(other.st) && !r.isEqualApprox(other.ed))
        {
            res = r;
        }
    }

    return res;
}

float RoadSeg::getDir() const
{
    return (ed - st).toAngle();
}
