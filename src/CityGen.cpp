#include "CityGen.h"
#include "GlobalConfig.h"
#include <iostream>

void CityGen::init()
{
    const GlobalConfig &cfg = GlobalConfig::getInstance();
    RoadSeg *first = new RoadSeg(Vec2(cfg.windowWidth / 2.f, cfg.windowHeight / 2.f),
                                 Vec2(cfg.windowWidth / 2.f + cfg.highwaySegLen, cfg.windowHeight / 2.f), 0, HighWay);
    RoadSeg *second = new RoadSeg(Vec2(cfg.windowWidth / 2.f, cfg.windowHeight / 2.f),
                                  Vec2(cfg.windowWidth / 2.f - cfg.highwaySegLen, cfg.windowHeight / 2.f), 0, HighWay);
    first->addStNeighbor(second);
    second->addStNeighbor(first);
    queue.emplace_back(first);
    queue.emplace_back(second);
}

void CityGen::step()
{
    if (roads.size() < GlobalConfig::getInstance().segLimit && queue.empty() == false)
    {
        // Debug
        // std::cout << roads.size() << ", " << queue.size() << std::endl;

        // Pop smallest t in queue
        RoadSeg *cur = PopSmallestInQueue();
        std::cout << "(" << cur->getSt().x << ", " << cur->getSt().y << ")"
                  << ", "
                  << "(" << cur->getEd().x << ", " << cur->getEd().y << ")" << std::endl;
        if (localConstrain(cur))
        {
            std::vector<RoadSeg *> newRoads = globalGoal(cur);
            for (RoadSeg *newRoad : newRoads)
            {
                queue.push_back(newRoad);
            }
        }
        else
        {
            delete cur;
        }
    }
}

RoadSeg *CityGen::PopSmallestInQueue()
{
    RoadSeg *smallest = nullptr;
    int min = __INT_MAX__;
    for (RoadSeg *seg : queue)
    {
        if (seg->getT() < min)
        {
            min = seg->getT();
            smallest = seg;
        }
    }
    queue.remove(smallest);
    return smallest;
}

bool CityGen::localConstrain(RoadSeg *cur)
{
    ActionType at = NullAction;
    RoadSeg *_otherSeg = nullptr;
    Vec2 _point;
    int actionPrio = 0;
    float minIntersectDistSquared = __INT_MAX__;
    Vec2 curSt = cur->getSt();
    Vec2 curEd = cur->getEd();
    Vec2 mid = (curSt + curEd) / 2.f;
    float curLen = curSt.distance(curEd);

    std::vector<RoadSeg *> others = filterRoads([mid, curLen](RoadSeg *seg) -> bool {
        return segInCircle(seg->getSt(), seg->getEd(), mid, GlobalConfig::getInstance().maxSnapDist + curLen / 2.f);
    });

    for (RoadSeg *otherSeg : others)
    {
        // Check intersection
        if (actionPrio <= 4)
        {
            std::optional<Vec2> intersection = cur->intersectWith(*otherSeg);
            if (intersection.has_value())
            {
                float intersectionDistSquared = curSt.distanceSquared(intersection.value());
                if (intersectionDistSquared < minIntersectDistSquared)
                {
                    minIntersectDistSquared = intersectionDistSquared;
                    actionPrio = 4;
                    at = IntersectionAction;
                    _otherSeg = otherSeg;
                    _point = intersection.value();
                }
            }
        }
        // Check snap
        if (actionPrio <= 3)
        {
            if (curEd.distanceSquared(otherSeg->getEd()) <
                GlobalConfig::getInstance().maxSnapDist * GlobalConfig::getInstance().maxSnapDist)
            {
                actionPrio = 3;
                at = SnapAction;
                _otherSeg = otherSeg;
                _point = otherSeg->getEd();
            }
        }
        // Check extend
        if (actionPrio <= 2)
        {
            if (isPointInSegRange(curEd, otherSeg->getSt(), otherSeg->getEd()))
            {
                Vec2 intersection = closePointToLine(curEd, otherSeg->getSt(), otherSeg->getEd());
                float distSquared = curEd.distanceSquared(intersection);
                if (distSquared < GlobalConfig::getInstance().maxSnapDist * GlobalConfig::getInstance().maxSnapDist)
                {
                    actionPrio = 2;
                    at = ExtendAction;
                    _otherSeg = otherSeg;
                    _point = intersection;
                }
            }
        }
    }

    if (at != NullAction)
    {
        if (at == IntersectionAction)
        {
            handleIntersectionAction(cur, _otherSeg, _point);
        }
        else if (at == SnapAction)
        {
            handleSnapAction(cur, _otherSeg, _point);
        }
        else if (at == ExtendAction)
        {
            handleExtendAction(cur, _otherSeg, _point);
        }
    }
    else
    {
        roads.push_back(cur);
    }
    return true;
}

void CityGen::cutThroughSeg(Vec2 intersection, RoadSeg *cur, RoadSeg *seg)
{
    Vec2 st = seg->getSt();
    Vec2 ed = seg->getEd();
    seg->setSt(intersection);
    roads.push_back(seg);
    RoadSeg *segRemain = new RoadSeg(st, intersection, 0, seg->getType());
    roads.push_back(segRemain);
    cur->setEd(intersection);
    roads.push_back(cur);
    segRemain->addStNeighbors(seg->getStNeighbors());
    seg->clearStNeighbors();
    seg->addStNeighbor(segRemain);
    seg->addStNeighbor(cur);
    segRemain->addEdNeighbor(seg);
    segRemain->addEdNeighbor(cur);
    cur->addEdNeighbor(seg);
    cur->addEdNeighbor(segRemain);
    if (intersection.distanceSquared(cur->getEd()) > GlobalConfig::getInstance().maxSnapDist + 1.f)
    {
        RoadSeg *curRemain = new RoadSeg(intersection, cur->getEd(), 0, cur->getType());
        segRemain->addEdNeighbor(curRemain);
        curRemain->addStNeighbor(cur);
        curRemain->addStNeighbor(seg);
        curRemain->addStNeighbor(segRemain);
        cur->addEdNeighbor(curRemain);
        seg->addStNeighbor(curRemain);
        roads.push_back(curRemain);
    }
}

void CityGen::splitThroughSeg(Vec2 intersection, RoadSeg *cur, RoadSeg *seg)
{
    Vec2 st = seg->getSt();
    Vec2 ed = seg->getEd();
    seg->setSt(intersection);
    roads.push_back(seg);
    RoadSeg *segRemain = new RoadSeg(st, intersection, 0, seg->getType());
    roads.push_back(segRemain);
    cur->setEd(intersection);
    roads.push_back(cur);
    segRemain->addStNeighbors(seg->getStNeighbors());
    seg->clearStNeighbors();
    seg->addStNeighbor(segRemain);
    seg->addStNeighbor(cur);
    segRemain->addEdNeighbor(seg);
    segRemain->addEdNeighbor(cur);
    cur->addEdNeighbor(seg);
    cur->addEdNeighbor(segRemain);
}

bool CityGen::handleIntersectionAction(RoadSeg *cur, RoadSeg *otherSeg, Vec2 intersection)
{
    if (angleDiff(otherSeg->getDir(), cur->getDir()) < GlobalConfig::getInstance().minIntersectionDev)
    {
        return false;
    }
    cutThroughSeg(intersection, cur, otherSeg);
    cur->setSevered(true);
    return true;
}

bool CityGen::handleSnapAction(RoadSeg *cur, RoadSeg *otherSeg, Vec2 point)
{
    if (otherSeg->edNeighborHas(cur))
    {
        return false;
    }
    cur->addEdNeighbors(otherSeg->getEdNeighbors());
    cur->addEdNeighbor(otherSeg);
    otherSeg->addEdNeighbor(cur);
    for (RoadSeg *otherNeighbor : otherSeg->getEdNeighbors())
    {
        if (otherNeighbor->stNeighborHas(otherSeg))
        {
            otherNeighbor->addStNeighbor(cur);
        }
        else if (otherNeighbor->edNeighborHas(otherSeg))
        {
            otherNeighbor->addEdNeighbor(cur);
        }
    }
    roads.push_back(cur);
    cur->setSevered(true);
    return true;
}

bool CityGen::handleExtendAction(RoadSeg *cur, RoadSeg *otherSeg, Vec2 intersection)
{
    if (angleDiff(otherSeg->getDir(), cur->getDir()) < GlobalConfig::getInstance().minIntersectionDev)
    {
        return false;
    }
    splitThroughSeg(intersection, cur, otherSeg);
    cur->setSevered(true);
    return true;
}

std::vector<RoadSeg *> CityGen::globalGoal(RoadSeg *cur)
{
    std::vector<RoadSeg *> res;
    if (cur->getSevered())
    {
        return res;
    }
    RoadSeg *straight = nullptr;
    if (cur->getType() == HighWay)
    {
        straight = new RoadSeg(cur->getEd(),
                               RNG::randFloatFromRange(cur->getDir() - GlobalConfig::getInstance().straightAngleDev,
                                                       cur->getDir() + GlobalConfig::getInstance().straightAngleDev),
                               GlobalConfig::getInstance().highwaySegLen, cur->getT() + 1, HighWay);
    }
    else if (cur->getType() == NormalRoad)
    {
        straight = new RoadSeg(cur->getEd(),
                               RNG::randFloatFromRange(cur->getDir() - GlobalConfig::getInstance().straightAngleDev,
                                                       cur->getDir() + GlobalConfig::getInstance().straightAngleDev),
                               GlobalConfig::getInstance().normalSegLen,
                               cur->getT() + 1 + GlobalConfig::getInstance().normalDelay, NormalRoad);
    }
    if (straight)
        res.push_back(straight);
    if (cur->getType() == HighWay)
    {
        if (RNG::randFloatFromRange(0.f, 1.f) < GlobalConfig::getInstance().highwayBranchProb)
        {
            if (RNG::randFloatFromRange(0.f, 1.f) < 0.3f)
            {
                res.push_back(new RoadSeg(
                    cur->getEd(),
                    RNG::randFloatFromRange(cur->getDir() + M_PI / 2.f - GlobalConfig::getInstance().branchAngleDev,
                                            cur->getDir() + M_PI / 2.f + GlobalConfig::getInstance().branchAngleDev),
                    GlobalConfig::getInstance().highwaySegLen, cur->getT() + 1, HighWay));
            }
            else
            {
                res.push_back(new RoadSeg(
                    cur->getEd(),
                    RNG::randFloatFromRange(cur->getDir() + M_PI / 2.f - GlobalConfig::getInstance().branchAngleDev,
                                            cur->getDir() + M_PI / 2.f + GlobalConfig::getInstance().branchAngleDev),
                    GlobalConfig::getInstance().normalSegLen, cur->getT() + 1 + GlobalConfig::getInstance().normalDelay,
                    NormalRoad));
            }
        }
        if (RNG::randFloatFromRange(0.f, 1.f) < GlobalConfig::getInstance().highwayBranchProb)
        {
            if (RNG::randFloatFromRange(0.f, 1.f) < 0.3f)
            {
                res.push_back(new RoadSeg(
                    cur->getEd(),
                    RNG::randFloatFromRange(cur->getDir() - M_PI / 2.f - GlobalConfig::getInstance().branchAngleDev,
                                            cur->getDir() - M_PI / 2.f + GlobalConfig::getInstance().branchAngleDev),
                    GlobalConfig::getInstance().highwaySegLen, cur->getT() + 1, HighWay));
            }
            else
            {
                res.push_back(new RoadSeg(
                    cur->getEd(),
                    RNG::randFloatFromRange(cur->getDir() - M_PI / 2.f - GlobalConfig::getInstance().branchAngleDev,
                                            cur->getDir() - M_PI / 2.f + GlobalConfig::getInstance().branchAngleDev),
                    GlobalConfig::getInstance().normalSegLen, cur->getT() + 1 + GlobalConfig::getInstance().normalDelay,
                    NormalRoad));
            }
        }
    }
    else if (cur->getType() == NormalRoad)
    {
        if (RNG::randFloatFromRange(0.f, 1.f) < GlobalConfig::getInstance().normalBranchProb)
        {
            res.push_back(new RoadSeg(
                cur->getEd(),
                RNG::randFloatFromRange(cur->getDir() + M_PI / 2.f - GlobalConfig::getInstance().branchAngleDev,
                                        cur->getDir() + M_PI / 2.f + GlobalConfig::getInstance().branchAngleDev),
                GlobalConfig::getInstance().normalSegLen, cur->getT() + 1 + GlobalConfig::getInstance().normalDelay,
                NormalRoad));
        }
        if (RNG::randFloatFromRange(0.f, 1.f) < GlobalConfig::getInstance().normalBranchProb)
        {
            res.push_back(new RoadSeg(
                cur->getEd(),
                RNG::randFloatFromRange(cur->getDir() - M_PI / 2.f - GlobalConfig::getInstance().branchAngleDev,
                                        cur->getDir() - M_PI / 2.f + GlobalConfig::getInstance().branchAngleDev),
                GlobalConfig::getInstance().normalSegLen, cur->getT() + 1 + GlobalConfig::getInstance().normalDelay,
                NormalRoad));
        }
    }
    for (RoadSeg *seg : res)
    {
        seg->addStNeighbor(cur);
    }
    return res;
}

std::vector<RoadSeg *> CityGen::filterRoads(std::function<bool(RoadSeg *)> func)
{
    std::vector<RoadSeg *> res;
    for (RoadSeg *seg : roads)
    {
        if (func(seg))
        {
            res.push_back(seg);
        }
    }
    return res;
}

CityGen::~CityGen()
{
    for (RoadSeg *seg : roads)
    {
        delete seg;
    }

    for (RoadSeg *seg : queue)
    {
        delete seg;
    }
}

// bool IntersectionAction::apply(RoadSeg *cur)
// {
//     if (angleDiff(this->seg->getDir(), cur->getDir()) < GlobalConfig::getInstance().minIntersectionDev)
//     {
//         return false;
//     }
// }
