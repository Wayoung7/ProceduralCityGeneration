#include "CityGen.h"
#include "GlobalConfig.h"
#include "GlobalData.h"
#include <chrono>
#include <iostream>

void CityGen::init()
{
    const GlobalConfig &cfg = GlobalConfig::getInstance();
    RoadSeg *root = new RoadSeg(Vec2(cfg.windowWidth / 2.f, cfg.windowHeight / 2.f),
                                Vec2(cfg.windowWidth / 2.f + cfg.highwaySegLen, cfg.windowHeight / 2.f), 0, HighWay);
    RoadSeg first = RoadSeg(Vec2(cfg.windowWidth / 2.f + cfg.highwaySegLen, cfg.windowHeight / 2.f),
                            Vec2(cfg.windowWidth / 2.f + 2 * cfg.highwaySegLen, cfg.windowHeight / 2.f), 0, HighWay);
    RoadSeg second = RoadSeg(Vec2(cfg.windowWidth / 2.f, cfg.windowHeight / 2.f),
                             Vec2(cfg.windowWidth / 2.f - cfg.highwaySegLen, cfg.windowHeight / 2.f), 0, HighWay);
    first.addStNeighbor(root);
    second.addStNeighbor(root);
    queue.emplace_back(first);
    queue.emplace_back(second);
    roads.emplace_back(root);
}

void CityGen::step()
{
    if (GlobalData::getInstance().isFinished == false && roads.size() < GlobalConfig::getInstance().segLimit &&
        queue.empty() == false)
    {

        RoadSeg cur = PopSmallestInQueue();

        // std::cout << "(" << cur->getSt().x << ", " << cur->getSt().y << ")"
        //           << ", "
        //           << "(" << cur->getEd().x << ", " << cur->getEd().y << ")" << std::endl;
        RoadSeg *newSeg = localConstrain(cur);

        if (newSeg)
        {
            std::vector<RoadSeg> newRoads = globalGoal(newSeg);
            for (RoadSeg newRoad : newRoads)
            {
                queue.push_back(newRoad);
            }
        }
    }
    else
    {
        GlobalData::getInstance().isFinished = true;
    }
}

void CityGen::reset()
{
    for (RoadSeg *road : roads)
    {
        delete road;
    }
    roads.clear();
    queue.clear();
    this->init();
    GlobalData::getInstance().isFinished = false;
}

void CityGen::stop()
{
    GlobalData::getInstance().isFinished = true;
}

RoadSeg CityGen::PopSmallestInQueue()
{
    RoadSeg smallest = queue.front();
    int min = __INT_MAX__;
    int idx = 0;
    unsigned smallestIdx = 0;
    for (RoadSeg seg : queue)
    {
        if (seg.getT() < min)
        {
            min = seg.getT();
            smallest = seg;
            smallestIdx = idx;
        }
        idx++;
    }
    unsigned i = 0;
    for (std::list<RoadSeg>::iterator it = queue.begin(); it != queue.end(); i++, it++)
    {
        if (i == smallestIdx)
        {
            queue.erase(it);
            break;
        }
    }
    // queue.remove(smallest);
    return smallest;
}

RoadSeg *CityGen::localConstrain(RoadSeg &cur)
{
    ActionType at = NullAction;
    RoadSeg *_otherSeg = nullptr;
    Vec2 _point;
    int actionPrio = 0;
    float minIntersectDistSquared = __INT_MAX__;
    Vec2 curSt = cur.getSt();
    Vec2 curEd = cur.getEd();
    Vec2 mid = (curSt + curEd) / 2.f;
    float curLen = curSt.distance(curEd);

    std::vector<RoadSeg *> others = filterRoads([mid, curLen](RoadSeg *seg) -> bool {
        float range = GlobalConfig::getInstance().maxSnapDist + curLen / 2.f;
        if (std::abs(seg->getSt().x - mid.x) > range + curLen / 2.f ||
            std::abs(seg->getSt().y - mid.y) > range + curLen / 2.f)
        {
            return false;
        }
        return segInCircle(seg->getSt(), seg->getEd(), mid, range);
    });

    for (RoadSeg *otherSeg : others)
    {
        // Check intersection
        if (actionPrio <= 4)
        {
            std::optional<Vec2> intersection = cur.intersectWith(*otherSeg);
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

    RoadSeg *newSeg = nullptr;

    if (at != NullAction)
    {
        if (at == IntersectionAction)
        {
            newSeg = handleIntersectionAction(cur, _otherSeg, _point);
        }
        else if (at == SnapAction)
        {
            newSeg = handleSnapAction(cur, _otherSeg, _point);
        }
        else if (at == ExtendAction)
        {
            newSeg = handleExtendAction(cur, _otherSeg, _point);
        }
    }
    else
    {
        newSeg = new RoadSeg(cur);
        roads.push_back(newSeg);
    }
    return newSeg;
}

// void CityGen::cutThroughSeg(Vec2 intersection, RoadSeg cur, RoadSeg *seg)
// {
//     Vec2 st = seg->getSt();
//     Vec2 ed = seg->getEd();
//     seg->setSt(intersection);
//     roads.push_back(seg);
//     RoadSeg *segRemain = new RoadSeg(st, intersection, 0, seg->getType());
//     roads.push_back(segRemain);
//     cur->setEd(intersection);
//     roads.push_back(cur);
//     segRemain->addStNeighbors(seg->getStNeighbors());
//     seg->clearStNeighbors();
//     seg->addStNeighbor(segRemain);
//     seg->addStNeighbor(cur);
//     segRemain->addEdNeighbor(seg);
//     segRemain->addEdNeighbor(cur);
//     cur->addEdNeighbor(seg);
//     cur->addEdNeighbor(segRemain);
//     if (intersection.distanceSquared(cur->getEd()) > GlobalConfig::getInstance().maxSnapDist + 1.f)
//     {
//         RoadSeg *curRemain = new RoadSeg(intersection, cur->getEd(), 0, cur->getType());
//         segRemain->addEdNeighbor(curRemain);
//         curRemain->addStNeighbor(cur);
//         curRemain->addStNeighbor(seg);
//         curRemain->addStNeighbor(segRemain);
//         cur->addEdNeighbor(curRemain);
//         seg->addStNeighbor(curRemain);
//         roads.push_back(curRemain);
//     }
// }

RoadSeg *CityGen::splitThroughSeg(Vec2 intersection, RoadSeg &cur, RoadSeg *seg)
{
    RoadSeg *newSeg = new RoadSeg(cur);
    Vec2 st = seg->getSt();
    Vec2 ed = seg->getEd();
    seg->setSt(intersection);
    // roads.push_back(seg);
    RoadSeg *segRemain = new RoadSeg(st, intersection, 0, seg->getType());
    roads.push_back(segRemain);
    roads.push_back(newSeg);

    newSeg->setEd(intersection);
    segRemain->addStNeighbors(seg->getStNeighbors());
    seg->clearStNeighbors();
    seg->addStNeighbor(segRemain);
    seg->addStNeighbor(newSeg);
    segRemain->addEdNeighbor(seg);
    segRemain->addEdNeighbor(newSeg);
    newSeg->addEdNeighbor(seg);
    newSeg->addEdNeighbor(segRemain);
    newSeg->setSevered(true);
    cur = *newSeg;

    return newSeg;
}

RoadSeg *CityGen::handleIntersectionAction(RoadSeg &cur, RoadSeg *otherSeg, Vec2 intersection)
{
    if (angleDiff(otherSeg->getDir(), cur.getDir()) < GlobalConfig::getInstance().minIntersectionDev)
    {
        return nullptr;
    }
    return splitThroughSeg(intersection, cur, otherSeg);
}

RoadSeg *CityGen::handleSnapAction(RoadSeg &cur, RoadSeg *otherSeg, Vec2 point)
{
    // if (otherSeg->edNeighborHas(cur))
    // {
    //     return false;
    // }
    RoadSeg *newSeg = new RoadSeg(cur);
    newSeg->addEdNeighbors(otherSeg->getEdNeighbors());
    newSeg->addEdNeighbor(otherSeg);
    otherSeg->addEdNeighbor(newSeg);
    for (RoadSeg *otherNeighbor : otherSeg->getEdNeighbors())
    {
        if (otherNeighbor->stNeighborHas(otherSeg))
        {
            otherNeighbor->addStNeighbor(newSeg);
        }
        else if (otherNeighbor->edNeighborHas(otherSeg))
        {
            otherNeighbor->addEdNeighbor(newSeg);
        }
    }
    newSeg->setSevered(true);
    roads.push_back(newSeg);
    cur = *newSeg;
    return newSeg;
}

RoadSeg *CityGen::handleExtendAction(RoadSeg &cur, RoadSeg *otherSeg, Vec2 intersection)
{
    if (angleDiff(otherSeg->getDir(), cur.getDir()) < GlobalConfig::getInstance().minIntersectionDev)
    {
        return nullptr;
    }
    return splitThroughSeg(intersection, cur, otherSeg);
}

std::vector<RoadSeg> CityGen::globalGoal(RoadSeg *cur)
{
    std::vector<RoadSeg> res;
    if (cur->getSevered())
    {
        return res;
    }

    if (cur->getType() == HighWay)
    {
        res.push_back(RoadSeg(cur->getEd(),
                              RNG::randFloatFromRange(cur->getDir() - GlobalConfig::getInstance().straightAngleDev,
                                                      cur->getDir() + GlobalConfig::getInstance().straightAngleDev),
                              GlobalConfig::getInstance().highwaySegLen, cur->getT() + 1, HighWay));
    }
    else if (cur->getType() == NormalRoad)
    {
        res.push_back(RoadSeg(cur->getEd(),
                              RNG::randFloatFromRange(cur->getDir() - GlobalConfig::getInstance().straightAngleDev,
                                                      cur->getDir() + GlobalConfig::getInstance().straightAngleDev),
                              GlobalConfig::getInstance().normalSegLen,
                              cur->getT() + 1 + GlobalConfig::getInstance().normalDelay, NormalRoad));
    }

    if (cur->getType() == HighWay)
    {
        if (RNG::randFloatFromRange(0.f, 1.f) < GlobalConfig::getInstance().highwayBranchProb)
        {
            if (RNG::randFloatFromRange(0.f, 1.f) < 0.3f)
            {
                res.push_back(RoadSeg(
                    cur->getEd(),
                    RNG::randFloatFromRange(cur->getDir() + M_PI / 2.f - GlobalConfig::getInstance().branchAngleDev,
                                            cur->getDir() + M_PI / 2.f + GlobalConfig::getInstance().branchAngleDev),
                    GlobalConfig::getInstance().highwaySegLen, cur->getT() + 1, HighWay));
            }
            else
            {
                res.push_back(RoadSeg(
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
                res.push_back(RoadSeg(
                    cur->getEd(),
                    RNG::randFloatFromRange(cur->getDir() - M_PI / 2.f - GlobalConfig::getInstance().branchAngleDev,
                                            cur->getDir() - M_PI / 2.f + GlobalConfig::getInstance().branchAngleDev),
                    GlobalConfig::getInstance().highwaySegLen, cur->getT() + 1, HighWay));
            }
            else
            {
                res.push_back(RoadSeg(
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
            res.push_back(RoadSeg(
                cur->getEd(),
                RNG::randFloatFromRange(cur->getDir() + M_PI / 2.f - GlobalConfig::getInstance().branchAngleDev,
                                        cur->getDir() + M_PI / 2.f + GlobalConfig::getInstance().branchAngleDev),
                GlobalConfig::getInstance().normalSegLen, cur->getT() + 1 + GlobalConfig::getInstance().normalDelay,
                NormalRoad));
        }
        if (RNG::randFloatFromRange(0.f, 1.f) < GlobalConfig::getInstance().normalBranchProb)
        {
            res.push_back(RoadSeg(
                cur->getEd(),
                RNG::randFloatFromRange(cur->getDir() - M_PI / 2.f - GlobalConfig::getInstance().branchAngleDev,
                                        cur->getDir() - M_PI / 2.f + GlobalConfig::getInstance().branchAngleDev),
                GlobalConfig::getInstance().normalSegLen, cur->getT() + 1 + GlobalConfig::getInstance().normalDelay,
                NormalRoad));
        }
    }
    for (RoadSeg seg : res)
    {
        seg.addStNeighbor(cur);
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
}

// bool IntersectionAction::apply(RoadSeg *cur)
// {
//     if (angleDiff(this->seg->getDir(), cur->getDir()) < GlobalConfig::getInstance().minIntersectionDev)
//     {
//         return false;
//     }
// }
