#include "Generator.h"
#include "GlobalConfig.h"
#include "GlobalData.h"
#include "Math.h"
#include <cmath>
#include <iostream>

using namespace std;

void Generator::init(RoadNetwork &rn)
{
    RoadVertex *root = rn.addRoadVertex(
        Vec2(GlobalConfig::getInstance().windowWidth / 2.f, GlobalConfig::getInstance().windowHeight / 2.f));
    rn.addRoadVertex(
        Vec2(GlobalConfig::getInstance().windowWidth / 2.f + 10.f, GlobalConfig::getInstance().windowHeight / 2.f),
        root);
    queue.push(*root);
}

void Generator::step(RoadNetwork &rn)
{
    std::cout << queue.size() << std::endl;
    if (queue.empty())
    {
        GlobalData::getInstance().isRunning = false;
        return;
    }
    RoadVertex cur = queue.front();
    queue.pop();
    bool accepted = false;

    // Local constrains
    RoadVertex *parent = cur.getNeighbors()[0];
    // If vertex is out of window, skip current step
    if (!GlobalConfig::getInstance().inside(cur.getPos()))
    {
        return;
    }

    // The parent vertex has too many neighbors
    if (parent->neighborCount() > 4)
    {
        return;
    }

    // bool isTooSmallAngle = false;
    // for (RoadVertex *v : parent->getNeighbors())
    // {
    //     if (std::abs(cur.getPos().angleFrom(parent->getPos()) - v->getPos().angleFrom(parent->getPos())) < 0.3)
    //     {
    //         isTooSmallAngle = true;
    //         break;
    //     }
    // }
    // if (isTooSmallAngle)
    // {
    //     return;
    // }

    // If vertex is close to an existing vertex, snap current vertex to it
    RoadVertex *newParent = nullptr;
    RoadVertex *near = rn.nearestVertex(cur.getPos());
    if (near && cur.getPos().distance(near->getPos()) < 8.f)
    {
        rn.addEdge(cur.getNeighbors()[0], near);
        newParent = near;
        accepted = true;
    }
    else
    {
        newParent = rn.addRoadVertex(cur.getPos(), cur.getNeighbors()[0]);
        accepted = true;
    }

    // Global goals

    if (accepted)
    {
        Vec2 parentPos = cur.getNeighbors()[0]->getPos();
        Vec2 curPos = cur.getPos();
        float minLen = GlobalConfig::getInstance().minSegLen;
        float maxLen = GlobalConfig::getInstance().maxSegLen;
        float angle = curPos.angleFrom(parentPos);
        queue.push(RoadVertex(newParent->getPos() + Vec2::randFromRange(angle - 0.3f, angle + 0.3f) *
                                                        RNG::randFloatFromRange(minLen, maxLen),
                              newParent));
        queue.push(RoadVertex(newParent->getPos() + Vec2::randFromRange(angle + 1.5f - 0.3f, angle + 1.5f + 0.3f) *
                                                        RNG::randFloatFromRange(minLen, maxLen),
                              newParent));
        queue.push(RoadVertex(newParent->getPos() + Vec2::randFromRange(angle - 1.5f - 0.3f, angle - 1.5f + 0.3f) *
                                                        RNG::randFloatFromRange(minLen, maxLen),
                              newParent));
    }
}
