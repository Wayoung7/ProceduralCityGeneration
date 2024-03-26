#include "Math.h"

bool segInCircle(Vec2 st, Vec2 ed, Vec2 center, float radius)
{
    if (distanceFromSegSquared(center, st, ed) < radius * radius)
    {
        return true;
    }
    else
    {
        return false;
    }
}

float distanceFromSegSquared(Vec2 center, Vec2 st, Vec2 ed)
{
    Vec2 v = ed - st;
    Vec2 w = center - st;
    float dot = dotProduct(v, w);
    float lenSquared = v.lenSquared();
    float t = dot / lenSquared;
    t = std::max(0.f, std::min(1.f, t));
    return center.distanceSquared(st + v * t);
}

float dotProduct(Vec2 a, Vec2 b)
{
    return a.x * b.x + a.y * b.y;
}

bool isPointInSegRange(Vec2 p, Vec2 st, Vec2 ed)
{
    Vec2 v = ed - st;
    float dot = dotProduct(p - st, v);
    return dot >= 0 && dot <= v.lenSquared();
}

float distanceFromLineSquared(Vec2 center, Vec2 st, Vec2 ed)
{
    Vec2 v = ed - st;
    Vec2 w = center - st;
    float dot = dotProduct(v, w);
    float lenSquared = v.lenSquared();
    float t = dot / lenSquared;
    return center.distanceSquared(st + v * t);
}

Vec2 closePointToLine(Vec2 p, Vec2 st, Vec2 ed)
{
    Vec2 v = ed - st;
    Vec2 w = p - st;
    float dot = dotProduct(v, w);
    float lenSquared = v.lenSquared();
    float t = dot / lenSquared;
    return st + v * t;
}

float angleDiff(float a, float b)
{
    float larger = std::max(a, b);
    float smaller = std::min(a, b);
    return std::min(dirDiff(a, b), dirDiff(larger - M_PI, smaller));
}

float dirDiff(float a, float b)
{
    return std::abs(b - a);
}
