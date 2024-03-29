#include "Math.h"

bool segInCircle(const Vec2 &st, const Vec2 &ed, const Vec2 &center, float radius)
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

float distanceFromSegSquared(const Vec2 &center, const Vec2 &st, const Vec2 &ed)
{
    Vec2 v = ed - st;
    Vec2 w = center - st;
    float dot = dotProduct(v, w);
    float lenSquared = v.lenSquared();
    float t = dot / lenSquared;
    t = std::max(0.f, std::min(1.f, t));
    return center.distanceSquared(st + v * t);
}

float dotProduct(const Vec2 &a, const Vec2 &b)
{
    return a.x * b.x + a.y * b.y;
}

bool isPointInSegRange(const Vec2 &p, const Vec2 &st, const Vec2 &ed)
{
    Vec2 v = ed - st;
    float dot = dotProduct(p - st, v);
    return dot >= 0 && dot <= v.lenSquared();
}

float distanceFromLineSquared(const Vec2 &center, const Vec2 &st, const Vec2 &ed)
{
    Vec2 v = ed - st;
    Vec2 w = center - st;
    float dot = dotProduct(v, w);
    float lenSquared = v.lenSquared();
    float t = dot / lenSquared;
    return center.distanceSquared(st + v * t);
}

Vec2 closePointToLine(const Vec2 &p, const Vec2 &st, const Vec2 &ed)
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
