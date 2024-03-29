#pragma once

#include <cmath>

const float EPS = 0.000001;

struct Vec2;

bool segInCircle(const Vec2 &st, const Vec2 &ed, const Vec2 &center, float radius);
float distanceFromSegSquared(const Vec2 &center, const Vec2 &st, const Vec2 &ed);
float dotProduct(const Vec2 &a, const Vec2 &b);
bool isPointInSegRange(const Vec2 &p, const Vec2 &st, const Vec2 &ed);
float distanceFromLineSquared(const Vec2 &center, const Vec2 &st, const Vec2 &ed);
Vec2 closePointToLine(const Vec2 &p, const Vec2 &st, const Vec2 &ed);
float angleDiff(float a, float b);
float dirDiff(float a, float b);

struct Vec2
{
    float x;
    float y;

    inline Vec2()
    {
        this->x = 0.f;
        this->y = 0.f;
    }

    inline Vec2(float _x, float _y)
    {
        this->x = _x;
        this->y = _y;
    }

    inline Vec2 operator+(const Vec2 &rhs) const
    {
        return Vec2(this->x + rhs.x, this->y + rhs.y);
    }

    inline Vec2 operator-(const Vec2 &rhs) const
    {
        return Vec2(this->x - rhs.x, this->y - rhs.y);
    }

    inline Vec2 operator*(float rhs) const
    {
        return Vec2(this->x * rhs, this->y * rhs);
    }

    inline Vec2 operator/(float rhs) const
    {
        if (rhs == 0.f)
            return *this;
        else
            return Vec2(this->x / rhs, this->y / rhs);
    }

    inline float distance(const Vec2 &rhs) const
    {
        return sqrtf32((this->x - rhs.x) * (this->x - rhs.x) + (this->y - rhs.y) * (this->y - rhs.y));
    }

    inline float distanceSquared(const Vec2 &rhs) const
    {
        return (this->x - rhs.x) * (this->x - rhs.x) + (this->y - rhs.y) * (this->y - rhs.y);
    }

    inline float len() const
    {
        return sqrtf32(this->x * this->x + this->y * this->y);
    }

    inline float lenSquared() const
    {
        return this->x * this->x + this->y * this->y;
    }

    inline Vec2 normalize() const
    {
        float len = this->len();
        return Vec2(this->x / len, this->y / len);
    }

    inline float toAngle() const
    {
        return std::atan2(y, x);
    }

    inline float angleFrom(const Vec2 &rhs) const
    {
        Vec2 v = *this - rhs;
        return v.toAngle();
    }

    inline bool isEqualApprox(const Vec2 &other) const
    {
        return std::abs(x - other.x) < EPS && std::abs(y - other.y) < EPS;
    }

    inline Vec2 getNormal() const
    {
        return Vec2(-y, x).normalize();
    }

    static inline Vec2 fromAngle(float angle)
    {
        return Vec2(cosf32(angle), sinf32(angle));
    }
};
