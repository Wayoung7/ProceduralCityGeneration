#pragma once

#include <cmath>
#include <random>

const float EPS = 0.0000001;

class Vec2;

bool segInCircle(Vec2 st, Vec2 ed, Vec2 center, float radius);
float distanceFromSegSquared(Vec2 center, Vec2 st, Vec2 ed);
float dotProduct(Vec2 a, Vec2 b);
bool isPointInSegRange(Vec2 p, Vec2 st, Vec2 ed);
float distanceFromLineSquared(Vec2 center, Vec2 st, Vec2 ed);
Vec2 closePointToLine(Vec2 p, Vec2 st, Vec2 ed);
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

    inline Vec2 operator*(const Vec2 &rhs) const
    {
        return Vec2(this->x * rhs.x, this->y * rhs.y);
    }

    inline Vec2 operator*(const float &rhs) const
    {
        return Vec2(this->x * rhs, this->y * rhs);
    }

    inline Vec2 operator/(const Vec2 &rhs) const
    {
        return Vec2(this->x / rhs.x, this->y / rhs.y);
    }

    inline Vec2 operator/(const float &rhs) const
    {
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

    static inline Vec2 fromAngle(float angle)
    {
        return Vec2(cosf32(angle), sinf32(angle));
    }

    static inline Vec2 randFromRange(float st, float ed)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(st, ed);
        float angle = dis(gen);
        return fromAngle(angle);
    }
};

class RNG
{
  public:
    static inline float randFloatFromRange(float st, float ed)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(st, ed);
        return dis(gen);
    }
};
