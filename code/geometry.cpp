#include "geometry.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Vec2 vec2(float x, float y)
{
    Vec2 value = {x, y};
    return value;
}

Vec2 vadd(Vec2 a, Vec2 b)
{
    return vec2(a.x + b.x, a.y + b.y);
}

Vec2 vsub(Vec2 a, Vec2 b)
{
    return vec2(a.x - b.x, a.y - b.y);
}

Vec2 vmul(Vec2 a, float scalar)
{
    return vec2(a.x * scalar, a.y * scalar);
}

float vdot(Vec2 a, Vec2 b)
{
    return a.x * b.x + a.y * b.y;
}

float vlength(Vec2 a)
{
    return sqrtf(vdot(a, a));
}

Vec2 vnormalize(Vec2 a)
{
    float length = vlength(a);
    if (length < 1e-8f)
        return vec2(0.0f, 0.0f);
    return vmul(a, 1.0f / length);
}

float cross_2d(Vec2 a, Vec2 b)
{
    return a.x * b.y - a.y * b.x;
}

float wrap_angle(float angle)
{
    while (angle > (float)M_PI)
        angle -= 2.0f * (float)M_PI;
    while (angle < -(float)M_PI)
        angle += 2.0f * (float)M_PI;
    return angle;
}

float clamp01(float value)
{
    return fmaxf(0.0f, fminf(value, 1.0f));
}

int point_is_on_segment(Vec2 point, Vec2 start, Vec2 end)
{
    Vec2 segment = vsub(end, start);
    Vec2 relative = vsub(point, start);
    float segment_length = vlength(segment);
    if (segment_length < 1e-6f)
        return vlength(relative) < 1e-4f;

    float perpendicular_distance = fabsf(cross_2d(segment, relative)) /
        segment_length;
    if (perpendicular_distance > 1e-4f)
        return 0;

    float projection = vdot(relative, segment);
    return projection >= 0.0f &&
        projection <= vdot(segment, segment);
}
