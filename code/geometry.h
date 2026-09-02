#pragma once

typedef struct {
    float x;
    float y;
} Vec2;

typedef struct {
    Vec2 start;
    Vec2 end;
} BorderSegment;

Vec2 vec2(float x, float y);
Vec2 vadd(Vec2 a, Vec2 b);
Vec2 vsub(Vec2 a, Vec2 b);
Vec2 vmul(Vec2 a, float scalar);
float vdot(Vec2 a, Vec2 b);
float vlength(Vec2 a);
Vec2 vnormalize(Vec2 a);
float cross_2d(Vec2 a, Vec2 b);
float wrap_angle(float angle);
float clamp01(float value);
int point_is_on_segment(Vec2 point, Vec2 start, Vec2 end);
