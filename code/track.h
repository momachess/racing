#pragma once

#include "geometry.h"

#define MAX_TRACK_POINTS 512
#define MAX_BOUNDARY_POINTS (MAX_TRACK_POINTS * 2)
#define TRACK_HALF_WIDTH 7.0f
#define TRACK_HEADING_HALF_SAMPLE_DISTANCE 5.0f
#define TRACK_CLOSEST_SEGMENT_SEARCH_RADIUS 3
#define TRACK_START_POINT_INDEX 70
#define TRACK_SECTOR1_POINT_INDEX 106
#define TRACK_SECTOR2_POINT_INDEX 33
#define TRACK_TRAINING_SEGMENT_START_POINT_INDEX TRACK_START_POINT_INDEX
#define TRACK_TRAINING_SEGMENT_END_POINT_INDEX TRACK_SECTOR1_POINT_INDEX
#define TRACK_SECTOR_COUNT 2
#define TIMING_LINE_START_FINISH 0
#define TIMING_LINE_SECTOR1 1
#define TIMING_LINE_SECTOR2 2
#define TIMING_LINE_COUNT 3

typedef struct {
    Vec2 points[MAX_TRACK_POINTS];
    BorderSegment left_boundary[MAX_BOUNDARY_POINTS];
    BorderSegment right_boundary[MAX_BOUNDARY_POINTS];
    Vec2 boundary_polygon[MAX_BOUNDARY_POINTS * 2];

    int point_count;
    int left_boundary_count;
    int right_boundary_count;
    int boundary_count;

    Vec2 finish_line_start;
    Vec2 finish_line_end;
    int has_finish_line;

    Vec2 sector_line_start[TRACK_SECTOR_COUNT];
    Vec2 sector_line_end[TRACK_SECTOR_COUNT];
    float sector_s[TRACK_SECTOR_COUNT];
    int has_sectors;

    float s[MAX_TRACK_POINTS];
    float total_length;
} Track;

int load_track_geojson(Track *track, const char *filename);
Vec2 track_position_at_s(const Track *track, float s);
float track_heading_at_s(const Track *track, float s);
float track_curvature_at_s(
    const Track *track,
    float s,
    float sample_distance);
int track_segment_index_at_s(const Track *track, float s);
float track_closest_s_near(
    const Track *track,
    Vec2 position,
    int center_segment,
    int *closest_segment);
int track_s_at_geojson_point(
    const Track *track,
    int geojson_index,
    float *track_s);
int track_geojson_index_at_point(
    const Track *track,
    int track_point_index);
float track_forward_distance(
    const Track *track,
    float start_s,
    float end_s);
