#include "racing.h"

static void initialize_track_borders(Track *track)
{
    Vec2 segment_normals[MAX_TRACK_POINTS];
    Vec2 left_points[MAX_TRACK_POINTS];
    Vec2 right_points[MAX_TRACK_POINTS];
    float signed_area = 0.0f;
    int count = track->point_count;

    track->left_boundary_count = 0;
    track->right_boundary_count = 0;
    track->boundary_count = 0;
    track->has_finish_line = 0;

    if (count < 3)
        return;

    for (int i = 0; i < count; i++) {
        int next = (i + 1) % count;
        Vec2 edge = vsub(track->points[next], track->points[i]);
        float length = vlength(edge);
        if (length < 1e-6f)
            length = 1e-6f;
        segment_normals[i] = vec2(-edge.y / length, edge.x / length);
        signed_area += cross_2d(track->points[i], track->points[next]);
    }

    for (int i = 0; i < count; i++) {
        int previous = (i + count - 1) % count;
        Vec2 bisector = vadd(segment_normals[previous], segment_normals[i]);
        float bisector_length = vlength(bisector);

        if (bisector_length < 1e-6f)
            bisector = segment_normals[i];
        else
            bisector = vmul(bisector, 1.0f / bisector_length);

        float cosine_half = vdot(bisector, segment_normals[previous]);
        if (cosine_half < 0.15f)
            cosine_half = 0.15f;

        float miter = TRACK_HALF_WIDTH / cosine_half;
        left_points[i] = vadd(track->points[i], vmul(bisector, miter));
        right_points[i] = vsub(track->points[i], vmul(bisector, miter));
    }

    Vec2 *left = signed_area >= 0.0f ? left_points : right_points;
    Vec2 *right = signed_area >= 0.0f ? right_points : left_points;

    for (int i = 0; i < count; i++) {
        int next = (i + 1) % count;
        track->left_boundary[i].start = left[i];
        track->left_boundary[i].end = left[next];
        track->right_boundary[i].start = right[i];
        track->right_boundary[i].end = right[next];
    }
    track->left_boundary_count = count;
    track->right_boundary_count = count;

    for (int i = 0; i < count; i++)
        track->boundary_polygon[track->boundary_count++] = left[i];
    for (int i = count - 1; i >= 0; i--)
        track->boundary_polygon[track->boundary_count++] = right[i];

    track->finish_line_start = vadd(
        track->points[0], vmul(segment_normals[0], TRACK_HALF_WIDTH));
    track->finish_line_end = vsub(
        track->points[0], vmul(segment_normals[0], TRACK_HALF_WIDTH));
    track->has_finish_line = 1;
}

static int rotated_track_index(const Track *track, int geojson_index)
{
    int first_geojson_index = TRACK_START_POINT_INDEX < track->point_count
        ? TRACK_START_POINT_INDEX
        : 0;
    return (geojson_index - first_geojson_index +
            track->point_count) % track->point_count;
}

int track_s_at_geojson_point(
    const Track *track,
    int geojson_index,
    float *track_s)
{
    if (!track || !track_s || geojson_index < 0 ||
        geojson_index >= track->point_count)
    {
        return 0;
    }

    *track_s = track->s[rotated_track_index(track, geojson_index)];
    return 1;
}

int track_geojson_index_at_point(
    const Track *track,
    int track_point_index)
{
    if (!track || track_point_index < 0 ||
        track_point_index >= track->point_count)
    {
        return -1;
    }

    int first_geojson_index = TRACK_START_POINT_INDEX < track->point_count
        ? TRACK_START_POINT_INDEX
        : 0;
    return (first_geojson_index + track_point_index) % track->point_count;
}

float track_forward_distance(
    const Track *track,
    float start_s,
    float end_s)
{
    if (!track || track->total_length <= 0.0f)
        return 0.0f;

    float distance = end_s - start_s;
    while (distance < 0.0f)
        distance += track->total_length;
    while (distance >= track->total_length)
        distance -= track->total_length;
    return distance;
}

static void initialize_track_sectors(Track *track)
{
    const int geojson_indices[TRACK_SECTOR_COUNT] = {
        TRACK_SECTOR1_POINT_INDEX,
        TRACK_SECTOR2_POINT_INDEX};

    track->has_sectors = 0;
    if (track->point_count < 3)
        return;

    for (int sector = 0; sector < TRACK_SECTOR_COUNT; sector++) {
        int index = rotated_track_index(track, geojson_indices[sector]);
        int previous = (index + track->point_count - 1) %
            track->point_count;
        int next = (index + 1) % track->point_count;
        Vec2 tangent = vnormalize(vsub(
            track->points[next], track->points[previous]));
        Vec2 normal = vec2(-tangent.y, tangent.x);

        track->sector_s[sector] = track->s[index];
        track->sector_line_start[sector] = vadd(
            track->points[index], vmul(normal, TRACK_HALF_WIDTH));
        track->sector_line_end[sector] = vsub(
            track->points[index], vmul(normal, TRACK_HALF_WIDTH));
    }

    track->has_sectors = 1;
}

int load_track_geojson(Track *track, const char *filename)
{
    FILE *file = fopen(filename, "rb");
    char *text;
    long file_size;
    char *coordinates;
    char *cursor;
    int count = 0;
    float origin_lon = 0.0f;
    float origin_lat = 0.0f;
    float latitude_scale;

    if (!file)
        return 0;

    fseek(file, 0, SEEK_END);
    file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (file_size <= 0) {
        fclose(file);
        return 0;
    }

    text = (char *)malloc((size_t)file_size + 1);
    if (!text) {
        fclose(file);
        return 0;
    }

    if (fread(text, 1, (size_t)file_size, file) !=
        (size_t)file_size)
    {
        free(text);
        fclose(file);
        return 0;
    }

    text[file_size] = '\0';
    fclose(file);

    coordinates = strstr(text, "\"coordinates\"");
    if (!coordinates) {
        free(text);
        return 0;
    }

    coordinates = strchr(coordinates, '[');
    if (!coordinates) {
        free(text);
        return 0;
    }

    cursor = coordinates + 1;
    while (count < MAX_TRACK_POINTS) {
        char *end;
        float longitude;
        float latitude;

        cursor = strchr(cursor, '[');
        if (!cursor)
            break;
        cursor++;

        longitude = strtof(cursor, &end);
        if (end == cursor)
            break;
        cursor = end;

        while (*cursor &&
               (*cursor == ' ' || *cursor == '\n' ||
                *cursor == '\r' || *cursor == '\t' ||
                *cursor == ','))
        {
            cursor++;
        }

        latitude = strtof(cursor, &end);
        if (end == cursor)
            break;
        cursor = end;

        while (*cursor && *cursor != ']')
            cursor++;
        if (*cursor == ']')
            cursor++;

        if (count == 0) {
            origin_lon = longitude;
            origin_lat = latitude;
        }

        track->points[count].x = longitude;
        track->points[count].y = latitude;
        count++;
    }

    if (count < 3) {
        free(text);
        return 0;
    }

    latitude_scale = cosf(origin_lat * (float)M_PI / 180.0f);
    for (int i = 0; i < count; i++) {
        float longitude = track->points[i].x;
        float latitude = track->points[i].y;
        track->points[i].x =
            (longitude - origin_lon) * 111320.0f * latitude_scale;
        track->points[i].y =
            (latitude - origin_lat) * 110540.0f;
    }

    if (count > 3 &&
        vlength(vsub(track->points[count - 1], track->points[0])) < 1e-4f)
    {
        count--;
    }

    if (TRACK_START_POINT_INDEX < count) {
        Vec2 reordered[MAX_TRACK_POINTS];
        for (int i = 0; i < count; i++)
            reordered[i] = track->points[
                (TRACK_START_POINT_INDEX + i) % count];
        for (int i = 0; i < count; i++)
            track->points[i] = reordered[i];
    }

    track->point_count = count;
    initialize_track_borders(track);
    track->s[0] = 0.0f;
    for (int i = 1; i < track->point_count; i++) {
        float distance = vlength(vsub(
            track->points[i],
            track->points[i - 1]));
        track->s[i] = track->s[i - 1] + distance;
    }

    float closing = vlength(vsub(
        track->points[0],
        track->points[track->point_count - 1]));
    track->total_length =
        track->s[track->point_count - 1] + closing;

    initialize_track_sectors(track);
    free(text);
    return 1;
}

static float normalize_track_s(const Track *track, float s)
{
    if (track->total_length <= 0.0f)
        return 0.0f;
    while (s < 0.0f)
        s += track->total_length;
    while (s >= track->total_length)
        s -= track->total_length;
    return s;
}

int track_segment_index_at_s(const Track *track, float s)
{
    if (track->point_count <= 1)
        return 0;

    s = normalize_track_s(track, s);
    int low = 0;
    int high = track->point_count;
    while (low + 1 < high) {
        int middle = low + (high - low) / 2;
        if (track->s[middle] <= s)
            low = middle;
        else
            high = middle;
    }
    return low;
}

Vec2 track_position_at_s(const Track *track, float s)
{
    if (track->point_count <= 0 || track->total_length <= 0.0f)
        return vec2(0.0f, 0.0f);

    s = normalize_track_s(track, s);
    int segment = track_segment_index_at_s(track, s);
    int next = (segment + 1) % track->point_count;
    float start = track->s[segment];
    float end = segment == track->point_count - 1
        ? track->total_length
        : track->s[next];
    float length = end - start;
    float amount = length > 0.0f ? (s - start) / length : 0.0f;
    return vadd(
        track->points[segment],
        vmul(vsub(track->points[next], track->points[segment]), amount));
}

float track_heading_at_s(const Track *track, float s)
{
    Vec2 before = track_position_at_s(
        track, s - TRACK_HEADING_HALF_SAMPLE_DISTANCE);
    Vec2 after = track_position_at_s(
        track, s + TRACK_HEADING_HALF_SAMPLE_DISTANCE);
    Vec2 direction = vsub(after, before);
    return atan2f(direction.y, direction.x);
}

float track_curvature_at_s(
    const Track *track,
    float s,
    float sample_distance)
{
    Vec2 before = track_position_at_s(track, s - sample_distance);
    Vec2 center = track_position_at_s(track, s);
    Vec2 after = track_position_at_s(track, s + sample_distance);
    Vec2 first = vsub(center, before);
    Vec2 second = vsub(after, center);
    float chord_length = vlength(vsub(after, before));
    float denominator = vlength(first) *
        vlength(second) * chord_length;
    float cross = cross_2d(first, second);
    return denominator > 1e-6f
        ? 2.0f * cross / denominator
        : 0.0f;
}

float track_closest_s_near(
    const Track *track,
    Vec2 position,
    int center_segment,
    int *closest_segment)
{
    float best_distance_squared = FLT_MAX;
    float best_s = 0.0f;
    int best_segment = center_segment;

    for (int offset = -TRACK_CLOSEST_SEGMENT_SEARCH_RADIUS;
         offset <= TRACK_CLOSEST_SEGMENT_SEARCH_RADIUS;
         offset++)
    {
        int i = center_segment + offset;
        while (i < 0)
            i += track->point_count;
        while (i >= track->point_count)
            i -= track->point_count;
        int j = (i + 1) % track->point_count;
        Vec2 segment = vsub(track->points[j], track->points[i]);
        float segment_length_squared = vdot(segment, segment);
        float amount = segment_length_squared > 0.0f
            ? vdot(vsub(position, track->points[i]), segment) /
              segment_length_squared
            : 0.0f;
        amount = fmaxf(0.0f, fminf(amount, 1.0f));

        Vec2 nearest = vadd(track->points[i], vmul(segment, amount));
        Vec2 difference = vsub(position, nearest);
        float distance_squared = vdot(difference, difference);
        if (distance_squared < best_distance_squared) {
            best_distance_squared = distance_squared;
            float segment_length = i == track->point_count - 1
                ? track->total_length - track->s[i]
                : track->s[i + 1] - track->s[i];
            best_s = track->s[i] + segment_length * amount;
            best_segment = i;
        }
    }

    if (best_s >= track->total_length)
        best_s -= track->total_length;
    *closest_segment = best_segment;
    return best_s;
}
