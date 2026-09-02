#include "racing.h"

#define d2d_factory (renderer->d2d_factory)
#define d2d_round_stroke (renderer->d2d_round_stroke)
#define d2d_target (renderer->d2d_target)
#define d2d_track_brush (renderer->d2d_track_brush)
#define d2d_center_brush (renderer->d2d_center_brush)
#define d2d_background_brush (renderer->d2d_background_brush)
#define d2d_white_brush (renderer->d2d_white_brush)
#define d2d_black_brush (renderer->d2d_black_brush)
#define d2d_car_brush (renderer->d2d_car_brush)
#define d2d_blue_brush (renderer->d2d_blue_brush)
#define d2d_red_brush (renderer->d2d_red_brush)
#define d2d_green_brush (renderer->d2d_green_brush)
#define d2d_amber_brush (renderer->d2d_amber_brush)
#define d2d_tick_brush (renderer->d2d_tick_brush)
#define d2d_gauge_text_brush (renderer->d2d_gauge_text_brush)
#define d2d_command_brush (renderer->d2d_command_brush)
#define d2d_command_hover_brush (renderer->d2d_command_hover_brush)
#define d2d_text_brush (renderer->d2d_text_brush)
#define d2d_pane_background_brush (renderer->d2d_pane_background_brush)
#define d2d_pane_border_brush (renderer->d2d_pane_border_brush)
#define d2d_pane_heading_brush (renderer->d2d_pane_heading_brush)
#define d2d_pane_label_brush (renderer->d2d_pane_label_brush)
#define d2d_pane_value_brush (renderer->d2d_pane_value_brush)
#define dwrite_factory (renderer->dwrite_factory)
#define dwrite_format (renderer->dwrite_format)
#define dwrite_pane_format (renderer->dwrite_pane_format)
#define dwrite_button_format (renderer->dwrite_button_format)
#define dwrite_gauge_format (renderer->dwrite_gauge_format)
#define dwrite_gauge_tick_format (renderer->dwrite_gauge_tick_format)
#define dwrite_speed_format (renderer->dwrite_speed_format)
#define track_zoom (renderer->track_zoom)
#define track_pan_x (renderer->track_pan_x)
#define track_pan_y (renderer->track_pan_y)
#define command_hover (renderer->command_hover)
#define display_track (*environment->track)
#define display_car (environment->car)
#define car_parameters (*environment->parameters)
#define ga_training (training->metrics)
#define training_running (training->running)

void renderer_context_initialize(RendererContext *renderer)
{
    memset(renderer, 0, sizeof(*renderer));
    track_zoom = 1.0f;
}

HRESULT create_d2d_resources(RendererContext *renderer, HWND window)
{
    HRESULT result = S_OK;

    if (!d2d_factory) {
        result = D2D1CreateFactory(
            D2D1_FACTORY_TYPE_SINGLE_THREADED,
            &d2d_factory);
        if (FAILED(result))
            return result;

        D2D1_STROKE_STYLE_PROPERTIES stroke_properties =
            D2D1::StrokeStyleProperties(
                D2D1_CAP_STYLE_ROUND,
                D2D1_CAP_STYLE_ROUND,
                D2D1_CAP_STYLE_ROUND,
                D2D1_LINE_JOIN_ROUND);
        result = d2d_factory->CreateStrokeStyle(
            stroke_properties,
            NULL,
            0,
            &d2d_round_stroke);
        if (FAILED(result))
            return result;

        result = DWriteCreateFactory(
            DWRITE_FACTORY_TYPE_SHARED,
            __uuidof(IDWriteFactory),
            (IUnknown **)&dwrite_factory);
        if (FAILED(result))
            return result;

        result = dwrite_factory->CreateTextFormat(
            L"Consolas",
            NULL,
            DWRITE_FONT_WEIGHT_NORMAL,
            DWRITE_FONT_STYLE_NORMAL,
            DWRITE_FONT_STRETCH_NORMAL,
            14.0f,
            L"en-US",
            &dwrite_button_format);
        if (FAILED(result))
            return result;

        dwrite_button_format->SetTextAlignment(
            DWRITE_TEXT_ALIGNMENT_CENTER);
        dwrite_button_format->SetParagraphAlignment(
            DWRITE_PARAGRAPH_ALIGNMENT_CENTER);

        result = dwrite_factory->CreateTextFormat(
            L"Consolas",
            NULL,
            DWRITE_FONT_WEIGHT_NORMAL,
            DWRITE_FONT_STYLE_NORMAL,
            DWRITE_FONT_STRETCH_NORMAL,
            14.0f,
            L"en-US",
            &dwrite_pane_format);
        if (FAILED(result))
            return result;

        result = dwrite_factory->CreateTextFormat(
            L"Segoe UI",
            NULL,
            DWRITE_FONT_WEIGHT_NORMAL,
            DWRITE_FONT_STYLE_NORMAL,
            DWRITE_FONT_STRETCH_NORMAL,
            18.0f,
            L"",
            &dwrite_format);
        if (FAILED(result))
            return result;

        result = dwrite_factory->CreateTextFormat(
            L"Segoe UI",
            NULL,
            DWRITE_FONT_WEIGHT_NORMAL,
            DWRITE_FONT_STYLE_NORMAL,
            DWRITE_FONT_STRETCH_NORMAL,
            18.0f,
            L"",
            &dwrite_gauge_format);
        if (FAILED(result))
            return result;

        dwrite_gauge_format->SetTextAlignment(
            DWRITE_TEXT_ALIGNMENT_CENTER);
        dwrite_gauge_format->SetParagraphAlignment(
            DWRITE_PARAGRAPH_ALIGNMENT_CENTER);

        result = dwrite_factory->CreateTextFormat(
            L"Segoe UI",
            NULL,
            DWRITE_FONT_WEIGHT_NORMAL,
            DWRITE_FONT_STYLE_NORMAL,
            DWRITE_FONT_STRETCH_NORMAL,
            13.0f,
            L"",
            &dwrite_gauge_tick_format);
        if (FAILED(result))
            return result;

        dwrite_gauge_tick_format->SetTextAlignment(
            DWRITE_TEXT_ALIGNMENT_CENTER);
        dwrite_gauge_tick_format->SetParagraphAlignment(
            DWRITE_PARAGRAPH_ALIGNMENT_CENTER);

        result = dwrite_factory->CreateTextFormat(
            L"Segoe UI",
            NULL,
            DWRITE_FONT_WEIGHT_NORMAL,
            DWRITE_FONT_STYLE_NORMAL,
            DWRITE_FONT_STRETCH_NORMAL,
            36.0f,
            L"",
            &dwrite_speed_format);
        if (FAILED(result))
            return result;

        dwrite_speed_format->SetTextAlignment(
            DWRITE_TEXT_ALIGNMENT_CENTER);
        dwrite_speed_format->SetParagraphAlignment(
            DWRITE_PARAGRAPH_ALIGNMENT_CENTER);
    }

    if (!d2d_target) {
        RECT client;
        GetClientRect(window, &client);
        result = d2d_factory->CreateHwndRenderTarget(
            D2D1::RenderTargetProperties(),
            D2D1::HwndRenderTargetProperties(
                window,
                D2D1::SizeU(client.right, client.bottom)),
            &d2d_target);
        if (FAILED(result))
            return result;

        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.18f, 0.18f, 0.20f),
            &d2d_track_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.86f, 0.27f, 0.18f),
            &d2d_center_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.05f, 0.06f, 0.08f),
            &d2d_background_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(D2D1::ColorF::White),
            &d2d_white_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(D2D1::ColorF::Black),
            &d2d_black_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.14f, 0.49f, 0.90f),
            &d2d_car_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.14f, 0.49f, 0.90f),
            &d2d_blue_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.92f, 0.12f, 0.12f),
            &d2d_red_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.25f, 0.80f, 0.42f),
            &d2d_green_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.95f, 0.68f, 0.20f),
            &d2d_amber_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.24f, 0.43f, 0.64f),
            &d2d_tick_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.78f, 0.88f, 1.00f),
            &d2d_gauge_text_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.20f, 0.22f, 0.26f),
            &d2d_command_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.28f, 0.42f, 0.65f),
            &d2d_command_hover_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.10f, 0.10f, 0.10f),
            &d2d_text_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.12f, 0.13f, 0.15f, 0.92f),
            &d2d_pane_background_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.30f, 0.32f, 0.35f),
            &d2d_pane_border_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.70f, 0.75f, 0.80f),
            &d2d_pane_heading_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.55f, 0.58f, 0.62f),
            &d2d_pane_label_brush);
        if (FAILED(result))
            return result;
        result = d2d_target->CreateSolidColorBrush(
            D2D1::ColorF(0.90f, 0.92f, 0.95f),
            &d2d_pane_value_brush);
        if (FAILED(result))
            return result;
    }

    return result;
}

void discard_d2d_target(RendererContext *renderer)
{
    release_d2d(&d2d_pane_value_brush);
    release_d2d(&d2d_pane_label_brush);
    release_d2d(&d2d_pane_heading_brush);
    release_d2d(&d2d_pane_border_brush);
    release_d2d(&d2d_pane_background_brush);
    release_d2d(&d2d_text_brush);
    release_d2d(&d2d_command_hover_brush);
    release_d2d(&d2d_command_brush);
    release_d2d(&d2d_tick_brush);
    release_d2d(&d2d_gauge_text_brush);
    release_d2d(&d2d_amber_brush);
    release_d2d(&d2d_green_brush);
    release_d2d(&d2d_red_brush);
    release_d2d(&d2d_blue_brush);
    release_d2d(&d2d_car_brush);
    release_d2d(&d2d_black_brush);
    release_d2d(&d2d_white_brush);
    release_d2d(&d2d_background_brush);
    release_d2d(&d2d_center_brush);
    release_d2d(&d2d_track_brush);
    release_d2d(&d2d_target);
}

TrackRenderView make_track_render_view(
    const RendererContext *renderer,
    const Track *track,
    const RECT *client)
{
    TrackRenderView view;
    float min_y = FLT_MAX;
    float max_x = -FLT_MAX;

    view.map_width = client->right - RIGHT_PANE_WIDTH;
    if (view.map_width < 1)
        view.map_width = 1;

    view.min_x = FLT_MAX;
    view.max_y = -FLT_MAX;
    for (int i = 0; i < track->point_count; i++) {
        view.min_x = fminf(view.min_x, track->points[i].x);
        max_x = fmaxf(max_x, track->points[i].x);
        min_y = fminf(min_y, track->points[i].y);
        view.max_y = fmaxf(view.max_y, track->points[i].y);
    }

    float track_width = fmaxf(max_x - view.min_x, 1e-6f);
    float track_height = fmaxf(view.max_y - min_y, 1e-6f);
    float scale_x = (view.map_width - 80.0f) / track_width;
    float scale_y = (client->bottom - 80.0f) / track_height;

    view.scale = fminf(scale_x, scale_y) * track_zoom;
    view.offset_x = (view.map_width - track_width * view.scale) * 0.5f +
        track_pan_x;
    view.offset_y = (client->bottom - track_height * view.scale) * 0.5f +
        track_pan_y;
    view.world_to_screen = D2D1::Matrix3x2F(
        view.scale,
        0.0f,
        0.0f,
        -view.scale,
        view.offset_x - view.min_x * view.scale,
        view.offset_y + view.max_y * view.scale);

    return view;
}

D2D1_POINT_2F track_screen_point(
    Vec2 point,
    const TrackRenderView *view)
{
    return D2D1::Point2F(
        view->offset_x + (point.x - view->min_x) * view->scale,
        view->offset_y + (view->max_y - point.y) * view->scale);
}

void center_track_render_view_on(
    RendererContext *renderer,
    TrackRenderView *view,
    const RECT *client,
    Vec2 point)
{
    view->offset_x = view->map_width * 0.5f -
        (point.x - view->min_x) * view->scale;
    view->offset_y = client->bottom * 0.5f -
        (view->max_y - point.y) * view->scale;
    view->world_to_screen = D2D1::Matrix3x2F(
        view->scale,
        0.0f,
        0.0f,
        -view->scale,
        view->offset_x - view->min_x * view->scale,
        view->offset_y + view->max_y * view->scale);
}

void draw_track_boundary(
    RendererContext *renderer,
    const BorderSegment *boundary,
    int count,
    float stroke_width)
{
    ID2D1PathGeometry *geometry = NULL;
    ID2D1GeometrySink *sink = NULL;

    if (count < 2)
        return;

    if (FAILED(d2d_factory->CreatePathGeometry(&geometry)) ||
        FAILED(geometry->Open(&sink)))
    {
        release_d2d(&sink);
        release_d2d(&geometry);
        return;
    }

    sink->BeginFigure(
        D2D1::Point2F(boundary[0].start.x, boundary[0].start.y),
        D2D1_FIGURE_BEGIN_HOLLOW);

    for (int i = 0; i < count; i++)
        sink->AddLine(D2D1::Point2F(
            boundary[i].end.x, boundary[i].end.y));

    sink->EndFigure(D2D1_FIGURE_END_OPEN);
    sink->Close();
    release_d2d(&sink);

    d2d_target->DrawGeometry(
        geometry,
        d2d_white_brush,
        stroke_width,
        d2d_round_stroke);
    release_d2d(&geometry);
}

void fill_track_surface(RendererContext *renderer, const Track *track)
{
    float width = TRACK_HALF_WIDTH * 2.0f;

    /* Match _main_.cpp: paint the road as wide centerline segments and
       circular joins. This avoids fill artifacts on a concave loop. */
    for (int i = 0; i < track->point_count; i++) {
        int next = (i + 1) % track->point_count;
        d2d_target->DrawLine(
            D2D1::Point2F(track->points[i].x, track->points[i].y),
            D2D1::Point2F(track->points[next].x, track->points[next].y),
            d2d_track_brush,
            width);
    }

    for (int i = 0; i < track->point_count; i++) {
        d2d_target->FillEllipse(
            D2D1::Ellipse(
                D2D1::Point2F(track->points[i].x, track->points[i].y),
                TRACK_HALF_WIDTH,
                TRACK_HALF_WIDTH),
            d2d_track_brush);
    }
}

void draw_start_finish_line(RendererContext *renderer, const Track *track)
{
    if (!track->has_finish_line)
        return;

    Vec2 line = vsub(track->finish_line_end, track->finish_line_start);
    float length = vlength(line);
    if (length <= 1e-6f)
        return;

    Vec2 direction = vmul(line, 1.0f / length);
    Vec2 normal = vec2(-direction.y, direction.x);
    const int checks = 8;
    float cell = length / (float)checks;
    float band = cell * 1.5f;

    for (int row = 0; row < 2; row++) {
        for (int column = 0; column < checks; column++) {
            float along = ((float)column + 0.5f) * cell;
            float across = ((float)row - 0.5f) * band * 0.5f;
            Vec2 center = vadd(
                track->finish_line_start,
                vadd(vmul(direction, along), vmul(normal, across)));
            float angle = atan2f(direction.y, direction.x) *
                180.0f / (float)M_PI;
            D2D1_MATRIX_3X2_F previous;
            d2d_target->GetTransform(&previous);
            d2d_target->SetTransform(
                D2D1::Matrix3x2F::Rotation(angle) *
                D2D1::Matrix3x2F::Translation(center.x, center.y) *
                previous);
            d2d_target->FillRectangle(
                D2D1::RectF(
                    -cell * 0.5f,
                    -band * 0.25f,
                    cell * 0.5f,
                    band * 0.25f),
                (row + column) % 2 == 0
                ? d2d_black_brush
                : d2d_white_brush);
            d2d_target->SetTransform(previous);
        }
    }
}

void draw_sector_lines(RendererContext *renderer, const Track *track)
{
    if (!track->has_sectors)
        return;

    for (int sector = 0; sector < TRACK_SECTOR_COUNT; sector++) {
        d2d_target->DrawLine(
            D2D1::Point2F(
                track->sector_line_start[sector].x,
                track->sector_line_start[sector].y),
            D2D1::Point2F(
                track->sector_line_end[sector].x,
                track->sector_line_end[sector].y),
            d2d_blue_brush,
            0.6f);
    }
}


void draw_lidar_sensors(RendererContext *renderer, const RacingEnv *environment)
{
    Vec2 forward = vec2(
        cosf(display_car.heading),
        sinf(display_car.heading));
    Vec2 origin = vadd(
        display_car.position,
        vmul(forward, car_parameters.front_offset));

    for (int sensor = 0; sensor < car_parameters.lidar_sensor_count; sensor++) {
        float relative_degrees =
            (sensor - car_parameters.lidar_sensor_count / 2) *
            car_parameters.lidar_angle_step_degrees;
        float angle = display_car.heading +
            relative_degrees * (float)M_PI / 180.0f;
        Vec2 direction = vec2(cosf(angle), sinf(angle));
        Vec2 end = vadd(
            origin,
            vmul(direction, display_car.lidar_distance[sensor]));

        d2d_target->DrawLine(
            D2D1::Point2F(origin.x, origin.y),
            D2D1::Point2F(end.x, end.y),
            d2d_red_brush,
            0.15f);
        d2d_target->FillEllipse(
            D2D1::Ellipse(D2D1::Point2F(end.x, end.y), 0.35f, 0.35f),
            d2d_red_brush);
    }
}


void draw_lookahead_points(RendererContext *renderer, const RacingEnv *environment)
{
    for (int point = 0; point < car_parameters.lookahead_point_count; point++) {
        Vec2 position = display_car.lookahead_point[point];
        float radius = 0.65f + point * 0.18f;
        D2D1_ELLIPSE marker = D2D1::Ellipse(
            D2D1::Point2F(position.x, position.y),
            radius,
            radius);
        d2d_target->FillEllipse(marker, d2d_blue_brush);
        d2d_target->DrawEllipse(marker, d2d_white_brush, 0.18f);
    }
}


void draw_d2d_text(
    RendererContext *renderer,
    const char *text,
    float x,
    float y,
    float width,
    float height)
{
    wchar_t wide_text[512];
    int length = MultiByteToWideChar(
        CP_ACP,
        0,
        text,
        -1,
        wide_text,
        512);

    if (length > 0)
        d2d_target->DrawText(
            wide_text,
            length - 1,
            dwrite_format,
            D2D1::RectF(x, y, x + width, y + height),
            d2d_text_brush);
}

void draw_d2d_right_text(
    RendererContext *renderer,
    const char *text,
    float x,
    float y,
    float width,
    float height)
{
    wchar_t wide_text[512];
    int length = MultiByteToWideChar(
        CP_UTF8, 0, text, -1, wide_text, 512);

    if (length > 0) {
        dwrite_pane_format->SetTextAlignment(DWRITE_TEXT_ALIGNMENT_TRAILING);
        d2d_target->DrawText(
            wide_text,
            length - 1,
            dwrite_pane_format,
            D2D1::RectF(x, y, x + width, y + height),
            d2d_pane_value_brush);
        dwrite_pane_format->SetTextAlignment(DWRITE_TEXT_ALIGNMENT_LEADING);
    }
}

int track_geojson_point_near_screen(
    const RendererContext *renderer,
    const Track *track,
    const RECT *client,
    int screen_x,
    int screen_y,
    float maximum_distance)
{
    if (!renderer || !track || !client || track->point_count <= 0)
        return -1;

    TrackRenderView view = make_track_render_view(renderer, track, client);
    float closest_distance_squared = maximum_distance * maximum_distance;
    int closest_point = -1;
    for (int point = 0; point < track->point_count; point++) {
        D2D1_POINT_2F screen = track_screen_point(track->points[point], &view);
        float dx = screen.x - screen_x;
        float dy = screen.y - screen_y;
        float distance_squared = dx * dx + dy * dy;
        if (distance_squared <= closest_distance_squared) {
            closest_distance_squared = distance_squared;
            closest_point = point;
        }
    }

    return closest_point >= 0
        ? track_geojson_index_at_point(track, closest_point)
        : -1;
}

static void draw_track_point_marker(
    RendererContext *renderer,
    const Track *track,
    const TrackRenderView *view,
    int geojson_index,
    ID2D1Brush *fill_brush,
    float radius)
{
    float track_s;
    if (!track_s_at_geojson_point(track, geojson_index, &track_s))
        return;

    D2D1_POINT_2F screen = track_screen_point(
        track_position_at_s(track, track_s), view);
    D2D1_ELLIPSE marker = D2D1::Ellipse(screen, radius, radius);
    d2d_target->FillEllipse(marker, fill_brush);
    d2d_target->DrawEllipse(marker, d2d_white_brush, 2.0f);
}

static void draw_track_segment_selection(
    RendererContext *renderer,
    const Track *track,
    const TrackRenderView *view,
    const TrainingContext *training)
{
    if (training->track_segment_selection_stage != 0) {
        for (int point = 0; point < track->point_count; point++) {
            D2D1_POINT_2F screen = track_screen_point(
                track->points[point], view);
            d2d_target->FillEllipse(
                D2D1::Ellipse(screen, 2.5f, 2.5f),
                d2d_center_brush);
        }
    }

    if (training->use_track_segment ||
        training->track_segment_selection_stage == 2)
    {
        draw_track_point_marker(
            renderer,
            track,
            view,
            training->track_segment_start_geojson_index,
            d2d_green_brush,
            7.0f);
    }
    if (training->use_track_segment) {
        draw_track_point_marker(
            renderer,
            track,
            view,
            training->track_segment_end_geojson_index,
            d2d_red_brush,
            7.0f);
    }
    if (training->track_segment_selection_stage != 0 &&
        training->track_segment_hover_geojson_index >= 0)
    {
        draw_track_point_marker(
            renderer,
            track,
            view,
            training->track_segment_hover_geojson_index,
            d2d_amber_brush,
            9.0f);
    }
}

void draw_d2d_right_text_brush(
    RendererContext *renderer,
    const char *text,
    float x,
    float y,
    float width,
    float height,
    ID2D1Brush *brush)
{
    wchar_t wide_text[512];
    int length = MultiByteToWideChar(
        CP_UTF8, 0, text, -1, wide_text, 512);

    if (length > 0) {
        dwrite_pane_format->SetTextAlignment(DWRITE_TEXT_ALIGNMENT_TRAILING);
        d2d_target->DrawText(
            wide_text,
            length - 1,
            dwrite_pane_format,
            D2D1::RectF(x, y, x + width, y + height),
            brush);
        dwrite_pane_format->SetTextAlignment(DWRITE_TEXT_ALIGNMENT_LEADING);
    }
}

void draw_d2d_pane_text(
    RendererContext *renderer,
    const char *text,
    float x,
    float y,
    float width,
    float height,
    ID2D1Brush *brush)
{
    wchar_t wide_text[512];
    int length = MultiByteToWideChar(
        CP_UTF8, 0, text, -1, wide_text, 512);

    if (length > 0)
        d2d_target->DrawText(
            wide_text,
            length - 1,
            dwrite_pane_format,
            D2D1::RectF(x, y, x + width, y + height),
            brush);
}

typedef enum {
    TRAINING_METRIC_AVERAGE_FITNESS,
    TRAINING_METRIC_AVERAGE_SPEED,
    TRAINING_METRIC_PROGRESS_REWARD,
    TRAINING_METRIC_CONTROL_PENALTY,
    TRAINING_METRIC_OFF_TRACK,
    TRAINING_METRIC_LAP_COMPLETION,
    TRAINING_METRIC_MEDIAN_PROGRESS
} TrainingTrendMetric;

typedef struct {
    float average_fitness_delta;
    float average_speed_delta;
    float progress_reward_delta;
    float control_penalty_delta;
    float off_track_delta;
    float lap_completion_delta;
    float median_progress_delta;
    float score;
    int warming_up;
    int direction;
    const char *phase;
    const char *confidence;
} TrainingTrendAnalysis;

static const TrainingGenerationSample *training_history_sample(
    const GeneticTraining *metrics,
    int chronological_index)
{
    int oldest = metrics->history_next - metrics->history_count;
    while (oldest < 0)
        oldest += TRAINING_TREND_HISTORY_COUNT;
    int index = (oldest + chronological_index) %
        TRAINING_TREND_HISTORY_COUNT;
    return &metrics->history[index];
}

static float training_metric_value(
    const TrainingGenerationSample *sample,
    TrainingTrendMetric metric)
{
    switch (metric) {
    case TRAINING_METRIC_AVERAGE_SPEED:
        return sample->average_speed;
    case TRAINING_METRIC_PROGRESS_REWARD:
        return sample->average_progress_reward;
    case TRAINING_METRIC_CONTROL_PENALTY:
        return sample->average_control_penalty;
    case TRAINING_METRIC_OFF_TRACK:
        return sample->off_track_percentage;
    case TRAINING_METRIC_LAP_COMPLETION:
        return sample->lap_completion_percentage;
    case TRAINING_METRIC_MEDIAN_PROGRESS:
        return sample->median_track_progress;
    case TRAINING_METRIC_AVERAGE_FITNESS:
    default:
        return sample->average_fitness;
    }
}

static float training_metric_average(
    const GeneticTraining *metrics,
    TrainingTrendMetric metric,
    int first,
    int count)
{
    float sum = 0.0f;
    for (int index = 0; index < count; index++)
        sum += training_metric_value(
            training_history_sample(metrics, first + index), metric);
    return count > 0 ? sum / count : 0.0f;
}

static float training_metric_window_delta(
    const GeneticTraining *metrics,
    TrainingTrendMetric metric,
    int window)
{
    int count = metrics->history_count;
    float previous = training_metric_average(
        metrics, metric, count - 2 * window, window);
    float current = training_metric_average(
        metrics, metric, count - window, window);
    return current - previous;
}

static void analyze_training_trend(
    const TrainingContext *training,
    TrainingTrendAnalysis *analysis)
{
    memset(analysis, 0, sizeof(*analysis));
    const GeneticTraining *metrics = &training->metrics;
    int count = metrics->history_count;
    analysis->warming_up = count < 6;
    analysis->confidence = count >= 20
        ? "High"
        : count >= 10 ? "Medium" : "Low";

    if (training->fitness_function == TRAINING_FITNESS_CURRICULUM) {
        if (training->curriculum_performance_blend < 0.25f) {
            analysis->phase = "Track mastery";
        } else if (training->curriculum_performance_blend < 0.75f) {
            analysis->phase = "Balanced transition";
        } else {
            analysis->phase = "Performance tuning";
        }
    } else if (metrics->reported_lap_completion_percentage < 10.0f) {
        analysis->phase = "Track mastery";
    } else if (metrics->reported_off_track_percentage > 30.0f) {
        analysis->phase = "Reliability";
    } else {
        analysis->phase = "Performance tuning";
    }

    if (analysis->warming_up)
        return;

    int window = count / 2;
    if (window > 10)
        window = 10;
    analysis->average_fitness_delta = training_metric_window_delta(
        metrics, TRAINING_METRIC_AVERAGE_FITNESS, window);
    analysis->average_speed_delta = training_metric_window_delta(
        metrics, TRAINING_METRIC_AVERAGE_SPEED, window);
    analysis->progress_reward_delta = training_metric_window_delta(
        metrics, TRAINING_METRIC_PROGRESS_REWARD, window);
    analysis->control_penalty_delta = training_metric_window_delta(
        metrics, TRAINING_METRIC_CONTROL_PENALTY, window);
    analysis->off_track_delta = training_metric_window_delta(
        metrics, TRAINING_METRIC_OFF_TRACK, window);
    analysis->lap_completion_delta = training_metric_window_delta(
        metrics, TRAINING_METRIC_LAP_COMPLETION, window);
    analysis->median_progress_delta = training_metric_window_delta(
        metrics, TRAINING_METRIC_MEDIAN_PROGRESS, window);

    float previous_fitness = training_metric_average(
        metrics,
        TRAINING_METRIC_AVERAGE_FITNESS,
        count - 2 * window,
        window);
    float fitness_scale = fmaxf(fabsf(previous_fitness), 1.0f);
    float fitness_signal = clamp01(
        0.5f + analysis->average_fitness_delta /
            (0.20f * fitness_scale)) * 2.0f - 1.0f;
    float completion_signal = fmaxf(-1.0f, fminf(
        analysis->lap_completion_delta / 10.0f, 1.0f));
    float off_track_signal = fmaxf(-1.0f, fminf(
        -analysis->off_track_delta / 10.0f, 1.0f));
    float speed_signal = fmaxf(-1.0f, fminf(
        analysis->average_speed_delta / 10.0f, 1.0f));
    float control_signal = fmaxf(-1.0f, fminf(
        analysis->control_penalty_delta / 0.05f, 1.0f));

    if (training->fitness_function == TRAINING_FITNESS_CURRICULUM) {
        float progress_signal = fmaxf(-1.0f, fminf(
            analysis->median_progress_delta / 8.0f, 1.0f));
        float mastery_score =
            0.35f * progress_signal +
            0.25f * completion_signal +
            0.25f * off_track_signal +
            0.15f * fitness_signal;
        float performance_score =
            0.25f * fitness_signal +
            0.25f * completion_signal +
            0.20f * off_track_signal +
            0.20f * speed_signal +
            0.10f * control_signal;
        float performance_weight = clamp01(
            (training->curriculum_performance_blend - 0.25f) / 0.50f);
        analysis->score =
            (1.0f - performance_weight) * mastery_score +
            performance_weight * performance_score;
    } else {
        analysis->score =
            0.25f * fitness_signal +
            0.25f * completion_signal +
            0.20f * off_track_signal +
            0.20f * speed_signal +
            0.10f * control_signal;
    }
    analysis->direction = analysis->score > 0.15f
        ? 1
        : analysis->score < -0.15f ? -1 : 0;
}

static void draw_training_row(
    RendererContext *renderer,
    const char *label,
    const char *value,
    float left,
    float right,
    float y,
    ID2D1Brush *value_brush)
{
    draw_d2d_pane_text(
        renderer, label, left, y, right - left, 22.0f,
        d2d_pane_label_brush);
    draw_d2d_right_text_brush(
        renderer, value, left, y, right - left, 24.0f,
        value_brush);
}

void update_run_chart(
    RendererContext *renderer,
    const RacingEnv *environment,
    int animation_running,
    int training_is_running)
{
    int sampling = animation_running && !training_is_running;
    if (!sampling) {
        renderer->run_chart_sampling = 0;
        return;
    }

    ULONGLONG now = GetTickCount64();
    if (!renderer->run_chart_sampling) {
        renderer->run_chart_sample_count = 0;
        renderer->run_chart_next_sample = 0;
        renderer->run_chart_last_sample_tick = now;
        renderer->run_chart_sampling = 1;
    } else if (now - renderer->run_chart_last_sample_tick <
               RUN_CHART_SAMPLE_INTERVAL_MS)
    {
        return;
    } else {
        renderer->run_chart_last_sample_tick = now;
    }

    RunChartSample *sample =
        &renderer->run_chart_samples[renderer->run_chart_next_sample];
    sample->speed_kmh = display_car.v_x * 3.6f;
    sample->acceleration = display_car.a_x;
    renderer->run_chart_next_sample =
        (renderer->run_chart_next_sample + 1) % RUN_CHART_SAMPLE_COUNT;
    if (renderer->run_chart_sample_count < RUN_CHART_SAMPLE_COUNT)
        renderer->run_chart_sample_count++;
}


void draw_run_chart(
    RendererContext *renderer,
    const RacingEnv *environment,
    float left,
    float top,
    float right,
    float bottom)
{
    if (bottom - top < 70.0f)
        return;

    char speed_legend[48];
    char acceleration_legend[48];
    snprintf(speed_legend, sizeof(speed_legend),
             "Speed %.0f km/h", display_car.v_x * 3.6f);
    snprintf(acceleration_legend, sizeof(acceleration_legend),
             "Accel %+.1f m/s2", display_car.a_x);
    draw_d2d_pane_text(renderer, speed_legend,
                       left, top, (right - left) * 0.52f, 20.0f,
                       d2d_blue_brush);
    draw_d2d_pane_text(renderer, acceleration_legend,
                       left + (right - left) * 0.52f, top,
                       (right - left) * 0.48f, 20.0f,
                       d2d_red_brush);

    D2D1_RECT_F plot = D2D1::RectF(left, top + 22.0f, right, bottom);
    d2d_target->FillRectangle(plot, d2d_command_brush);
    for (int grid = 1; grid < 4; grid++) {
        float y = plot.top + (plot.bottom - plot.top) * grid / 4.0f;
        d2d_target->DrawLine(
            D2D1::Point2F(plot.left, y),
            D2D1::Point2F(plot.right, y),
            d2d_pane_border_brush,
            grid == 2 ? 1.0f : 0.5f);
    }
    d2d_target->DrawRectangle(plot, d2d_pane_border_brush, 1.0f);

    int count = renderer->run_chart_sample_count;
    if (count < 2)
        return;

    int oldest = renderer->run_chart_next_sample - count;
    if (oldest < 0)
        oldest += RUN_CHART_SAMPLE_COUNT;
    float speed_scale = fmaxf(car_parameters.max_speed_kmh, 1.0f);
    D2D1_POINT_2F previous_speed = {};
    D2D1_POINT_2F previous_acceleration = {};
    for (int point = 0; point < count; point++) {
        int sample_index = (oldest + point) % RUN_CHART_SAMPLE_COUNT;
        const RunChartSample *sample =
            &renderer->run_chart_samples[sample_index];
        float x = plot.left + (plot.right - plot.left) *
            point / (float)(count - 1);
        float speed_amount = fmaxf(
            0.0f, fminf(sample->speed_kmh / speed_scale, 1.0f));
        float acceleration_amount = fmaxf(
            -1.0f,
            fminf(sample->acceleration /
                      RUN_CHART_ACCELERATION_RANGE,
                  1.0f));
        D2D1_POINT_2F speed_point = D2D1::Point2F(
            x,
            plot.bottom - speed_amount * (plot.bottom - plot.top));
        D2D1_POINT_2F acceleration_point = D2D1::Point2F(
            x,
            (plot.top + plot.bottom) * 0.5f -
                acceleration_amount * (plot.bottom - plot.top) * 0.5f);
        if (point > 0) {
            d2d_target->DrawLine(
                previous_speed, speed_point, d2d_blue_brush, 1.75f);
            d2d_target->DrawLine(
                previous_acceleration,
                acceleration_point,
                d2d_red_brush,
                1.5f);
        }
        previous_speed = speed_point;
        previous_acceleration = acceleration_point;
    }
}

void draw_d2d_button_text(
    RendererContext *renderer,
    const char *text,
    const D2D1_RECT_F *rectangle,
    ID2D1Brush *brush)
{
    wchar_t wide_text[64];
    int length = MultiByteToWideChar(
        CP_ACP, 0, text, -1, wide_text, 64);

    if (length > 0)
        d2d_target->DrawText(
            wide_text,
            length - 1,
            dwrite_button_format,
            *rectangle,
            brush);
}

void draw_d2d_gauge_text(
    RendererContext *renderer,
    const char *text,
    float x,
    float y,
    float width,
    float height)
{
    wchar_t wide_text[128];
    int length = MultiByteToWideChar(
        CP_ACP, 0, text, -1, wide_text, 128);

    if (length > 0)
        d2d_target->DrawText(
            wide_text,
            length - 1,
            dwrite_gauge_format,
            D2D1::RectF(x, y, x + width, y + height),
            d2d_gauge_text_brush);
}

void draw_d2d_speed_text(
    RendererContext *renderer,
    const char *text,
    float x,
    float y,
    float width,
    float height)
{
    wchar_t wide_text[64];
    int length = MultiByteToWideChar(
        CP_ACP, 0, text, -1, wide_text, 64);

    if (length > 0)
        d2d_target->DrawText(
            wide_text,
            length - 1,
            dwrite_speed_format,
            D2D1::RectF(x, y, x + width, y + height),
            d2d_gauge_text_brush);
}

void draw_d2d_gauge_tick_text(
    RendererContext *renderer,
    const char *text,
    float x,
    float y,
    float width,
    float height)
{
    wchar_t wide_text[64];
    int length = MultiByteToWideChar(
        CP_ACP, 0, text, -1, wide_text, 64);

    if (length > 0)
        d2d_target->DrawText(
            wide_text,
            length - 1,
            dwrite_gauge_tick_format,
            D2D1::RectF(x, y, x + width, y + height),
            d2d_gauge_text_brush);
}

void draw_d2d_gauge_arc(
    RendererContext *renderer,
    float center_x,
    float center_y,
    float radius,
    float start_angle,
    float end_angle,
    ID2D1Brush *brush,
    float stroke_width)
{
    const int segments = 64;
    D2D1_POINT_2F previous = D2D1::Point2F(
        center_x + cosf(start_angle) * radius,
        center_y - sinf(start_angle) * radius);

    for (int segment = 1; segment <= segments; segment++) {
        float amount = (float)segment / (float)segments;
        float angle = start_angle +
            (end_angle - start_angle) * amount;
        D2D1_POINT_2F current = D2D1::Point2F(
            center_x + cosf(angle) * radius,
            center_y - sinf(angle) * radius);
        d2d_target->DrawLine(
            previous,
            current,
            brush,
            stroke_width);
        previous = current;
    }
}

int render_direct2d(
    RendererContext *renderer,
    HWND window,
    const RacingEnv *environment,
    const TrainingContext *training,
    int animation_running)
{
    RECT client;
    GetClientRect(window, &client);

    if (FAILED(create_d2d_resources(renderer, window)))
        return 0;

    if (client.right <= 0 || client.bottom <= 0)
        return 1;

    update_run_chart(
        renderer,
        environment,
        animation_running,
        training_running);

    TrackRenderView view = make_track_render_view(
        renderer,
        &display_track,
        &client);
    if (animation_running ||
        (training_running && training->render_car_during_training))
        center_track_render_view_on(
            renderer, &view, &client, display_car.position);
    int split = view.map_width;
    d2d_target->BeginDraw();
    d2d_target->Clear(D2D1::ColorF(0.22f, 0.55f, 0.22f));
    d2d_target->FillRectangle(
        D2D1::RectF((float)split, 0.0f,
                    (float)client.right, (float)client.bottom),
        d2d_pane_background_brush);

    d2d_target->PushAxisAlignedClip(
        D2D1::RectF(0.0f, 0.0f,
                    (float)view.map_width,
                    (float)client.bottom),
        D2D1_ANTIALIAS_MODE_PER_PRIMITIVE);
    d2d_target->SetTransform(view.world_to_screen);

    fill_track_surface(renderer, &display_track);

    draw_track_boundary(
        renderer,
        display_track.left_boundary,
        display_track.left_boundary_count,
        0.4f);
    draw_track_boundary(
        renderer,
        display_track.right_boundary,
        display_track.right_boundary_count,
        0.4f);
    draw_start_finish_line(renderer, &display_track);
    draw_sector_lines(renderer, &display_track);
    if (!training_running) {
        draw_lookahead_points(renderer, environment);
        draw_lidar_sensors(renderer, environment);
    }

    d2d_target->SetTransform(D2D1::Matrix3x2F::Identity());
    draw_track_segment_selection(
        renderer, &display_track, &view, training);
    d2d_target->PopAxisAlignedClip();

    if (!training_running || training->render_car_during_training) {
        D2D1_POINT_2F car_screen = track_screen_point(
            display_car.position, &view);
        float screen_heading =
            -display_car.heading * 180.0f / (float)M_PI;

        D2D1_MATRIX_3X2_F saved;
        d2d_target->GetTransform(&saved);
        d2d_target->SetTransform(
            D2D1::Matrix3x2F::Rotation(screen_heading) *
            D2D1::Matrix3x2F::Translation(car_screen.x, car_screen.y));

        float s = view.scale;

        /* F1 car body: 5.5m long, 1.0m wide body core */
        float body_half_l = 2.75f * s;
        float body_half_w = 0.50f * s;

        /* Front wing: 2.0m wide, 0.4m deep, at nose tip */
        float fw_half_w = 1.00f * s;
        float fw_depth  = 0.40f * s;
        float fw_x      = body_half_l;

        /* Rear wing: 1.0m wide, 0.3m deep, at tail */
        float rw_half_w = 0.50f * s;
        float rw_depth  = 0.30f * s;
        float rw_x      = -body_half_l;

        /* Front wheels: 0.7m long, 0.4m wide, offset 1.0m from center */
        float fwh_half_l = 0.35f * s;
        float fwh_half_w = 0.20f * s;
        float fwh_x      = 1.60f * s;
        float fwh_y      = 0.80f * s;

        /* Rear wheels: 0.8m long, 0.5m wide, offset 1.0m from center */
        float rwh_half_l = 0.40f * s;
        float rwh_half_w = 0.25f * s;
        float rwh_x      = -1.40f * s;
        float rwh_y      = 0.80f * s;

        /* Body — tapered shape */
        ID2D1PathGeometry *body_geom = NULL;
        ID2D1GeometrySink *body_sink = NULL;
        if (SUCCEEDED(d2d_factory->CreatePathGeometry(&body_geom)) &&
            SUCCEEDED(body_geom->Open(&body_sink)))
        {
            body_sink->BeginFigure(
                D2D1::Point2F(body_half_l, 0.0f),
                D2D1_FIGURE_BEGIN_FILLED);
            body_sink->AddLine(D2D1::Point2F(
                body_half_l * 0.6f, -body_half_w));
            body_sink->AddLine(D2D1::Point2F(
                -body_half_l * 0.3f, -body_half_w));
            body_sink->AddLine(D2D1::Point2F(
                -body_half_l, -body_half_w * 0.8f));
            body_sink->AddLine(D2D1::Point2F(
                -body_half_l, body_half_w * 0.8f));
            body_sink->AddLine(D2D1::Point2F(
                -body_half_l * 0.3f, body_half_w));
            body_sink->AddLine(D2D1::Point2F(
                body_half_l * 0.6f, body_half_w));
            body_sink->EndFigure(D2D1_FIGURE_END_CLOSED);
            body_sink->Close();
            d2d_target->FillGeometry(body_geom, d2d_car_brush);
        }
        release_d2d(&body_sink);
        release_d2d(&body_geom);

        /* Front wing */
        d2d_target->FillRectangle(
            D2D1::RectF(fw_x, -fw_half_w,
                        fw_x + fw_depth, fw_half_w),
            d2d_white_brush);

        /* Rear wing */
        d2d_target->FillRectangle(
            D2D1::RectF(rw_x - rw_depth, -rw_half_w,
                        rw_x, rw_half_w),
            d2d_white_brush);

        /* Wheels */
        d2d_target->FillRectangle(
            D2D1::RectF(fwh_x - fwh_half_l, -fwh_y - fwh_half_w,
                        fwh_x + fwh_half_l, -fwh_y + fwh_half_w),
            d2d_black_brush);
        d2d_target->FillRectangle(
            D2D1::RectF(fwh_x - fwh_half_l, fwh_y - fwh_half_w,
                        fwh_x + fwh_half_l, fwh_y + fwh_half_w),
            d2d_black_brush);
        d2d_target->FillRectangle(
            D2D1::RectF(rwh_x - rwh_half_l, -rwh_y - rwh_half_w,
                        rwh_x + rwh_half_l, -rwh_y + rwh_half_w),
            d2d_black_brush);
        d2d_target->FillRectangle(
            D2D1::RectF(rwh_x - rwh_half_l, rwh_y - rwh_half_w,
                        rwh_x + rwh_half_l, rwh_y + rwh_half_w),
            d2d_black_brush);

        d2d_target->SetTransform(saved);
    }

    int gauge_x = 170;
    int gauge_y = client.bottom - 145;
    float speed = display_car.v_x * 3.6f;

    float speed_ratio = fminf(speed / 350.0f, 1.0f);
    float gauge_radius = 112.0f;
    d2d_target->FillEllipse(
        D2D1::Ellipse(D2D1::Point2F((float)gauge_x, (float)gauge_y),
                      gauge_radius + 14.0f, gauge_radius + 14.0f),
        d2d_background_brush);

    float gauge_start = 225.0f * (float)M_PI / 180.0f;
    float gauge_end = -45.0f * (float)M_PI / 180.0f;
    draw_d2d_gauge_arc(renderer, 
        (float)gauge_x,
        (float)gauge_y,
        gauge_radius,
        gauge_start,
        gauge_end,
        d2d_tick_brush,
        12.0f);

    draw_d2d_gauge_arc(renderer, 
        (float)gauge_x,
        (float)gauge_y,
        gauge_radius,
        gauge_start,
        gauge_start - 0.2f * 270.0f * (float)M_PI / 180.0f,
        d2d_blue_brush,
        12.0f);

    d2d_target->DrawEllipse(
        D2D1::Ellipse(D2D1::Point2F((float)gauge_x, (float)gauge_y),
                      gauge_radius - 64.0f, gauge_radius - 64.0f),
        d2d_tick_brush,
        1.5f);

    for (int tick = 0; tick <= 35; tick++) {
        float angle = gauge_start -
            tick * 270.0f * (float)M_PI / (35.0f * 180.0f);
        float inner = tick % 5 == 0 ? gauge_radius - 22.0f :
                                     gauge_radius - 14.0f;
        D2D1_POINT_2F a = D2D1::Point2F(
            gauge_x + cosf(angle) * inner,
            gauge_y - sinf(angle) * inner);
        D2D1_POINT_2F b = D2D1::Point2F(
            gauge_x + cosf(angle) * (gauge_radius - 5.0f),
            gauge_y - sinf(angle) * (gauge_radius - 5.0f));
        d2d_target->DrawLine(
            a, b,
            tick >= 28 ? d2d_blue_brush : d2d_tick_brush,
            tick % 5 == 0 ? 3.0f : 2.0f);
    }

    float needle_angle = gauge_start -
        speed_ratio * 270.0f * (float)M_PI / 180.0f;
    float needle_inner_radius = gauge_radius - 64.0f;
    D2D1_POINT_2F needle_start = D2D1::Point2F(
        gauge_x + cosf(needle_angle) * needle_inner_radius,
        gauge_y - sinf(needle_angle) * needle_inner_radius);
    D2D1_POINT_2F needle_end = D2D1::Point2F(
        gauge_x + cosf(needle_angle) * (gauge_radius - 18.0f),
        gauge_y - sinf(needle_angle) * (gauge_radius - 18.0f));
    d2d_target->DrawLine(
        needle_start,
        needle_end,
        d2d_blue_brush,
        4.0f);

    char speed_text[32];
    snprintf(speed_text, sizeof(speed_text), "%.0f", speed);
    draw_d2d_speed_text(renderer, speed_text, gauge_x - 70.0f, gauge_y - 25.0f,
                        140.0f, 44.0f);
    draw_d2d_gauge_text(renderer, "km/h", gauge_x - 35.0f, gauge_y + 10.0f,
                        70.0f, 22.0f);

    for (int label = 0; label <= 7; label++) {
        float angle = gauge_start -
            label * 270.0f * (float)M_PI / (7.0f * 180.0f);
        float label_radius = gauge_radius - 39.0f;
        char label_text[8];
        snprintf(label_text, sizeof(label_text), "%d", label * 50);
        draw_d2d_gauge_tick_text(renderer,
            label_text,
            gauge_x + cosf(angle) * label_radius - 14.0f,
            gauge_y - sinf(angle) * label_radius - 10.0f,
            28.0f,
            22.0f);
    }

    d2d_target->DrawLine(
        D2D1::Point2F((float)split, 0.0f),
        D2D1::Point2F((float)split, (float)client.bottom),
        d2d_pane_border_brush,
        1.5f);

    const char *pane_title = "RUN MODE";
    if (training_running)
        pane_title = "TRAIN MODE";
    draw_d2d_pane_text(renderer, 
        pane_title,
        split + 40.0f, 40.0f,
        600.0f, 22.0f, d2d_pane_heading_brush);

    char current_lap_text[32];
    char last_lap_text[32];
    char split_time_1_text[32];
    char split_time_2_text[32];
    char lap_count_text[32];
    char speed_value_text[32];
    char throttle_text[32];
    char brake_text[32];
    char steering_command_text[32];
    char desired_speed_text[32];
    char actual_lateral_offset_text[32];
    char desired_lateral_offset_text[32];
    char heading_error_text[32];
    char desired_heading_offset_text[32];
    char upcoming_curvature_text[32];
    char step_reward_text[32];
    char generation_text[32];
    char generation_elapsed_text[32];
    char population_text[32];
    char best_fitness_text[32];
    char average_fitness_text[32];
    char best_average_speed_text[32];
    char best_top_speed_text[32];
    char average_progress_reward_text[32];
    char off_track_text[32];
    char lap_completion_text[32];
    char median_progress_text[32];
    char curriculum_blend_text[32];
    format_lap_time(
        display_car.lap_elapsed,
        current_lap_text,
        sizeof(current_lap_text));
    if (display_car.last_lap >= 0.0f) {
        format_lap_time(
            display_car.last_lap,
            last_lap_text,
            sizeof(last_lap_text));
    } else {
        snprintf(last_lap_text, sizeof(last_lap_text), "--:--.---");
    }
    if (display_car.split_time[0] >= 0.0f) {
        format_lap_time(display_car.split_time[0], split_time_1_text,
                        sizeof(split_time_1_text));
    } else {
        snprintf(split_time_1_text, sizeof(split_time_1_text),
                 "--:--.---");
    }
    if (display_car.split_time[1] >= 0.0f) {
        format_lap_time(display_car.split_time[1], split_time_2_text,
                        sizeof(split_time_2_text));
    } else {
        snprintf(split_time_2_text, sizeof(split_time_2_text),
                 "--:--.---");
    }
    snprintf(lap_count_text, sizeof(lap_count_text),
             "%d", display_car.lap_count);
    snprintf(speed_value_text, sizeof(speed_value_text),
             "%.0f km/h", speed);
    snprintf(throttle_text, sizeof(throttle_text),
             "%.3f", display_car.throttle);
    snprintf(brake_text, sizeof(brake_text),
             "%.3f", display_car.brake);
    snprintf(steering_command_text, sizeof(steering_command_text),
             "%+.3f", display_car.steering_command);
    snprintf(desired_speed_text, sizeof(desired_speed_text),
             "%.1f km/h", display_car.desired_speed * 3.6f);
    RacingLineTelemetry line_telemetry;
    racing_env_line_telemetry(environment, &line_telemetry);
    snprintf(actual_lateral_offset_text,
             sizeof(actual_lateral_offset_text),
             "%+.2f m", line_telemetry.lateral_offset);
    snprintf(desired_lateral_offset_text,
             sizeof(desired_lateral_offset_text),
             "%+.2f m", display_car.desired_lateral_offset);
    snprintf(heading_error_text, sizeof(heading_error_text),
             "%+.1f deg", line_telemetry.heading_error *
                 180.0f / (float)M_PI);
    snprintf(desired_heading_offset_text,
             sizeof(desired_heading_offset_text),
             "%+.1f deg",
             display_car.desired_heading_offset *
                 180.0f / (float)M_PI);
    snprintf(upcoming_curvature_text,
             sizeof(upcoming_curvature_text),
             "%+.4f 1/m", line_telemetry.fixed_curvature[1]);
    snprintf(step_reward_text, sizeof(step_reward_text),
             "%+.5f", environment->last_step_reward);
    snprintf(generation_text, sizeof(generation_text),
             "%d", ga_training.completed_generations);
    unsigned long long generation_elapsed_tenths =
        (unsigned long long)(
            ga_training.reported_generation_elapsed_seconds * 10.0f);
    snprintf(
        generation_elapsed_text,
        sizeof(generation_elapsed_text),
        "%llu:%02llu.%llu",
        generation_elapsed_tenths / 600,
        (generation_elapsed_tenths / 10) % 60,
        generation_elapsed_tenths % 10);
    snprintf(population_text, sizeof(population_text),
             "%d", GA_POPULATION_SIZE);
    if (ga_training.completed_generations > 0) {
        snprintf(best_fitness_text, sizeof(best_fitness_text),
                 "%.3f", ga_training.reported_best_fitness);
        snprintf(average_fitness_text, sizeof(average_fitness_text),
                 "%.3f", ga_training.reported_average_fitness);
        snprintf(best_average_speed_text,
                 sizeof(best_average_speed_text),
                 "%.1f km/h", ga_training.reported_best_average_speed);
        snprintf(best_top_speed_text,
                 sizeof(best_top_speed_text),
                 "%.1f km/h", ga_training.reported_best_top_speed);
        snprintf(average_progress_reward_text,
                 sizeof(average_progress_reward_text),
                 "%.3f", ga_training.reported_average_progress_reward);
        snprintf(off_track_text, sizeof(off_track_text),
                 "%.1f%%", ga_training.reported_off_track_percentage);
        snprintf(lap_completion_text, sizeof(lap_completion_text),
                 "%.1f%%",
                 ga_training.reported_lap_completion_percentage);
        if (training->fitness_function == TRAINING_FITNESS_CURRICULUM) {
            snprintf(median_progress_text, sizeof(median_progress_text),
                     "%.1f%%", ga_training.reported_median_track_progress);
        } else {
            snprintf(median_progress_text,
                     sizeof(median_progress_text), "--");
        }
    } else {
        snprintf(best_fitness_text, sizeof(best_fitness_text), "--");
        snprintf(average_fitness_text, sizeof(average_fitness_text), "--");
        snprintf(best_average_speed_text,
                 sizeof(best_average_speed_text), "--");
        snprintf(best_top_speed_text,
                 sizeof(best_top_speed_text), "--");
        snprintf(average_progress_reward_text,
                 sizeof(average_progress_reward_text), "--");
        snprintf(off_track_text, sizeof(off_track_text), "--");
        snprintf(lap_completion_text, sizeof(lap_completion_text), "--");
        snprintf(median_progress_text, sizeof(median_progress_text), "--");
    }
    snprintf(curriculum_blend_text, sizeof(curriculum_blend_text),
             training->fitness_function == TRAINING_FITNESS_CURRICULUM
                 ? "%.0f%%" : "--",
             training->curriculum_performance_blend * 100.0f);

    float result_left = split + 40.0f;
    float result_right = (float)client.right - 40.0f;
    float result_y = 82.0f;
    TrainingTrendAnalysis trend_analysis;
    analyze_training_trend(training, &trend_analysis);
    ID2D1Brush *trend_brush = trend_analysis.warming_up
        ? d2d_amber_brush
        : trend_analysis.direction > 0
            ? d2d_green_brush
            : trend_analysis.direction < 0
                ? d2d_red_brush
                : d2d_amber_brush;
    if (training_running) {
        const char *trend_text = trend_analysis.warming_up
            ? "... WARMING UP"
            : trend_analysis.direction > 0
                ? "\xE2\x96\xB2 IMPROVING"
                : trend_analysis.direction < 0
                    ? "\xE2\x96\xBC REGRESSING"
                    : "\xE2\x86\x92 STABLE";
        draw_training_row(
            renderer, "Training trend", trend_text,
            result_left, result_right, result_y, trend_brush);
        result_y += 26.0f;
        draw_training_row(
            renderer, "Learning phase", trend_analysis.phase,
            result_left, result_right, result_y, d2d_pane_value_brush);
        result_y += 26.0f;
        draw_training_row(
            renderer, "Confidence", trend_analysis.confidence,
            result_left, result_right, result_y,
            trend_analysis.warming_up
                ? d2d_amber_brush
                : d2d_pane_value_brush);
        result_y += 34.0f;

        draw_training_row(
            renderer, "Generations", generation_text,
            result_left, result_right, result_y, d2d_pane_value_brush);
        result_y += 26.0f;
        draw_training_row(
            renderer, "Generation elapsed", generation_elapsed_text,
            result_left, result_right, result_y, d2d_pane_value_brush);
        result_y += 26.0f;
        draw_training_row(
            renderer, "Best fitness", best_fitness_text,
            result_left, result_right, result_y, d2d_pane_value_brush);
        result_y += 26.0f;

        draw_training_row(
            renderer, "Average fitness", average_fitness_text,
            result_left, result_right, result_y,
            d2d_pane_value_brush);
        result_y += 26.0f;
        draw_training_row(
            renderer,
            training->fitness_function == TRAINING_FITNESS_CURRICULUM
                ? "Median progress"
                : "Mean progress reward",
            training->fitness_function == TRAINING_FITNESS_CURRICULUM
                ? median_progress_text
                : average_progress_reward_text,
            result_left, result_right, result_y,
            d2d_pane_value_brush);
        result_y += 26.0f;
        draw_training_row(
            renderer,
            training->use_track_segment
                ? "Segment completion"
                : "Lap completion",
            lap_completion_text,
            result_left, result_right, result_y,
            d2d_pane_value_brush);
        result_y += 26.0f;
        draw_training_row(
            renderer, "Off-track", off_track_text,
            result_left, result_right, result_y,
            d2d_pane_value_brush);
        result_y += 26.0f;
        draw_training_row(
            renderer, "Average speed", best_average_speed_text,
            result_left, result_right, result_y,
            d2d_pane_value_brush);
        result_y += 26.0f;
        draw_training_row(
            renderer, "Top speed", best_top_speed_text,
            result_left, result_right, result_y, d2d_pane_value_brush);
        result_y += 26.0f;
        draw_training_row(
            renderer, "Performance blend", curriculum_blend_text,
            result_left, result_right, result_y, d2d_pane_value_brush);
        result_y += 26.0f;
    } else {
        const char *run_labels[] = {
            "Speed", "Target speed", "Actual lateral", "Target lateral",
            "Heading error", "Target heading", "Curve at 25 m", "Step reward",
            "Controller throttle", "Controller brake", "Controller steering",
            "Current lap", "Split Time 1", "Split Time 2",
            "Last lap", "Laps" };
        const char *run_values[] = {
            speed_value_text, desired_speed_text,
            actual_lateral_offset_text, desired_lateral_offset_text,
            heading_error_text, desired_heading_offset_text,
            upcoming_curvature_text, step_reward_text,
            throttle_text, brake_text, steering_command_text,
            current_lap_text, split_time_1_text,
            split_time_2_text, last_lap_text, lap_count_text };
        for (int row = 0; row < 16; row++) {
            draw_d2d_pane_text(renderer, run_labels[row], result_left, result_y,
                               result_right - result_left, 22.0f,
                               d2d_pane_label_brush);
            draw_d2d_right_text(renderer, run_values[row], result_left, result_y,
                                result_right - result_left, 26.0f);
            result_y += 32.0f;
        }
    }

    float command_y = (float)(client.bottom - 60);
    float source_y = command_y -
        RIGHT_PANE_TRAIN_SOURCE_GAP -
        RIGHT_PANE_TRAIN_SOURCE_HEIGHT;
    float scope_y = source_y -
        RIGHT_PANE_TRAIN_SOURCE_GAP -
        RIGHT_PANE_TRAIN_SOURCE_HEIGHT;
    float training_car_y = scope_y -
        RIGHT_PANE_TRAIN_SOURCE_GAP -
        RIGHT_PANE_TRAIN_SOURCE_HEIGHT;
    float fitness_y = training_car_y -
        RIGHT_PANE_TRAIN_SOURCE_GAP -
        RIGHT_PANE_FITNESS_BUTTON_HEIGHT;
    if (!training_running) {
        float chart_bottom = fitness_y - 14.0f;
        float chart_top = fmaxf(result_y + 8.0f, chart_bottom - 200.0f);
        draw_run_chart(
            renderer,
            environment,
            result_left,
            chart_top,
            result_right,
            chart_bottom);
    }

    const char *fitness_labels[3] = {
        "Standard",
        "Corner exit",
        "Curriculum"
    };
    float fitness_group_width =
        RIGHT_PANE_FITNESS_BUTTON_WIDTH * 3.0f +
        RIGHT_PANE_FITNESS_BUTTON_GAP * 2.0f;
    float fitness_left = split +
        ((float)RIGHT_PANE_WIDTH - fitness_group_width) * 0.5f;
    for (int fitness = 0; fitness < 3; fitness++) {
        int command = 5 + fitness;
        int disabled = right_pane_command_is_disabled(
            command, training_running, training->workers_active);
        int selected = training->fitness_function ==
            (TrainingFitnessFunction)fitness;
        float x = fitness_left + fitness *
            (RIGHT_PANE_FITNESS_BUTTON_WIDTH +
             RIGHT_PANE_FITNESS_BUTTON_GAP);
        D2D1_RECT_F rectangle = D2D1::RectF(
            x,
            fitness_y,
            x + RIGHT_PANE_FITNESS_BUTTON_WIDTH,
            fitness_y + RIGHT_PANE_FITNESS_BUTTON_HEIGHT);
        d2d_target->FillRectangle(
            rectangle,
            selected
                ? d2d_blue_brush
                : disabled
                    ? d2d_pane_background_brush
                    : command_hover == command + 100
                        ? d2d_command_hover_brush
                        : d2d_command_brush);
        d2d_target->DrawRectangle(
            rectangle,
            selected ? d2d_white_brush : d2d_pane_border_brush,
            selected ? 2.0f : 1.0f);
        draw_d2d_button_text(
            renderer,
            fitness_labels[fitness],
            &rectangle,
            selected
                ? d2d_white_brush
                : disabled
                ? d2d_pane_label_brush
                : d2d_pane_value_brush);
    }

    float source_left = split +
        ((float)RIGHT_PANE_WIDTH - RIGHT_PANE_TRAIN_SOURCE_WIDTH) * 0.5f;
    D2D1_RECT_F source_rect = D2D1::RectF(
        source_left,
        source_y,
        source_left + RIGHT_PANE_TRAIN_SOURCE_WIDTH,
        source_y + RIGHT_PANE_TRAIN_SOURCE_HEIGHT);
    int source_disabled = right_pane_command_is_disabled(
        4, training_running, training->workers_active);
    d2d_target->FillRectangle(
        source_rect,
        source_disabled
            ? d2d_pane_background_brush
            : command_hover == 104
                ? d2d_command_hover_brush
                : d2d_command_brush);
    d2d_target->DrawRectangle(
        source_rect, d2d_pane_border_brush, 1.0f);
    draw_d2d_button_text(
        renderer,
        training->start_from_random_weights
            ? "Seed: New random NN"
            : "Seed: Current NN",
        &source_rect,
        source_disabled
            ? d2d_pane_label_brush
            : d2d_pane_value_brush);

    float scope_left = source_left;
    D2D1_RECT_F scope_rect = D2D1::RectF(
        scope_left,
        scope_y,
        scope_left + RIGHT_PANE_TRAIN_SOURCE_WIDTH,
        scope_y + RIGHT_PANE_TRAIN_SOURCE_HEIGHT);
    int scope_disabled = right_pane_command_is_disabled(
        8, training_running, training->workers_active);
    d2d_target->FillRectangle(
        scope_rect,
        scope_disabled
            ? d2d_pane_background_brush
            : command_hover == 108
                ? d2d_command_hover_brush
                : d2d_command_brush);
    d2d_target->DrawRectangle(
        scope_rect, d2d_pane_border_brush, 1.0f);
    char scope_text[64];
    if (training->track_segment_selection_stage == 1) {
        snprintf(scope_text, sizeof(scope_text), "Pick start point on track");
    } else if (training->track_segment_selection_stage == 2) {
        snprintf(scope_text, sizeof(scope_text), "Pick end point on track");
    } else if (training->use_track_segment) {
        snprintf(
            scope_text,
            sizeof(scope_text),
            "Scope: Points %d-%d",
            training->track_segment_start_geojson_index,
            training->track_segment_end_geojson_index);
    } else {
        snprintf(scope_text, sizeof(scope_text), "Scope: Full lap");
    }
    draw_d2d_button_text(
        renderer,
        scope_text,
        &scope_rect,
        scope_disabled
            ? d2d_pane_label_brush
            : d2d_pane_value_brush);

    float training_car_left = source_left;
    D2D1_RECT_F training_car_rect = D2D1::RectF(
        training_car_left,
        training_car_y,
        training_car_left + RIGHT_PANE_TRAIN_SOURCE_WIDTH,
        training_car_y + RIGHT_PANE_TRAIN_SOURCE_HEIGHT);
    d2d_target->FillRectangle(
        training_car_rect,
        command_hover == 109
            ? d2d_command_hover_brush
            : d2d_command_brush);
    d2d_target->DrawRectangle(
        training_car_rect, d2d_pane_border_brush, 1.0f);
    draw_d2d_button_text(
        renderer,
        training->render_car_during_training
            ? "Training car: On"
            : "Training car: Off",
        &training_car_rect,
        d2d_pane_value_brush);

    const char *labels[4] = {
        training_running ? "Stop" : "Train",
        animation_running ? "Stop" : "Run",
        "Save",
        "Load"
    };
    float command_group_width =
        RIGHT_PANE_BUTTON_WIDTH * 4.0f +
        RIGHT_PANE_BUTTON_GAP * 3.0f;
    float command_left = split +
        ((float)RIGHT_PANE_WIDTH - command_group_width) * 0.5f;
    for (int command = 0; command < 4; command++) {
        int disabled = right_pane_command_is_disabled(
            command, training_running, training->workers_active);
        float x = command_left + command *
            (RIGHT_PANE_BUTTON_WIDTH + RIGHT_PANE_BUTTON_GAP);
        D2D1_RECT_F rect = D2D1::RectF(
            x, command_y,
            x + RIGHT_PANE_BUTTON_WIDTH,
            command_y + RIGHT_PANE_BUTTON_HEIGHT);
        d2d_target->FillRectangle(
            rect,
            disabled ? d2d_pane_background_brush :
            command_hover == command + 100
                ? d2d_command_hover_brush
                : d2d_command_brush);
        d2d_target->DrawRectangle(rect, d2d_pane_border_brush, 1.0f);
        draw_d2d_button_text(
            renderer,
            labels[command],
            &rect,
            disabled ? d2d_pane_label_brush : d2d_pane_value_brush);
    }

    HRESULT result = d2d_target->EndDraw();
    if (result == D2DERR_RECREATE_TARGET)
        discard_d2d_target(renderer);
    return 1;
}



void renderer_context_shutdown(RendererContext *renderer)
{
    discard_d2d_target(renderer);
    release_d2d(&d2d_round_stroke);
    release_d2d(&dwrite_format);
    release_d2d(&dwrite_pane_format);
    release_d2d(&dwrite_button_format);
    release_d2d(&dwrite_gauge_format);
    release_d2d(&dwrite_gauge_tick_format);
    release_d2d(&dwrite_speed_format);
    release_d2d(&dwrite_factory);
    release_d2d(&d2d_factory);
}
