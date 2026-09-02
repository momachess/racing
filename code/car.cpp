#include "racing.h"

#define display_track (*environment->track)
#define display_car (environment->car)
#define car_parameters (*environment->parameters)
#define controller_config (*environment->controller_parameters)

void car_parameters_initialize_default(CarParameters *parameters)
{
    memset(parameters, 0, sizeof(*parameters));
    parameters->max_speed_kmh = 330.0f;
    parameters->wheelbase = 3.6f;
    parameters->max_steering_angle = 0.35f;
    parameters->max_steering_rate = 0.9f;
    parameters->max_yaw_rate = 2.5f;
    parameters->mass_kg = 1500.0f;
    parameters->max_engine_force_n = 18000.0f;
    parameters->max_engine_power_w = 598125.0f;
    parameters->engine_power_speed_floor = 1.0f;
    parameters->max_brake_force_n = 18000.0f;
    parameters->rolling_deceleration = 0.35f;
    parameters->aero_deceleration_at_max_speed = 4.0f;
    parameters->mechanical_lateral_acceleration = 18.0f;
    parameters->aerodynamic_lateral_acceleration_at_max_speed = 35.0f;
    parameters->half_width = 0.5f;
    parameters->front_offset = 2.75f;

    parameters->lidar_sensor_count = 7;
    parameters->lidar_angle_step_degrees = 30.0f;
    parameters->lidar_range = 100.0f;

    parameters->lookahead_point_count = 4;
    parameters->lookahead_time[0] = 0.5f;
    parameters->lookahead_time[1] = 1.5f;
    parameters->lookahead_time[2] = 2.5f;
    parameters->lookahead_time[3] = 3.5f;
    parameters->lookahead_minimum_distance[0] = 12.0f;
    parameters->lookahead_minimum_distance[1] = 24.0f;
    parameters->lookahead_minimum_distance[2] = 36.0f;
    parameters->lookahead_minimum_distance[3] = 48.0f;
    parameters->lookahead_curvature_sample_distance = 10.0f;
    parameters->lookahead_curvature_scale = 50.0f;
}

void vehicle_controller_parameters_initialize_default(
    VehicleControllerParameters *parameters)
{
    memset(parameters, 0, sizeof(*parameters));
    parameters->maximum_lateral_offset = 5.5f;
    parameters->maximum_heading_offset =
        20.0f * (float)M_PI / 180.0f;
    parameters->target_filter_time_constant = 0.15f;
    parameters->speed_proportional_gain = 0.8f;
    parameters->speed_integral_gain = 0.10f;
    parameters->speed_integral_limit = 20.0f;
    parameters->lateral_error_gain = 2.0f;
    parameters->lateral_softening_speed = 5.0f;
    parameters->heading_error_gain = 1.0f;
    parameters->track_heading_lookahead_time = 0.25f;
    parameters->track_heading_minimum_lookahead = 5.0f;
}

float car_maximum_lateral_acceleration_at_speed(
    const CarParameters *parameters,
    float speed)
{
    float maximum_speed = parameters->max_speed_kmh / 3.6f;
    float speed_ratio = maximum_speed > 0.0f
        ? fmaxf(0.0f, fminf(speed / maximum_speed, 1.0f))
        : 0.0f;
    return fmaxf(
        parameters->mechanical_lateral_acceleration +
        parameters->aerodynamic_lateral_acceleration_at_max_speed *
            speed_ratio * speed_ratio,
        1e-6f);
}

float car_encode_longitudinal_acceleration_input(
    const CarParameters *parameters,
    float acceleration)
{
    float maximum_forward_acceleration = parameters->mass_kg > 0.0f
        ? parameters->max_engine_force_n / parameters->mass_kg
        : 0.0f;
    float maximum_braking_deceleration = parameters->mass_kg > 0.0f
        ? parameters->max_brake_force_n / parameters->mass_kg +
            parameters->rolling_deceleration +
            parameters->aero_deceleration_at_max_speed
        : 0.0f;
    if (acceleration >= 0.0f)
        return 0.5f + 0.5f * clamp01(
            maximum_forward_acceleration > 0.0f
            ? acceleration / maximum_forward_acceleration
            : 0.0f);
    return 0.5f - 0.5f * clamp01(
        maximum_braking_deceleration > 0.0f
        ? -acceleration / maximum_braking_deceleration
        : 0.0f);
}

float car_lookahead_distance_for_point(
    const RacingEnv *environment,
    int point)
{
    return fmaxf(
        display_car.v_x * car_parameters.lookahead_time[point],
        car_parameters.lookahead_minimum_distance[point]);
}

void car_update_lookahead_points(RacingEnv *environment)
{
    for (int point = 0; point < car_parameters.lookahead_point_count; point++) {
        display_car.lookahead_point[point] = track_position_at_s(
            &display_track,
            display_car.track_s +
                car_lookahead_distance_for_point(environment, point));
    }
}

static float lidar_segment_distance(
    Vec2 origin,
    Vec2 direction,
    BorderSegment segment,
    float lidar_range)
{
    Vec2 edge = vsub(segment.end, segment.start);
    float denominator = cross_2d(direction, edge);
    if (fabsf(denominator) < 1e-6f)
        return lidar_range;

    Vec2 offset = vsub(segment.start, origin);
    float ray_distance = cross_2d(offset, edge) / denominator;
    float segment_amount = cross_2d(offset, direction) / denominator;
    if (ray_distance >= 0.0f && ray_distance <= lidar_range &&
        segment_amount >= 0.0f && segment_amount <= 1.0f)
    {
        return ray_distance;
    }
    return lidar_range;
}

void car_update_lidar_sensors(RacingEnv *environment)
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
        float nearest = car_parameters.lidar_range;

        for (int i = 0; i < display_track.left_boundary_count; i++)
            nearest = fminf(nearest, lidar_segment_distance(
                origin, direction, display_track.left_boundary[i],
                car_parameters.lidar_range));
        for (int i = 0; i < display_track.right_boundary_count; i++)
            nearest = fminf(nearest, lidar_segment_distance(
                origin, direction, display_track.right_boundary[i],
                car_parameters.lidar_range));

        display_car.lidar_distance[sensor] = nearest;
    }
}

static int car_center_is_inside_track(const Track *track, Vec2 point)
{
    if (track->boundary_count < 3)
        return 1;

    int inside = 0;
    for (int i = 0, j = track->boundary_count - 1;
         i < track->boundary_count;
         j = i++)
    {
        Vec2 start = track->boundary_polygon[j];
        Vec2 end = track->boundary_polygon[i];
        if (point_is_on_segment(point, start, end))
            return 1;

        int crosses_y = (start.y > point.y) != (end.y > point.y);
        if (crosses_y) {
            float intersection_x = start.x +
                (point.y - start.y) * (end.x - start.x) /
                (end.y - start.y);
            if (point.x < intersection_x)
                inside = !inside;
        }
    }
    return inside;
}

int car_has_left_track(const RacingEnv *environment)
{
    const CarState *car = &environment->car;
    Vec2 center = track_position_at_s(&display_track, car->track_s);
    float center_distance = vlength(vsub(car->position, center));
    float maximum_center_distance =
        TRACK_HALF_WIDTH - car_parameters.half_width;

    /* The boundary polygon has artificial end caps around the closing
       centerline segment, so use the exact width test near start/finish. */
    int last_point = display_track.point_count - 1;
    if (last_point >= 0 && car->track_s >= display_track.s[last_point])
        return center_distance > maximum_center_distance;

    return !car_center_is_inside_track(&display_track, car->position) ||
        center_distance > maximum_center_distance;
}

static int timing_line_geometry(
    RacingEnv *environment,
    int timing_line,
    Vec2 *line_start,
    Vec2 *line_end,
    Vec2 *forward)
{
    float line_s = 0.0f;
    if (timing_line == TIMING_LINE_START_FINISH) {
        if (!display_track.has_finish_line)
            return 0;
        *line_start = display_track.finish_line_start;
        *line_end = display_track.finish_line_end;
    } else {
        int sector = timing_line - TIMING_LINE_SECTOR1;
        if (!display_track.has_sectors || sector < 0 ||
            sector >= TRACK_SECTOR_COUNT)
        {
            return 0;
        }
        *line_start = display_track.sector_line_start[sector];
        *line_end = display_track.sector_line_end[sector];
        line_s = display_track.sector_s[sector];
    }

    float heading = track_heading_at_s(&display_track, line_s);
    *forward = vec2(cosf(heading), sinf(heading));
    return 1;
}

static int swept_front_crosses_timing_line(
    RacingEnv *environment,
    Vec2 previous_front,
    Vec2 current_front,
    int timing_line,
    float *step_fraction)
{
    Vec2 line_start;
    Vec2 line_end;
    Vec2 forward;
    if (!timing_line_geometry(
            environment,
            timing_line, &line_start, &line_end, &forward))
    {
        return 0;
    }

    Vec2 movement = vsub(current_front, previous_front);
    if (vdot(movement, forward) <= 0.0f)
        return 0;

    Vec2 timing_segment = vsub(line_end, line_start);
    float denominator = cross_2d(movement, timing_segment);
    if (fabsf(denominator) < 1e-6f)
        return 0;

    Vec2 offset = vsub(line_start, previous_front);
    float movement_amount =
        cross_2d(offset, timing_segment) / denominator;
    float line_amount = cross_2d(offset, movement) / denominator;
    if (movement_amount < 0.0f || movement_amount > 1.0f ||
        line_amount < 0.0f || line_amount > 1.0f)
    {
        return 0;
    }

    *step_fraction = movement_amount;
    return 1;
}

void car_update_timing(
    RacingEnv *environment,
    Vec2 previous_front,
    Vec2 current_front,
    float elapsed,
    float elapsed_before_step)
{
    display_car.circuit_completed_this_step = 0;
    display_car.completed_circuit_time = -1.0f;
    display_car.lap_elapsed = elapsed_before_step + elapsed;

    float crossing_fraction = 0.0f;
    int crossed_line = display_car.timing_next_line;
    if (!swept_front_crosses_timing_line(
            environment,
            previous_front,
            current_front,
            crossed_line,
            &crossing_fraction))
    {
        return;
    }

    float crossing_time = elapsed_before_step +
        crossing_fraction * elapsed;
    if (display_car.timing_start_line == TIMING_LINE_START_FINISH &&
        crossed_line >= TIMING_LINE_SECTOR1)
    {
        int sector = crossed_line - TIMING_LINE_SECTOR1;
        display_car.split_time[sector] = crossing_time;
    }

    display_car.timing_crossing_count++;
    if (crossed_line == display_car.timing_start_line &&
        display_car.timing_crossing_count == TIMING_LINE_COUNT)
    {
        display_car.circuit_completed_this_step = 1;
        display_car.completed_circuit_time = crossing_time;
        display_car.last_lap = crossing_time;
        display_car.lap_count++;
        display_car.lap_elapsed =
            (1.0f - crossing_fraction) * elapsed;
        display_car.timing_crossing_count = 0;
        display_car.timing_next_line =
            (display_car.timing_start_line + 1) % TIMING_LINE_COUNT;
        for (int sector = 0; sector < TRACK_SECTOR_COUNT; sector++)
            display_car.split_time[sector] = -1.0f;
        return;
    }

    display_car.timing_next_line =
        (crossed_line + 1) % TIMING_LINE_COUNT;
}

void car_history_sample_from_state(
    const CarState *car,
    CarHistorySample *sample)
{
    sample->v_x = car->v_x;
    sample->v_y = car->v_y;
    sample->a_x = car->a_x;
    sample->a_y = car->a_y;
    sample->steering_angle = car->steering_angle;
    sample->throttle = car->throttle;
    sample->brake = car->brake;
}

static void car_push_history(
    RacingEnv *environment,
    const CarHistorySample *sample)
{
    int last = environment->history_count;
    if (last >= CAR_HISTORY_STEP_COUNT)
        last = CAR_HISTORY_STEP_COUNT - 1;
    for (int step = last; step > 0; step--)
        environment->history[step] = environment->history[step - 1];

    environment->history[0] = *sample;
    if (environment->history_count < CAR_HISTORY_STEP_COUNT)
        environment->history_count++;
}

void car_update_history(
    RacingEnv *environment,
    const CarHistorySample *previous,
    float elapsed)
{
    if (elapsed <= 0.0f || CAR_HISTORY_SAMPLE_INTERVAL <= 0.0f)
        return;

    CarHistorySample current;
    car_history_sample_from_state(&display_car, &current);
    float first_sample_time =
        CAR_HISTORY_SAMPLE_INTERVAL -
        environment->history_sample_elapsed;
    for (float sample_time = first_sample_time;
         sample_time <= elapsed + 1e-6f;
         sample_time += CAR_HISTORY_SAMPLE_INTERVAL)
    {
        float fraction = clamp01(sample_time / elapsed);
        CarHistorySample sample;
        sample.v_x = previous->v_x +
            (current.v_x - previous->v_x) * fraction;
        sample.v_y = previous->v_y +
            (current.v_y - previous->v_y) * fraction;
        sample.a_x = previous->a_x +
            (current.a_x - previous->a_x) * fraction;
        sample.a_y = previous->a_y +
            (current.a_y - previous->a_y) * fraction;
        sample.steering_angle = previous->steering_angle +
            (current.steering_angle - previous->steering_angle) * fraction;
        sample.throttle = previous->throttle +
            (current.throttle - previous->throttle) * fraction;
        sample.brake = previous->brake +
            (current.brake - previous->brake) * fraction;
        if (environment->pending_history_sample_valid) {
            car_push_history(
                environment,
                &environment->pending_history_sample);
        }
        environment->pending_history_sample = sample;
        environment->pending_history_sample_valid = 1;
    }

    environment->history_sample_elapsed = fmodf(
        environment->history_sample_elapsed + elapsed,
        CAR_HISTORY_SAMPLE_INTERVAL);
}

void car_controller_update(
    RacingEnv *environment,
    const RacingAction *action,
    float elapsed)
{
    if (!environment->controller_target_initialized) {
        environment->filtered_desired_speed = action->desired_speed;
        environment->filtered_desired_lateral_offset =
            action->desired_lateral_offset;
        environment->filtered_desired_heading_offset =
            action->desired_heading_offset;
        environment->controller_target_initialized = 1;
    } else {
        float time_constant = fmaxf(
            controller_config.target_filter_time_constant, 0.0f);
        float alpha = time_constant > 0.0f
            ? elapsed / (time_constant + elapsed)
            : 1.0f;
        environment->filtered_desired_speed += alpha *
            (action->desired_speed - environment->filtered_desired_speed);
        environment->filtered_desired_lateral_offset += alpha *
            (action->desired_lateral_offset -
             environment->filtered_desired_lateral_offset);
        environment->filtered_desired_heading_offset += alpha *
            (action->desired_heading_offset -
             environment->filtered_desired_heading_offset);
    }

    display_car.desired_speed = environment->filtered_desired_speed;
    display_car.desired_lateral_offset =
        environment->filtered_desired_lateral_offset;
    display_car.desired_heading_offset =
        environment->filtered_desired_heading_offset;

    float speed_error = display_car.desired_speed - display_car.v_x;
    environment->speed_error_integral += speed_error * elapsed;
    environment->speed_error_integral = fmaxf(
        -controller_config.speed_integral_limit,
        fminf(environment->speed_error_integral,
              controller_config.speed_integral_limit));
    float desired_acceleration =
        controller_config.speed_proportional_gain * speed_error +
        controller_config.speed_integral_gain *
            environment->speed_error_integral;

    float maximum_speed = car_parameters.max_speed_kmh / 3.6f;
    float speed_ratio = maximum_speed > 0.0f
        ? display_car.v_x / maximum_speed
        : 0.0f;
    float resistance_acceleration =
        car_parameters.aero_deceleration_at_max_speed *
            speed_ratio * speed_ratio;
    if (display_car.v_x > 0.0f)
        resistance_acceleration += car_parameters.rolling_deceleration;
    float actuator_acceleration =
        desired_acceleration + resistance_acceleration;
    if (actuator_acceleration >= 0.0f) {
        float power_limited_force = car_parameters.max_engine_power_w /
            fmaxf(display_car.v_x,
                  car_parameters.engine_power_speed_floor);
        float available_engine_force = fminf(
            car_parameters.max_engine_force_n,
            power_limited_force);
        float available_engine_acceleration =
            available_engine_force / car_parameters.mass_kg;
        display_car.throttle = available_engine_acceleration > 0.0f
            ? fmaxf(0.0f, fminf(
                actuator_acceleration / available_engine_acceleration,
                1.0f))
            : 0.0f;
        display_car.brake = 0.0f;
    } else {
        float maximum_brake_deceleration =
            car_parameters.max_brake_force_n / car_parameters.mass_kg;
        display_car.throttle = 0.0f;
        display_car.brake = maximum_brake_deceleration > 0.0f
            ? fmaxf(0.0f, fminf(
                -actuator_acceleration / maximum_brake_deceleration,
                1.0f))
            : 0.0f;
    }

    Vec2 track_center = track_position_at_s(
        &display_track, display_car.track_s);
    float local_track_heading = track_heading_at_s(
        &display_track, display_car.track_s);
    Vec2 track_left = vec2(
        -sinf(local_track_heading),
        cosf(local_track_heading));
    float current_lateral_offset = vdot(
        vsub(display_car.position, track_center), track_left);
    float lateral_error =
        display_car.desired_lateral_offset - current_lateral_offset;
    float heading_lookahead = fmaxf(
        controller_config.track_heading_minimum_lookahead,
        display_car.v_x *
            controller_config.track_heading_lookahead_time);
    float target_heading = track_heading_at_s(
        &display_track,
        display_car.track_s + heading_lookahead) +
        display_car.desired_heading_offset;
    float heading_error = wrap_angle(
        target_heading - display_car.heading);
    float steering_angle =
        controller_config.heading_error_gain * heading_error +
        atanf(controller_config.lateral_error_gain * lateral_error /
              (display_car.v_x +
               controller_config.lateral_softening_speed));
    display_car.steering_command =
        car_parameters.max_steering_angle > 0.0f
        ? fmaxf(-1.0f, fminf(
            steering_angle / car_parameters.max_steering_angle,
            1.0f))
        : 0.0f;
}

float car_step_dynamics(RacingEnv *environment, float elapsed)
{
    float previous_speed = display_car.v_x;
    Vec2 previous_position = display_car.position;
    float previous_heading = display_car.heading;
    float target_steering_angle = display_car.steering_command *
        car_parameters.max_steering_angle;
    float maximum_speed = car_parameters.max_speed_kmh / 3.6f;
    float peak_lateral_overload_ratio = 0.0f;
    float remaining = elapsed;
    while (remaining > 0.0f) {
        float step = fminf(remaining, PHYSICS_MAX_STEP);
        float steering_change =
            target_steering_angle - display_car.steering_angle;
        float max_change = car_parameters.max_steering_rate * step;
        steering_change = fmaxf(
            -max_change,
            fminf(steering_change, max_change));
        display_car.steering_angle += steering_change;

        float speed_ratio = maximum_speed > 0.0f
            ? display_car.v_x / maximum_speed
            : 0.0f;
        float brake_force =
            display_car.brake * car_parameters.max_brake_force_n;
        float deceleration = brake_force / car_parameters.mass_kg;
        if (display_car.v_x > 0.0f)
            deceleration += car_parameters.rolling_deceleration;
        deceleration += car_parameters.aero_deceleration_at_max_speed *
            speed_ratio * speed_ratio;

        float power_limited_force = car_parameters.max_engine_power_w /
            fmaxf(display_car.v_x,
                  car_parameters.engine_power_speed_floor);
        float engine_force = fminf(
            car_parameters.max_engine_force_n,
            power_limited_force);
        float engine_acceleration = display_car.throttle *
            engine_force / car_parameters.mass_kg;
        float acceleration = engine_acceleration - deceleration;
        display_car.v_x = fmaxf(
            0.0f,
            fminf(display_car.v_x + acceleration * step,
                  maximum_speed));

        float requested_curvature = tanf(display_car.steering_angle) /
            car_parameters.wheelbase;
        float speed_squared = display_car.v_x * display_car.v_x;
        float requested_lateral_acceleration =
            speed_squared * fabsf(requested_curvature);
        float maximum_lateral_acceleration =
            car_maximum_lateral_acceleration_at_speed(
                environment->parameters, display_car.v_x);
        peak_lateral_overload_ratio = fmaxf(
            peak_lateral_overload_ratio,
            fmaxf(requested_lateral_acceleration /
                      maximum_lateral_acceleration - 1.0f,
                  0.0f));
        float maximum_curvature = speed_squared > 1e-6f
            ? maximum_lateral_acceleration / speed_squared
            : fabsf(requested_curvature);
        float effective_curvature = fmaxf(
            -maximum_curvature,
            fminf(requested_curvature, maximum_curvature));
        display_car.a_y = speed_squared * effective_curvature;
        float yaw_rate = display_car.v_x * effective_curvature;
        float midpoint_heading = display_car.heading +
            0.5f * yaw_rate * step;
        display_car.position.x += display_car.v_x *
            cosf(midpoint_heading) * step;
        display_car.position.y += display_car.v_x *
            sinf(midpoint_heading) * step;
        display_car.heading = wrap_angle(
            display_car.heading + yaw_rate * step);
        remaining -= step;
    }

    if (elapsed > 1e-6f) {
        display_car.a_x =
            (display_car.v_x - previous_speed) / elapsed;
        Vec2 velocity = vmul(
            vsub(display_car.position, previous_position),
            1.0f / elapsed);
        Vec2 left = vec2(
            -sinf(display_car.heading),
            cosf(display_car.heading));
        display_car.v_y = vdot(velocity, left);
        display_car.yaw_rate = wrap_angle(
            display_car.heading - previous_heading) / elapsed;
    } else {
        display_car.a_x = 0.0f;
        display_car.a_y = 0.0f;
        display_car.yaw_rate = 0.0f;
    }

    return peak_lateral_overload_ratio;
}

#undef display_track
#undef display_car
#undef car_parameters
#undef controller_config
