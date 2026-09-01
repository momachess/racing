#include "racing.h"

#include <random>
#if defined(RACING_USE_AVX2)
#include <immintrin.h>
#endif

#define display_track (*environment->track)
#define display_car (environment->car)
#define car_parameters (*environment->parameters)
#define reward_config (*environment->reward_parameters)
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

void reward_parameters_initialize_default(RewardParameters *parameters)
{
    memset(parameters, 0, sizeof(*parameters));
    parameters->lap_progress_reward = 10.0f;
    parameters->time_penalty_per_second = 0.02f;
    parameters->pedal_conflict_penalty_per_second = 0.0f;
    parameters->steering_change_penalty = 0.002f;
    parameters->excess_lateral_penalty_per_second = 0.01f;
    parameters->off_track_penalty = 1.0f;
    parameters->stationary_penalty = 1.0f;
    parameters->lap_completion_reward = 5.0f;
    parameters->lap_speed_reward = 15.0f;
    parameters->corner_exit_speed_reward = 0.20f;
    parameters->corner_exit_zone_length = 50.0f;
    parameters->corner_exit_curvature_drop_threshold = 0.001f;
    parameters->corner_exit_minimum_curvature = 0.002f;
    parameters->corner_exit_speed_gain_scale = 20.0f;
    parameters->curriculum_progress_scale = 1.5f;
    parameters->curriculum_incomplete_average_speed_reward = 3.0f;
    parameters->curriculum_longitudinal_jerk_penalty = 0.0025f;
    parameters->curriculum_pedal_change_penalty = 0.01f;
    parameters->curriculum_pedal_conflict_penalty_per_second = 0.05f;
    parameters->stationary_speed_threshold = 1.0f;
    parameters->stationary_timeout = 3.0f;
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

void racing_env_initialize(
    RacingEnv *environment,
    const Track *track,
    const CarParameters *parameters,
    const RewardParameters *reward_parameters_pointer,
    const VehicleControllerParameters *controller_parameters_pointer)
{
    memset(environment, 0, sizeof(*environment));
    environment->track = track;
    environment->parameters = parameters;
    environment->reward_parameters = reward_parameters_pointer;
    environment->controller_parameters = controller_parameters_pointer;
}

Vec2 vec2(float x, float y)
{
    Vec2 v = {x, y};
    return v;
}


Vec2 vadd(Vec2 a, Vec2 b)
{
    return vec2(a.x + b.x,
                a.y + b.y);
}


Vec2 vsub(Vec2 a, Vec2 b)
{
    return vec2(a.x - b.x,
                a.y - b.y);
}


Vec2 vmul(Vec2 a, float s)
{
    return vec2(a.x * s,
                a.y * s);
}


float vdot(Vec2 a, Vec2 b)
{
    return a.x * b.x +
           a.y * b.y;
}


float vlength(Vec2 a)
{
    return sqrtf(vdot(a, a));
}


Vec2 vnormalize(Vec2 a)
{
    float len = vlength(a);

    if (len < 1e-8f)
        return vec2(0.0f, 0.0f);

    return vmul(a, 1.0f / len);
}


float clamp01(float value)
{
    return fmaxf(0.0f, fminf(value, 1.0f));
}


float encode_signed_input(float value, float magnitude)
{
    if (magnitude <= 0.0f)
        return 0.5f;
    return clamp01(0.5f + 0.5f * value / magnitude);
}


float encode_longitudinal_acceleration_input(
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


float network_random_unit(unsigned int *random_state)
{
    *random_state = *random_state * 1664525u + 1013904223u;
    return (float)((*random_state >> 8) & 0x00ffffffu) /
        16777216.0f;
}


float network_random_signed(unsigned int *random_state)
{
    return network_random_unit(random_state) * 2.0f - 1.0f;
}


float network_random_gaussian(unsigned int *random_state)
{
    float u1 = fmaxf(network_random_unit(random_state), 1e-7f);
    float u2 = network_random_unit(random_state);
    return sqrtf(-2.0f * logf(u1)) *
        cosf(2.0f * (float)M_PI * u2);
}


/* Mechanical grip dominates at low speed. Aerodynamic downforce increases
   the available lateral acceleration with the square of vehicle speed. */
static float maximum_lateral_acceleration_at_speed(
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


unsigned int network_random_seed(void)
{
    std::random_device entropy;
    LARGE_INTEGER performance_counter;
    QueryPerformanceCounter(&performance_counter);

    unsigned int seed = entropy();
    seed ^= (unsigned int)performance_counter.LowPart;
    seed ^= (unsigned int)performance_counter.HighPart;
    seed ^= GetCurrentProcessId() * 0x9e3779b9u;
    seed ^= (unsigned int)GetTickCount64();
    return seed;
}


void neural_policy_initialize(
    NeuralPolicy *policy,
    unsigned int *random_state)
{
    memset(policy, 0, sizeof(*policy));

    float limit1 = sqrtf(6.0f /
        (NN_INPUT_COUNT + NN_HIDDEN1_COUNT));
    for (int neuron = 0; neuron < NN_HIDDEN1_COUNT; neuron++)
        for (int input = 0; input < NN_INPUT_COUNT; input++)
            policy->hidden1_weights[neuron][input] =
                network_random_signed(random_state) * limit1;

    float limit2 = sqrtf(6.0f /
        (NN_HIDDEN1_COUNT + NN_HIDDEN2_COUNT));
    for (int neuron = 0; neuron < NN_HIDDEN2_COUNT; neuron++)
        for (int input = 0; input < NN_HIDDEN1_COUNT; input++)
            policy->hidden2_weights[neuron][input] =
                network_random_signed(random_state) * limit2;

    float output_limit = sqrtf(6.0f /
        (NN_HIDDEN2_COUNT + NN_OUTPUT_COUNT));
    for (int output = 0; output < NN_OUTPUT_COUNT; output++)
        for (int input = 0; input < NN_HIDDEN2_COUNT; input++)
            policy->output_weights[output][input] =
                network_random_signed(random_state) * output_limit;
}


void network_to_genome(
    const NeuralPolicy *network,
    float genes[NN_GENOME_COUNT])
{
    int gene = 0;
    for (int neuron = 0; neuron < NN_HIDDEN1_COUNT; neuron++)
        for (int input = 0; input < NN_INPUT_COUNT; input++)
            genes[gene++] = network->hidden1_weights[neuron][input];
    for (int neuron = 0; neuron < NN_HIDDEN1_COUNT; neuron++)
        genes[gene++] = network->hidden1_bias[neuron];
    for (int neuron = 0; neuron < NN_HIDDEN2_COUNT; neuron++)
        for (int input = 0; input < NN_HIDDEN1_COUNT; input++)
            genes[gene++] = network->hidden2_weights[neuron][input];
    for (int neuron = 0; neuron < NN_HIDDEN2_COUNT; neuron++)
        genes[gene++] = network->hidden2_bias[neuron];
    for (int output = 0; output < NN_OUTPUT_COUNT; output++)
        for (int input = 0; input < NN_HIDDEN2_COUNT; input++)
            genes[gene++] = network->output_weights[output][input];
    for (int output = 0; output < NN_OUTPUT_COUNT; output++)
        genes[gene++] = network->output_bias[output];
}


void genome_to_network(
    const float genes[NN_GENOME_COUNT],
    NeuralPolicy *network)
{
    int gene = 0;
    for (int neuron = 0; neuron < NN_HIDDEN1_COUNT; neuron++)
        for (int input = 0; input < NN_INPUT_COUNT; input++)
            network->hidden1_weights[neuron][input] = genes[gene++];
    for (int neuron = 0; neuron < NN_HIDDEN1_COUNT; neuron++)
        network->hidden1_bias[neuron] = genes[gene++];
    for (int neuron = 0; neuron < NN_HIDDEN2_COUNT; neuron++)
        for (int input = 0; input < NN_HIDDEN1_COUNT; input++)
            network->hidden2_weights[neuron][input] = genes[gene++];
    for (int neuron = 0; neuron < NN_HIDDEN2_COUNT; neuron++)
        network->hidden2_bias[neuron] = genes[gene++];
    for (int output = 0; output < NN_OUTPUT_COUNT; output++)
        for (int input = 0; input < NN_HIDDEN2_COUNT; input++)
            network->output_weights[output][input] = genes[gene++];
    for (int output = 0; output < NN_OUTPUT_COUNT; output++)
        network->output_bias[output] = genes[gene++];
}


Vec2 track_position_at_s(const Track *track, float s);
float track_heading_at_s(const Track *track, float s);
float wrap_angle(float angle);

static const float fixed_curvature_distance[
    FIXED_CURVATURE_INPUT_COUNT] = {10.0f, 25.0f, 50.0f, 100.0f};


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
    float cross = first.x * second.y - first.y * second.x;
    return denominator > 1e-6f
        ? 2.0f * cross / denominator
        : 0.0f;
}


float lookahead_distance_for_point(
    const RacingEnv *environment,
    int point)
{
    return fmaxf(
        display_car.v_x * car_parameters.lookahead_time[point],
        car_parameters.lookahead_minimum_distance[point]);
}


void racing_env_line_telemetry(
    const RacingEnv *environment,
    RacingLineTelemetry *telemetry)
{
    memset(telemetry, 0, sizeof(*telemetry));
    Vec2 track_center = track_position_at_s(
        &display_track, display_car.track_s);
    float local_track_heading = track_heading_at_s(
        &display_track, display_car.track_s);
    Vec2 track_left = vec2(
        -sinf(local_track_heading),
        cosf(local_track_heading));
    telemetry->lateral_offset = vdot(
        vsub(display_car.position, track_center), track_left);
    telemetry->heading_error = wrap_angle(
        display_car.heading - local_track_heading);

    for (int sample = 0; sample < FIXED_CURVATURE_INPUT_COUNT; sample++) {
        telemetry->fixed_curvature[sample] = track_curvature_at_s(
            &display_track,
            display_car.track_s + fixed_curvature_distance[sample],
            car_parameters.lookahead_curvature_sample_distance);
    }
}


void racing_env_observe(
    const RacingEnv *environment,
    RacingObservation *observation)
{
    float maximum_speed = car_parameters.max_speed_kmh / 3.6f;
    observation->normalized_speed =
        clamp01(display_car.v_x / maximum_speed);
    observation->encoded_lateral_speed = encode_signed_input(
        display_car.v_y, NN_LATERAL_SPEED_SCALE_MPS);
    observation->encoded_steering_angle = encode_signed_input(
        display_car.steering_angle, car_parameters.max_steering_angle);
    observation->encoded_yaw_rate = encode_signed_input(
        display_car.yaw_rate, car_parameters.max_yaw_rate);
    observation->encoded_longitudinal_acceleration =
        encode_longitudinal_acceleration_input(
            environment->parameters, display_car.a_x);
    observation->encoded_lateral_acceleration = encode_signed_input(
        display_car.a_y,
        maximum_lateral_acceleration_at_speed(
            environment->parameters, display_car.v_x));

    for (int step = 0; step < CAR_HISTORY_STEP_COUNT; step++) {
        CarHistorySample sample = {0};
        if (step < environment->history_count)
            sample = environment->history[step];
        RacingHistoryObservation *history = &observation->history[step];
        history->normalized_speed = clamp01(sample.v_x / maximum_speed);
        history->encoded_lateral_speed =
            encode_signed_input(sample.v_y, NN_LATERAL_SPEED_SCALE_MPS);
        history->encoded_longitudinal_acceleration =
            encode_longitudinal_acceleration_input(
                environment->parameters, sample.a_x);
        history->encoded_lateral_acceleration = encode_signed_input(
            sample.a_y,
            maximum_lateral_acceleration_at_speed(
                environment->parameters, sample.v_x));
        history->encoded_steering_angle = encode_signed_input(
            sample.steering_angle,
            car_parameters.max_steering_angle);
        history->normalized_throttle = clamp01(sample.throttle);
        history->normalized_brake = clamp01(sample.brake);
    }

    for (int sensor = 0; sensor < MAX_LIDAR_SENSOR_COUNT; sensor++)
        observation->normalized_lidar_distance[sensor] = 1.0f;
    for (int sensor = 0; sensor < car_parameters.lidar_sensor_count; sensor++)
        observation->normalized_lidar_distance[sensor] =
            clamp01(display_car.lidar_distance[sensor] /
                    car_parameters.lidar_range);

    Vec2 forward = vec2(
        cosf(display_car.heading),
        sinf(display_car.heading));
    Vec2 left = vec2(-forward.y, forward.x);
    for (int point = 0; point < MAX_LOOKAHEAD_POINT_COUNT; point++) {
        RacingLookaheadObservation *lookahead =
            &observation->lookahead[point];
        lookahead->encoded_lateral_offset = 0.5f;
        lookahead->encoded_forward_offset = 0.5f;
        lookahead->encoded_angle = 0.5f;
        lookahead->normalized_curvature = 0.5f;
    }
    for (int point = 0; point < car_parameters.lookahead_point_count; point++) {
        float distance = lookahead_distance_for_point(environment, point);
        Vec2 relative = vsub(
            display_car.lookahead_point[point],
            display_car.position);
        float lateral_offset = vdot(relative, left);
        float forward_offset = vdot(relative, forward);
        float angle_to_point = atan2f(relative.y, relative.x);
        float angle_difference = angle_to_point - display_car.heading;
        while (angle_difference > (float)M_PI)
            angle_difference -= 2.0f * (float)M_PI;
        while (angle_difference < -(float)M_PI)
            angle_difference += 2.0f * (float)M_PI;

        float point_s = display_car.track_s + distance;
        float curvature = track_curvature_at_s(
            &display_track,
            point_s,
            car_parameters.lookahead_curvature_sample_distance);

        RacingLookaheadObservation *lookahead =
            &observation->lookahead[point];
        lookahead->encoded_lateral_offset =
            encode_signed_input(lateral_offset, distance);
        lookahead->encoded_forward_offset =
            encode_signed_input(forward_offset, distance);
        lookahead->encoded_angle =
            encode_signed_input(angle_difference, (float)M_PI);
        lookahead->normalized_curvature =
            clamp01(0.5f + 0.5f * tanhf(
                curvature * car_parameters.lookahead_curvature_scale));
    }

    RacingLineTelemetry telemetry;
    racing_env_line_telemetry(environment, &telemetry);
    float usable_half_width = fmaxf(
        TRACK_HALF_WIDTH - car_parameters.half_width,
        1e-6f);
    observation->encoded_lateral_offset =
        encode_signed_input(telemetry.lateral_offset, usable_half_width);
    observation->encoded_heading_error =
        encode_signed_input(
            telemetry.heading_error,
            NN_HEADING_ERROR_SCALE_RADIANS);

    for (int sample = 0; sample < FIXED_CURVATURE_INPUT_COUNT; sample++) {
        observation->normalized_fixed_curvature[sample] =
            clamp01(0.5f + 0.5f * tanhf(
                telemetry.fixed_curvature[sample] *
                car_parameters.lookahead_curvature_scale));
    }
}


static void observation_to_nn_inputs(
    const RacingObservation *observation,
    float inputs[NN_INPUT_COUNT])
{
    int input = 0;
    inputs[input++] = observation->normalized_speed;
    inputs[input++] = observation->encoded_lateral_speed;
    inputs[input++] = observation->encoded_steering_angle;
    inputs[input++] = observation->encoded_yaw_rate;
    inputs[input++] = observation->encoded_longitudinal_acceleration;
    inputs[input++] = observation->encoded_lateral_acceleration;

    for (int step = 0; step < CAR_HISTORY_STEP_COUNT; step++) {
        const RacingHistoryObservation *history =
            &observation->history[step];
        inputs[input++] = history->normalized_speed;
        inputs[input++] = history->encoded_lateral_speed;
        inputs[input++] = history->encoded_longitudinal_acceleration;
        inputs[input++] = history->encoded_lateral_acceleration;
        inputs[input++] = history->encoded_steering_angle;
        inputs[input++] = history->normalized_throttle;
        inputs[input++] = history->normalized_brake;
    }

    for (int sensor = 0; sensor < MAX_LIDAR_SENSOR_COUNT; sensor++)
        inputs[input++] = observation->normalized_lidar_distance[sensor];

    /* Preserve the saved-network input layout: lookahead values are stored
       by feature rather than by point in the flattened NN vector. */
    for (int point = 0; point < MAX_LOOKAHEAD_POINT_COUNT; point++)
        inputs[input++] =
            observation->lookahead[point].encoded_lateral_offset;
    for (int point = 0; point < MAX_LOOKAHEAD_POINT_COUNT; point++)
        inputs[input++] =
            observation->lookahead[point].encoded_forward_offset;
    for (int point = 0; point < MAX_LOOKAHEAD_POINT_COUNT; point++)
        inputs[input++] = observation->lookahead[point].encoded_angle;
    for (int point = 0; point < MAX_LOOKAHEAD_POINT_COUNT; point++)
        inputs[input++] =
            observation->lookahead[point].normalized_curvature;

    inputs[input++] = observation->encoded_lateral_offset;
    inputs[input++] = observation->encoded_heading_error;
    for (int sample = 0; sample < FIXED_CURVATURE_INPUT_COUNT; sample++)
        inputs[input++] = observation->normalized_fixed_curvature[sample];
}


template <int value_count>
static float neural_dot_product(
    const float *weights,
    const float *values)
{
#if defined(RACING_USE_AVX2)
    __m256 vector_sum = _mm256_setzero_ps();
    int value = 0;
    for (; value + 8 <= value_count; value += 8) {
        __m256 weight_vector = _mm256_loadu_ps(weights + value);
        __m256 value_vector = _mm256_loadu_ps(values + value);
        vector_sum = _mm256_add_ps(
            vector_sum,
            _mm256_mul_ps(weight_vector, value_vector));
    }

    __m128 sum128 = _mm_add_ps(
        _mm256_castps256_ps128(vector_sum),
        _mm256_extractf128_ps(vector_sum, 1));
    sum128 = _mm_hadd_ps(sum128, sum128);
    sum128 = _mm_hadd_ps(sum128, sum128);
    float sum = _mm_cvtss_f32(sum128);

    for (; value < value_count; value++)
        sum += weights[value] * values[value];
    return sum;
#else
    float sum = 0.0f;
    for (int value = 0; value < value_count; value++)
        sum += weights[value] * values[value];
    return sum;
#endif
}


void neural_policy_predict(
    const NeuralPolicy *policy,
    const RacingObservation *observation,
    const CarParameters *car_parameters_pointer,
    const VehicleControllerParameters *controller_parameters,
    RacingAction *action)
{
    alignas(32) float inputs[NN_INPUT_COUNT];
    observation_to_nn_inputs(observation, inputs);
    float hidden1[NN_HIDDEN1_COUNT];
    float hidden2[NN_HIDDEN2_COUNT];

    for (int neuron = 0; neuron < NN_HIDDEN1_COUNT; neuron++) {
        float sum = policy->hidden1_bias[neuron] +
            neural_dot_product<NN_INPUT_COUNT>(
                policy->hidden1_weights[neuron], inputs);
        hidden1[neuron] = tanhf(sum);
    }

    for (int neuron = 0; neuron < NN_HIDDEN2_COUNT; neuron++) {
        float sum = policy->hidden2_bias[neuron] +
            neural_dot_product<NN_HIDDEN1_COUNT>(
                policy->hidden2_weights[neuron], hidden1);
        hidden2[neuron] = tanhf(sum);
    }

    float raw[NN_OUTPUT_COUNT];
    for (int output = 0; output < NN_OUTPUT_COUNT; output++) {
        raw[output] = policy->output_bias[output] +
            neural_dot_product<NN_HIDDEN2_COUNT>(
                policy->output_weights[output], hidden2);
    }

    float maximum_speed = car_parameters_pointer->max_speed_kmh / 3.6f;
    action->desired_speed =
        maximum_speed / (1.0f + expf(-raw[0]));
    action->desired_lateral_offset =
        controller_parameters->maximum_lateral_offset * tanhf(raw[1]);
    action->desired_heading_offset =
        controller_parameters->maximum_heading_offset * tanhf(raw[2]);
}


float cross_2d(Vec2 a, Vec2 b)
{
    return a.x * b.y - a.y * b.x;
}


/* Build the closed road borders with the averaged-normal/miter rule used
   by _main_.cpp. The miter denominator is clamped there to keep sharp
   corners bounded; retain that rule here. */
void initialize_track_borders(Track *track)
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

        if (bisector_length < 1e-6f) {
            bisector = segment_normals[i];
        }
        else {
            bisector = vmul(bisector, 1.0f / bisector_length);
        }

        float cosine_half = vdot(bisector, segment_normals[previous]);
        if (cosine_half < 0.15f)
            cosine_half = 0.15f;

        float miter = TRACK_HALF_WIDTH / cosine_half;
        left_points[i] = vadd(track->points[i], vmul(bisector, miter));
        right_points[i] = vsub(track->points[i], vmul(bisector, miter));
    }

    /* _main_.cpp classifies the left edge as outside for a CCW loop.
       Keep the same ordering in the cached border arrays. */
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

    /* As in _main_.cpp, orient the timing line perpendicular to the
       outgoing first segment rather than along the corner miter. */
    track->finish_line_start = vadd(
        track->points[0], vmul(segment_normals[0], TRACK_HALF_WIDTH));
    track->finish_line_end = vsub(
        track->points[0], vmul(segment_normals[0], TRACK_HALF_WIDTH));
    track->has_finish_line = 1;
}


int rotated_track_index(const Track *track, int geojson_index)
{
    return (geojson_index - TRACK_START_POINT_INDEX +
            track->point_count) % track->point_count;
}


void initialize_track_sectors(Track *track)
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


/* ================================================================
   TRACK GENERATION
   ================================================================ */

/*
 * Creates a non-trivial closed circuit.
 *
 * You can later replace this with track data loaded from a file.
 */

int load_track_geojson(
    Track *track,
    const char *filename)
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

    latitude_scale = cosf(origin_lat *
                           (float)M_PI / 180.0f);

    for (int i = 0; i < count; i++) {
        float longitude = track->points[i].x;
        float latitude = track->points[i].y;

        track->points[i].x =
            (longitude - origin_lon) *
            111320.0f * latitude_scale;

        track->points[i].y =
            (latitude - origin_lat) *
            110540.0f;
    }

    /* GeoJSON rings commonly repeat the first point at the end. The
       track is stored cyclically, so retaining it would create a zero-
       length closing segment and an invalid border normal. */
    if (count > 3 &&
        vlength(vsub(track->points[count - 1], track->points[0])) < 1e-4f)
    {
        count--;
    }

    /* The requested start is the 70th GeoJSON coordinate (zero-based
       index 69). Rotate the closed loop so all existing s=0 logic,
       including the start/finish line, uses that point. */
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

    /* Calculate cumulative distances */

    track->s[0] = 0.0f;

    for (int i = 1; i < track->point_count; i++) {

        float d =
            vlength(
                vsub(
                    track->points[i],
                    track->points[i - 1]));

        track->s[i] =
            track->s[i - 1] + d;
    }

    float closing =
        vlength(
            vsub(track->points[0],
                 track->points[track->point_count - 1]));

    track->total_length =
        track->s[track->point_count - 1] + closing;

    initialize_track_sectors(track);

    free(text);
    return 1;
}


/* ================================================================
   TRACK SAMPLE AT S
   ================================================================ */

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


static int track_segment_index_at_s(const Track *track, float s)
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


Vec2 track_position_at_s(
    const Track *track,
    float s)
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


/* ================================================================
   TRACK HEADING AT S
   ================================================================ */

float track_heading_at_s(
    const Track *track,
    float s)
{
    Vec2 p1 =
        track_position_at_s(
            track, s - TRACK_HEADING_HALF_SAMPLE_DISTANCE);

    Vec2 p2 =
        track_position_at_s(
            track, s + TRACK_HEADING_HALF_SAMPLE_DISTANCE);

    Vec2 d =
        vsub(p2, p1);

    return atan2f(d.y, d.x);
}


float wrap_angle(float angle)
{
    while (angle > (float)M_PI)
        angle -= 2.0f * (float)M_PI;
    while (angle < -(float)M_PI)
        angle += 2.0f * (float)M_PI;
    return angle;
}


static float closest_track_s_near(
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
        float t = segment_length_squared > 0.0f
            ? vdot(vsub(position, track->points[i]), segment) /
              segment_length_squared
            : 0.0f;
        t = fmaxf(0.0f, fminf(t, 1.0f));

        Vec2 nearest = vadd(track->points[i], vmul(segment, t));
        float distance_squared = vdot(
            vsub(position, nearest),
            vsub(position, nearest));
        if (distance_squared < best_distance_squared) {
            best_distance_squared = distance_squared;
            float segment_length = i == track->point_count - 1
                ? track->total_length - track->s[i]
                : track->s[i + 1] - track->s[i];
            best_s = track->s[i] + segment_length * t;
            best_segment = i;
        }
    }

    if (best_s >= track->total_length)
        best_s -= track->total_length;
    *closest_segment = best_segment;
    return best_s;
}


int point_is_on_segment(
    Vec2 point,
    Vec2 start,
    Vec2 end)
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


int car_center_is_inside_track(const Track *track, Vec2 point)
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


void update_lookahead_points(RacingEnv *environment)
{
    for (int point = 0; point < car_parameters.lookahead_point_count; point++) {
        display_car.lookahead_point[point] = track_position_at_s(
            &display_track,
            display_car.track_s + lookahead_distance_for_point(environment, point));
    }
}


float cross2d(Vec2 a, Vec2 b)
{
    return a.x * b.y - a.y * b.x;
}


float lidar_segment_distance(
    Vec2 origin,
    Vec2 direction,
    BorderSegment segment,
    float lidar_range)
{
    Vec2 edge = vsub(segment.end, segment.start);
    float denominator = cross2d(direction, edge);
    if (fabsf(denominator) < 1e-6f)
        return lidar_range;

    Vec2 offset = vsub(segment.start, origin);
    float ray_distance = cross2d(offset, edge) / denominator;
    float segment_amount = cross2d(offset, direction) / denominator;
    if (ray_distance >= 0.0f && ray_distance <= lidar_range &&
        segment_amount >= 0.0f && segment_amount <= 1.0f)
    {
        return ray_distance;
    }
    return lidar_range;
}


void update_lidar_sensors(RacingEnv *environment)
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


int car_has_left_track(const RacingEnv *environment)
{
    const CarState *car = &environment->car;
    Vec2 center = track_position_at_s(
        &display_track,
        car->track_s);
    float center_distance = vlength(
        vsub(car->position, center));
    float maximum_center_distance = TRACK_HALF_WIDTH - car_parameters.half_width;

    /* boundary_polygon describes the open ribbon from centerline point 0
       through the last point. Its artificial end caps exclude the closing
       centerline segment from the last point back to point 0. Use the exact
       centerline-width test over that entire segment so a valid approach to
       start/finish is not classified as outside. Genuine off-track centers
       still exceed maximum_center_distance and remain crashes. */
    int last_point = display_track.point_count - 1;
    if (last_point >= 0 && car->track_s >= display_track.s[last_point])
        return center_distance > maximum_center_distance;

    return !car_center_is_inside_track(&display_track, car->position) ||
        center_distance > maximum_center_distance;
}


void racing_env_reset_at(
    RacingEnv *environment,
    float track_s,
    int timing_start_line)
{
    memset(&display_car, 0, sizeof(display_car));
    environment->stationary_elapsed = 0.0f;
    environment->speed_error_integral = 0.0f;
    environment->filtered_desired_speed = 0.0f;
    environment->filtered_desired_lateral_offset = 0.0f;
    environment->filtered_desired_heading_offset = 0.0f;
    environment->controller_target_initialized = 0;
    memset(environment->history, 0, sizeof(environment->history));
    environment->history_count = 0;
    environment->history_sample_elapsed = 0.0f;
    memset(&environment->pending_history_sample, 0,
           sizeof(environment->pending_history_sample));
    environment->pending_history_sample_valid = 0;
    environment->last_step_reward = 0.0f;
    environment->corner_exit_phase = 0;
    environment->corner_exit_peak_curvature = 0.0f;
    environment->corner_exit_apex_speed = 0.0f;
    environment->corner_exit_distance_from_apex = 0.0f;
    display_car.last_lap = -1.0f;
    display_car.completed_circuit_time = -1.0f;
    for (int sector = 0; sector < TRACK_SECTOR_COUNT; sector++)
        display_car.split_time[sector] = -1.0f;
    display_car.heading = track_heading_at_s(
        &display_track,
        track_s);
    Vec2 forward = vec2(
        cosf(display_car.heading),
        sinf(display_car.heading));
    display_car.position = vsub(
        track_position_at_s(&display_track, track_s),
        vmul(forward, car_parameters.front_offset));
    environment->closest_track_segment = track_segment_index_at_s(
        &display_track, track_s);
    display_car.track_s = closest_track_s_near(
        &display_track,
        display_car.position,
        environment->closest_track_segment,
        &environment->closest_track_segment);
    display_car.timing_start_line = timing_start_line;
    display_car.timing_next_line =
        (timing_start_line + 1) % TIMING_LINE_COUNT;
    update_lookahead_points(environment);
    update_lidar_sensors(environment);
}


void racing_env_reset(RacingEnv *environment)
{
    racing_env_reset_at(environment, 0.0f, TIMING_LINE_START_FINISH);
}


int timing_line_geometry(
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


int swept_front_crosses_timing_line(
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


void update_car_timing(
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


void racing_env_push_history(
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


void racing_env_update_history(
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
            racing_env_push_history(
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


void vehicle_controller_update(
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
            (action->desired_speed -
             environment->filtered_desired_speed);
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


void racing_env_step(
    RacingEnv *environment,
    const RacingAction *action,
    float elapsed,
    RacingStepResult *result)
{
    memset(result, 0, sizeof(*result));
    if (display_track.total_length <= 0.0f)
    {
        return;
    }

    CarHistorySample previous_history_sample;
    car_history_sample_from_state(
        &display_car, &previous_history_sample);
    float previous_track_s = display_car.track_s;
    float previous_command = display_car.steering_command;
    float previous_speed = display_car.v_x;
    float previous_longitudinal_acceleration = display_car.a_x;
    float previous_throttle = display_car.throttle;
    float previous_brake = display_car.brake;
    Vec2 previous_position = display_car.position;
    float previous_heading = display_car.heading;
    float elapsed_before_step = display_car.lap_elapsed;
    Vec2 previous_front = vadd(
        previous_position,
        vmul(vec2(cosf(previous_heading), sinf(previous_heading)),
             car_parameters.front_offset));

    vehicle_controller_update(environment, action, elapsed);

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
        float brake_force = display_car.brake * car_parameters.max_brake_force_n;
        float deceleration = brake_force / car_parameters.mass_kg;
        if (display_car.v_x > 0.0f)
            deceleration += car_parameters.rolling_deceleration;
        deceleration += car_parameters.aero_deceleration_at_max_speed *
            speed_ratio * speed_ratio;

        /* Hypercar drivetrain: tyre traction limits force at low speed,
           while available power makes tractive force fall as speed rises.
           car_parameters.max_engine_power_w balances rolling and aerodynamic resistance at
           330 km/h, making that speed the physical full-throttle equilibrium
           instead of relying only on the final safety clamp. */
        float power_limited_force = car_parameters.max_engine_power_w /
            fmaxf(display_car.v_x, car_parameters.engine_power_speed_floor);
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
            maximum_lateral_acceleration_at_speed(
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

    display_car.track_s = closest_track_s_near(
        &display_track,
        display_car.position,
        environment->closest_track_segment,
        &environment->closest_track_segment);

    Vec2 current_front = vadd(
        display_car.position,
        vmul(vec2(cosf(display_car.heading), sinf(display_car.heading)),
             car_parameters.front_offset));
    int left_track = car_has_left_track(environment);
    if (!left_track) {
        update_car_timing(
            environment,
            previous_front,
            current_front,
            elapsed,
            elapsed_before_step);
    } else {
        display_car.circuit_completed_this_step = 0;
        display_car.completed_circuit_time = -1.0f;
        display_car.lap_elapsed = elapsed_before_step + elapsed;
    }

    update_lookahead_points(environment);
    update_lidar_sensors(environment);

    float progress = display_car.track_s - previous_track_s;
    if (progress < -display_track.total_length * 0.5f)
        progress += display_track.total_length;
    else if (progress > display_track.total_length * 0.5f)
        progress -= display_track.total_length;

    result->track_progress = progress;
    result->progress_reward = display_track.total_length > 0.0f
        ? progress / display_track.total_length *
            reward_config.lap_progress_reward
        : 0.0f;
    result->time_penalty =
        -elapsed * reward_config.time_penalty_per_second;
    result->pedal_conflict_penalty =
        -display_car.throttle * display_car.brake * elapsed *
            reward_config.pedal_conflict_penalty_per_second;
    result->steering_change_penalty =
        -fabsf(display_car.steering_command - previous_command) *
            reward_config.steering_change_penalty;
    result->lateral_acceleration_penalty =
        -peak_lateral_overload_ratio * elapsed *
            reward_config.excess_lateral_penalty_per_second;
    /* Integrated absolute jerk is the acceleration change over this step.
       Penalizing its magnitude discourages oscillation without penalizing
       sustained hard acceleration or braking. */
    result->longitudinal_jerk_penalty =
        -fabsf(display_car.a_x - previous_longitudinal_acceleration) *
            reward_config.curriculum_longitudinal_jerk_penalty;
    result->pedal_change_penalty =
        -(fabsf(display_car.throttle - previous_throttle) +
          fabsf(display_car.brake - previous_brake)) *
            reward_config.curriculum_pedal_change_penalty;
    result->curriculum_pedal_conflict_penalty =
        -display_car.throttle * display_car.brake * elapsed *
            reward_config.curriculum_pedal_conflict_penalty_per_second;
    result->corner_exit_speed_reward = 0.0f;
    if (left_track) {
        /* A corner earns nothing unless the car remains on track through the
           complete apex-to-exit measurement zone. */
        environment->corner_exit_phase = 0;
        environment->corner_exit_peak_curvature = 0.0f;
        environment->corner_exit_apex_speed = 0.0f;
        environment->corner_exit_distance_from_apex = 0.0f;
    } else if (progress > 0.0f &&
               reward_config.corner_exit_zone_length > 0.0f &&
               reward_config.corner_exit_curvature_drop_threshold > 0.0f &&
               reward_config.corner_exit_speed_gain_scale > 0.0f) {
        float current_curvature = fabsf(track_curvature_at_s(
            &display_track,
            display_car.track_s,
            car_parameters.lookahead_curvature_sample_distance));

        if (environment->corner_exit_phase == 0 &&
            current_curvature >= reward_config.corner_exit_minimum_curvature) {
            environment->corner_exit_phase = 1;
            environment->corner_exit_peak_curvature = current_curvature;
            environment->corner_exit_apex_speed = display_car.v_x;
            environment->corner_exit_distance_from_apex = 0.0f;
        } else if (environment->corner_exit_phase == 1) {
            if (current_curvature >
                environment->corner_exit_peak_curvature) {
                /* The geometric apex is the greatest curvature reached in
                   this corner. Keep its speed and restart the exit distance. */
                environment->corner_exit_peak_curvature = current_curvature;
                environment->corner_exit_apex_speed = display_car.v_x;
                environment->corner_exit_distance_from_apex = 0.0f;
            } else {
                environment->corner_exit_distance_from_apex += progress;
                if (environment->corner_exit_peak_curvature -
                        current_curvature >=
                    reward_config.corner_exit_curvature_drop_threshold) {
                    environment->corner_exit_phase = 2;
                }
            }
        } else if (environment->corner_exit_phase == 2) {
            /* If a compound corner tightens again, move the apex forward
               instead of rewarding an intermediate bend as a full exit. */
            if (current_curvature >
                environment->corner_exit_peak_curvature) {
                environment->corner_exit_phase = 1;
                environment->corner_exit_peak_curvature = current_curvature;
                environment->corner_exit_apex_speed = display_car.v_x;
                environment->corner_exit_distance_from_apex = 0.0f;
            } else {
                environment->corner_exit_distance_from_apex += progress;
            }

            if (environment->corner_exit_phase == 2 &&
                environment->corner_exit_distance_from_apex >=
                    reward_config.corner_exit_zone_length) {
                /* One bounded reward per corner. Absolute exit speed rewards
                   carrying momentum; speed gain rewards acceleration from the
                   measured apex. Waiting for the full zone makes an off-track
                   excursion before this point cancel the event. */
                float maximum_speed =
                    car_parameters.max_speed_kmh / 3.6f;
                float exit_speed_ratio = maximum_speed > 0.0f
                    ? clamp01(display_car.v_x / maximum_speed)
                    : 0.0f;
                float speed_gain_ratio = clamp01(
                    (display_car.v_x -
                     environment->corner_exit_apex_speed) /
                    reward_config.corner_exit_speed_gain_scale);
                float exit_score =
                    0.5f * exit_speed_ratio * exit_speed_ratio +
                    0.5f * speed_gain_ratio;

                result->corner_exit_speed_reward =
                    reward_config.corner_exit_speed_reward *
                    clamp01(exit_score);
                environment->corner_exit_phase = 3;
            }
        } else if (current_curvature <
                   reward_config.corner_exit_minimum_curvature) {
            /* Re-arm only after leaving the rewarded corner, preventing one
               long exit from producing several rewards. */
            environment->corner_exit_phase = 0;
            environment->corner_exit_peak_curvature = 0.0f;
            environment->corner_exit_apex_speed = 0.0f;
            environment->corner_exit_distance_from_apex = 0.0f;
        }
    }

    if (display_car.v_x < reward_config.stationary_speed_threshold)
        environment->stationary_elapsed += elapsed;
    else
        environment->stationary_elapsed = 0.0f;

    result->left_track = left_track;
    result->stationary = environment->stationary_elapsed >=
        reward_config.stationary_timeout;
    result->completed_lap = display_car.circuit_completed_this_step;
    if (left_track)
        result->terminal_reward -= reward_config.off_track_penalty;
    else if (result->stationary)
        result->terminal_reward -= reward_config.stationary_penalty;
    else if (result->completed_lap) {
        float maximum_speed = car_parameters.max_speed_kmh / 3.6f;
        float average_lap_speed = display_car.completed_circuit_time > 0.0f
            ? display_track.total_length /
                display_car.completed_circuit_time
            : 0.0f;
        float speed_ratio = maximum_speed > 0.0f
            ? fmaxf(0.0f, fminf(average_lap_speed / maximum_speed, 1.0f))
            : 0.0f;
        result->terminal_reward +=
            reward_config.lap_completion_reward +
            reward_config.lap_speed_reward * speed_ratio;
    }
    result->terminated =
        left_track || result->stationary || result->completed_lap;
    result->reward =
        result->progress_reward +
        result->time_penalty +
        result->pedal_conflict_penalty +
        result->steering_change_penalty +
        result->lateral_acceleration_penalty +
        result->terminal_reward;
    environment->last_step_reward = result->reward;
    racing_env_update_history(
        environment, &previous_history_sample, elapsed);
    racing_env_observe(environment, &result->observation);
}


#undef car_parameters
#undef reward_config
#undef controller_config
#undef display_car
#undef display_track
