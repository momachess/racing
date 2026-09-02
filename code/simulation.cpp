#include "racing.h"

#include <random>
#if defined(RACING_USE_AVX2)
#include <immintrin.h>
#endif

#define display_track (*environment->track)
#define display_car (environment->car)
#define car_parameters (*environment->parameters)
#define reward_config (*environment->reward_parameters)

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

float encode_signed_input(float value, float magnitude)
{
    if (magnitude <= 0.0f)
        return 0.5f;
    return clamp01(0.5f + 0.5f * value / magnitude);
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


static const float fixed_curvature_distance[
    FIXED_CURVATURE_INPUT_COUNT] = {10.0f, 25.0f, 50.0f, 100.0f};


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
        car_encode_longitudinal_acceleration_input(
            environment->parameters, display_car.a_x);
    observation->encoded_lateral_acceleration = encode_signed_input(
        display_car.a_y,
        car_maximum_lateral_acceleration_at_speed(
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
            car_encode_longitudinal_acceleration_input(
                environment->parameters, sample.a_x);
        history->encoded_lateral_acceleration = encode_signed_input(
            sample.a_y,
            car_maximum_lateral_acceleration_at_speed(
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
        float distance = car_lookahead_distance_for_point(
            environment, point);
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
    display_car.track_s = track_closest_s_near(
        &display_track,
        display_car.position,
        environment->closest_track_segment,
        &environment->closest_track_segment);
    display_car.timing_start_line = timing_start_line;
    display_car.timing_next_line =
        (timing_start_line + 1) % TIMING_LINE_COUNT;
    car_update_lookahead_points(environment);
    car_update_lidar_sensors(environment);
}


void racing_env_reset(RacingEnv *environment)
{
    racing_env_reset_at(environment, 0.0f, TIMING_LINE_START_FINISH);
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

    car_controller_update(environment, action, elapsed);
    float peak_lateral_overload_ratio =
        car_step_dynamics(environment, elapsed);

    display_car.track_s = track_closest_s_near(
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
        car_update_timing(
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

    car_update_lookahead_points(environment);
    car_update_lidar_sensors(environment);

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
    car_update_history(
        environment, &previous_history_sample, elapsed);
    racing_env_observe(environment, &result->observation);
}


#undef car_parameters
#undef reward_config
#undef display_car
#undef display_track
