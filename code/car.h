#pragma once

typedef struct {
    float max_speed_kmh;
    float wheelbase;
    float max_steering_angle;
    float max_steering_rate;
    float max_yaw_rate;
    float mass_kg;
    float max_engine_force_n;
    float max_engine_power_w;
    float engine_power_speed_floor;
    float max_brake_force_n;
    float rolling_deceleration;
    float aero_deceleration_at_max_speed;
    float mechanical_lateral_acceleration;
    float aerodynamic_lateral_acceleration_at_max_speed;
    float half_width;
    float front_offset;

    int lidar_sensor_count;
    float lidar_angle_step_degrees;
    float lidar_range;

    int lookahead_point_count;
    float lookahead_time[MAX_LOOKAHEAD_POINT_COUNT];
    float lookahead_minimum_distance[MAX_LOOKAHEAD_POINT_COUNT];
    float lookahead_curvature_sample_distance;
    float lookahead_curvature_scale;
} CarParameters;

typedef struct {
    float maximum_lateral_offset;
    float maximum_heading_offset;
    float target_filter_time_constant;
    float speed_proportional_gain;
    float speed_integral_gain;
    float speed_integral_limit;
    float lateral_error_gain;
    float lateral_softening_speed;
    float heading_error_gain;
    float track_heading_lookahead_time;
    float track_heading_minimum_lookahead;
} VehicleControllerParameters;

typedef struct {
    float v_x;
    float v_y;
    float a_x;
    float a_y;
    float steering_angle;
    float throttle;
    float brake;
} CarHistorySample;

typedef struct {
    Vec2 position;
    float heading;
    float v_x;
    float v_y;
    float a_x;
    float a_y;
    float steering_angle;
    float yaw_rate;
    float throttle;
    float brake;
    float steering_command;
    float desired_speed;
    float desired_lateral_offset;
    float desired_heading_offset;
    float track_s;
    float lap_elapsed;
    float last_lap;
    float split_time[TRACK_SECTOR_COUNT];
    int lap_count;
    int timing_start_line;
    int timing_next_line;
    int timing_crossing_count;
    int circuit_completed_this_step;
    float completed_circuit_time;
    float lidar_distance[MAX_LIDAR_SENSOR_COUNT];
    Vec2 lookahead_point[MAX_LOOKAHEAD_POINT_COUNT];
} CarState;

struct RacingAction;
struct RacingEnv;

void car_parameters_initialize_default(CarParameters *parameters);
void vehicle_controller_parameters_initialize_default(
    VehicleControllerParameters *parameters);
float car_maximum_lateral_acceleration_at_speed(
    const CarParameters *parameters,
    float speed);
float car_encode_longitudinal_acceleration_input(
    const CarParameters *parameters,
    float acceleration);
float car_lookahead_distance_for_point(
    const struct RacingEnv *environment,
    int point);
void car_update_lookahead_points(struct RacingEnv *environment);
void car_update_lidar_sensors(struct RacingEnv *environment);
int car_has_left_track(const struct RacingEnv *environment);
void car_update_timing(
    struct RacingEnv *environment,
    Vec2 previous_front,
    Vec2 current_front,
    float elapsed,
    float elapsed_before_step);
void car_history_sample_from_state(
    const CarState *car,
    CarHistorySample *sample);
void car_update_history(
    struct RacingEnv *environment,
    const CarHistorySample *previous,
    float elapsed);
void car_controller_update(
    struct RacingEnv *environment,
    const struct RacingAction *action,
    float elapsed);
float car_step_dynamics(struct RacingEnv *environment, float elapsed);
