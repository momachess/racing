#pragma once

/*
 * neuro_racer.cpp
 *
 * Minimal Direct2D racing-track viewer in C++.
 *
 * A kinematic bicycle car autonomously follows the loaded track centreline.
 *
 * Build with MSVC:
 *      cl /O2 /EHsc gpt_first_step.cpp /Fe:gpt_first_step_desktop.exe /link /SUBSYSTEM:WINDOWS
 *
 * Run:
 *      gpt_first_step_desktop.exe
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <windows.h>
#include <d2d1.h>
#include <dwrite.h>

#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "d2d1.lib")
#pragma comment(lib, "dwrite.lib")

/* ================================================================
   CONSTANTS
   ================================================================ */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ANIMATION_INTERVAL (1.0 / 60.0)
#define PHYSICS_MAX_STEP 0.01f
#define MAX_LIDAR_SENSOR_COUNT 7
#define MAX_LOOKAHEAD_POINT_COUNT 4
#define CAR_HISTORY_STEP_COUNT 5
#define CAR_HISTORY_VALUE_COUNT 7
#define CAR_HISTORY_SAMPLE_INTERVAL 0.05f
#define NN_INPUT_COUNT \
    (6 + CAR_HISTORY_STEP_COUNT * CAR_HISTORY_VALUE_COUNT + \
     MAX_LIDAR_SENSOR_COUNT + 4 * MAX_LOOKAHEAD_POINT_COUNT)
#define NN_HIDDEN1_COUNT 32
#define NN_HIDDEN2_COUNT 16
#define NN_OUTPUT_COUNT 3
#define NN_GENOME_COUNT 2659
#define GA_POPULATION_SIZE 64
#define GA_ELITE_COUNT 2
#define GA_TOURNAMENT_SIZE 3
#define GA_MAX_EPISODE_STEPS 12000
#define GA_START_POSITION_COUNT 3
#define GA_MUTATION_PROBABILITY 0.10f
#define GA_MUTATION_SCALE 0.05f
#define GA_FIXED_STEP (1.0f / 60.0f)
#define WM_GA_GENERATION_COMPLETE (WM_APP + 1)

static_assert(
    NN_GENOME_COUNT ==
        NN_HIDDEN1_COUNT * NN_INPUT_COUNT + NN_HIDDEN1_COUNT +
        NN_HIDDEN2_COUNT * NN_HIDDEN1_COUNT + NN_HIDDEN2_COUNT +
        NN_OUTPUT_COUNT * NN_HIDDEN2_COUNT + NN_OUTPUT_COUNT,
    "NN_GENOME_COUNT must match the neural-network topology");

#define MAX_TRACK_POINTS 512
#define MAX_BOUNDARY_POINTS (MAX_TRACK_POINTS * 2)
#define RIGHT_PANE_WIDTH 384
#define RIGHT_PANE_BUTTON_WIDTH 80
#define RIGHT_PANE_BUTTON_HEIGHT 36
#define RIGHT_PANE_BUTTON_GAP 8

#define TRACK_HALF_WIDTH 7.0f
#define TRACK_START_POINT_INDEX 70
#define TRACK_SECTOR1_POINT_INDEX 106
#define TRACK_SECTOR2_POINT_INDEX 33
#define TRACK_SECTOR_COUNT 2
#define TIMING_LINE_START_FINISH 0
#define TIMING_LINE_SECTOR1 1
#define TIMING_LINE_SECTOR2 2
#define TIMING_LINE_COUNT 3


/* ================================================================
   BASIC TYPES
   ================================================================ */

typedef struct {
    float x;
    float y;
} Vec2;

typedef struct {
    Vec2 start;
    Vec2 end;
} BorderSegment;

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

    /* cumulative distance around track */
    float s[MAX_TRACK_POINTS];

    float total_length;

} Track;

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
    float max_lateral_acceleration;
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
    float lap_progress_reward;
    float time_penalty_per_second;
    float pedal_conflict_penalty_per_second;
    float steering_change_penalty;
    float excess_lateral_penalty_per_second;
    float off_track_penalty;
    float stationary_penalty;
    float lap_completion_reward;
    float lap_speed_reward;
    float stationary_speed_threshold;
    float stationary_timeout;
} RewardParameters;

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

typedef struct {
    float hidden1_weights[NN_HIDDEN1_COUNT][NN_INPUT_COUNT];
    float hidden1_bias[NN_HIDDEN1_COUNT];
    float hidden2_weights[NN_HIDDEN2_COUNT][NN_HIDDEN1_COUNT];
    float hidden2_bias[NN_HIDDEN2_COUNT];
    float output_weights[NN_OUTPUT_COUNT][NN_HIDDEN2_COUNT];
    float output_bias[NN_OUTPUT_COUNT];
} NeuralPolicy;

typedef struct {
    const Track *track;
    const CarParameters *parameters;
    const RewardParameters *reward_parameters;
    const VehicleControllerParameters *controller_parameters;
    unsigned int random_state;
    float stationary_elapsed;
    float speed_error_integral;
    float filtered_desired_speed;
    float filtered_desired_lateral_offset;
    float filtered_desired_heading_offset;
    int controller_target_initialized;
    CarHistorySample history[CAR_HISTORY_STEP_COUNT];
    int history_count;
    float history_sample_elapsed;
    CarHistorySample pending_history_sample;
    int pending_history_sample_valid;
    CarState car;
} RacingEnv;

typedef struct {
    float desired_speed;
    float desired_lateral_offset;
    float desired_heading_offset;
} RacingAction;

typedef struct {
    float values[NN_INPUT_COUNT];
} RacingObservation;

typedef struct {
    RacingObservation observation;
    float reward;
    float track_progress;
    float progress_reward;
    float time_penalty;
    float pedal_conflict_penalty;
    float steering_change_penalty;
    float lateral_acceleration_penalty;
    float terminal_reward;
    int terminated;
    int truncated;
    int left_track;
    int stationary;
    int completed_lap;
} RacingStepResult;

typedef struct {
    RacingEnv *environment;
    const NeuralPolicy *policy;
} EpisodeEvaluator;

typedef struct {
    float fitness;
    float progress_reward;
    float control_penalty;
    float speed_integral;
    float forward_progress;
    float circuit_time;
    int steps;
    int completed_lap;
    int left_track;
    int stationary;
} EpisodeEvaluationResult;

typedef struct {
    float genes[NN_GENOME_COUNT];
    float fitness;
    float average_speed;
    float track_progress;
} Genome;

typedef struct {
    int initialized;
    int generation;
    float best_fitness;
    float reported_best_fitness;
    float reported_average_fitness;
    float reported_best_average_speed;
    float reported_best_track_progress;
    float reported_average_progress_reward;
    float reported_average_control_penalty;
    float reported_off_track_percentage;
    float reported_stationary_percentage;
    float reported_lap_completion_percentage;
    int completed_generations;
} GeneticTraining;

typedef struct TrainingContext TrainingContext;

typedef struct {
    int candidate;
    LONG batch_token;
    TrainingContext *owner;
    RacingEnv environment;
    NeuralPolicy policy;
    float fitness;
    float average_speed;
    float track_progress;
    float average_progress_reward;
    float average_control_penalty;
    int start_finish_left_track;
    int start_finish_stationary;
    int completed_start_finish_lap;
    PTP_WORK work;
} GaWorkContext;

struct TrainingContext {
    const Track *track;
    const CarParameters *parameters;
    const RewardParameters *reward_parameters;
    const VehicleControllerParameters *controller_parameters;
    NeuralPolicy *active_policy;
    HWND completion_window;
    unsigned int random_state;
    Genome population[GA_POPULATION_SIZE];
    Genome next_population[GA_POPULATION_SIZE];
    Genome best_genome;
    GeneticTraining metrics;
    PTP_POOL thread_pool;
    TP_CALLBACK_ENVIRON callback_environment;
    int callback_environment_initialized;
    GaWorkContext work_contexts[GA_POPULATION_SIZE];
    volatile LONG completed_work_count;
    LONG next_batch_token;
    LONG active_batch_token;
    int workers_active;
    int discard_active_batch;
    int running;
};

typedef struct {
    int map_width;
    float min_x;
    float max_y;
    float scale;
    float offset_x;
    float offset_y;
    D2D1_MATRIX_3X2_F world_to_screen;
} TrackRenderView;

typedef struct {
    ID2D1Factory *d2d_factory;
    ID2D1StrokeStyle *d2d_round_stroke;
    ID2D1HwndRenderTarget *d2d_target;
    ID2D1SolidColorBrush *d2d_track_brush;
    ID2D1SolidColorBrush *d2d_center_brush;
    ID2D1SolidColorBrush *d2d_background_brush;
    ID2D1SolidColorBrush *d2d_white_brush;
    ID2D1SolidColorBrush *d2d_black_brush;
    ID2D1SolidColorBrush *d2d_car_brush;
    ID2D1SolidColorBrush *d2d_blue_brush;
    ID2D1SolidColorBrush *d2d_red_brush;
    ID2D1SolidColorBrush *d2d_tick_brush;
    ID2D1SolidColorBrush *d2d_gauge_text_brush;
    ID2D1SolidColorBrush *d2d_command_brush;
    ID2D1SolidColorBrush *d2d_command_hover_brush;
    ID2D1SolidColorBrush *d2d_text_brush;
    ID2D1SolidColorBrush *d2d_pane_background_brush;
    ID2D1SolidColorBrush *d2d_pane_border_brush;
    ID2D1SolidColorBrush *d2d_pane_heading_brush;
    ID2D1SolidColorBrush *d2d_pane_label_brush;
    ID2D1SolidColorBrush *d2d_pane_value_brush;
    IDWriteFactory *dwrite_factory;
    IDWriteTextFormat *dwrite_format;
    IDWriteTextFormat *dwrite_pane_format;
    IDWriteTextFormat *dwrite_button_format;
    IDWriteTextFormat *dwrite_gauge_format;
    IDWriteTextFormat *dwrite_gauge_tick_format;
    IDWriteTextFormat *dwrite_speed_format;
    float track_zoom;
    int track_pan_x;
    int track_pan_y;
    int track_panning;
    int track_pan_start_x;
    int track_pan_start_y;
    int track_pan_origin_x;
    int track_pan_origin_y;
    int command_hover;
} RendererContext;

typedef struct {
    HWND window;
    char status_message[256];
    Track track;
    CarParameters car_parameters;
    RewardParameters reward_parameters;
    VehicleControllerParameters controller_parameters;
    RacingEnv environment;
    NeuralPolicy policy;
    TrainingContext training;
    RendererContext renderer;
    int animation_running;
} ApplicationContext;


#define release_d2d(resource_pointer)                \
    do {                                             \
        if (*(resource_pointer)) {                   \
            (*(resource_pointer))->Release();        \
            *(resource_pointer) = NULL;              \
        }                                            \
    } while (0)


/* Shared state declarations used while the procedural modules are
   orchestrated by the application layer. */
void set_status_message(ApplicationContext *application, const char *text);
void format_lap_time(float seconds, char *text, size_t text_size);
int right_pane_command_at(const RECT *client, int x, int y);
int right_pane_command_is_disabled(int command);

Vec2 vec2(float x, float y);
Vec2 vadd(Vec2 a, Vec2 b);
Vec2 vsub(Vec2 a, Vec2 b);
Vec2 vmul(Vec2 a, float scalar);
float vdot(Vec2 a, Vec2 b);
float vlength(Vec2 a);
Vec2 vnormalize(Vec2 a);

void car_parameters_initialize_default(CarParameters *parameters);
void reward_parameters_initialize_default(RewardParameters *parameters);
void vehicle_controller_parameters_initialize_default(
    VehicleControllerParameters *parameters);
void racing_env_initialize(
    RacingEnv *environment,
    const Track *track,
    const CarParameters *parameters,
    const RewardParameters *reward_parameters,
    const VehicleControllerParameters *controller_parameters);
void neural_policy_initialize(
    NeuralPolicy *policy,
    unsigned int *random_state);
void network_to_genome(const NeuralPolicy *network,
                       float genes[NN_GENOME_COUNT]);
void genome_to_network(const float genes[NN_GENOME_COUNT],
                       NeuralPolicy *network);
float network_random_unit(unsigned int *random_state);
float network_random_gaussian(unsigned int *random_state);
int load_track_geojson(Track *track, const char *filename);
void racing_env_reset_at(
    RacingEnv *environment,
    float track_s,
    int timing_start_line);
void racing_env_reset(RacingEnv *environment);
void racing_env_observe(
    const RacingEnv *environment,
    RacingObservation *observation);
void racing_env_step(
    RacingEnv *environment,
    const RacingAction *action,
    float elapsed,
    RacingStepResult *result);
int car_has_left_track(const RacingEnv *environment);
void neural_policy_predict(
    const NeuralPolicy *policy,
    const RacingObservation *observation,
    const CarParameters *car_parameters,
    const VehicleControllerParameters *controller_parameters,
    RacingAction *action);
void episode_evaluate(
    EpisodeEvaluator *evaluator,
    float start_s,
    int timing_start_line,
    EpisodeEvaluationResult *result);

void training_context_initialize(
    TrainingContext *training,
    const Track *track,
    const CarParameters *parameters,
    const RewardParameters *reward_parameters,
    const VehicleControllerParameters *controller_parameters,
    NeuralPolicy *active_policy,
    HWND completion_window);
int initialize_ga_thread_pool(TrainingContext *training);
int submit_ga_generation(TrainingContext *training);
void cleanup_ga_thread_pool(TrainingContext *training);
void initialize_ga_population(TrainingContext *training);
int collect_completed_ga_generation(
    TrainingContext *training,
    LONG batch_token);

void renderer_context_initialize(RendererContext *renderer);
HRESULT create_d2d_resources(RendererContext *renderer, HWND window);
void discard_d2d_target(RendererContext *renderer);
void renderer_context_shutdown(RendererContext *renderer);
int render_direct2d(
    RendererContext *renderer,
    HWND window,
    const RacingEnv *environment,
    const TrainingContext *training,
    int animation_running,
    const char *status_message);
