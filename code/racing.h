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
#define RACING_LINE_STATE_INPUT_COUNT 2
#define FIXED_CURVATURE_INPUT_COUNT 4
#define CAR_HISTORY_STEP_COUNT 5
#define CAR_HISTORY_VALUE_COUNT 7
#define CAR_HISTORY_SAMPLE_INTERVAL 0.05f
#define NN_INPUT_COUNT \
    (6 + CAR_HISTORY_STEP_COUNT * CAR_HISTORY_VALUE_COUNT + \
     MAX_LIDAR_SENSOR_COUNT + 4 * MAX_LOOKAHEAD_POINT_COUNT + \
     RACING_LINE_STATE_INPUT_COUNT + FIXED_CURVATURE_INPUT_COUNT)
#define NN_HIDDEN1_COUNT 32
#define NN_HIDDEN2_COUNT 16
#define NN_OUTPUT_COUNT 3
#define NN_GENOME_COUNT 2851
#define NN_LATERAL_SPEED_SCALE_MPS 20.0f
#define NN_HEADING_ERROR_SCALE_RADIANS ((float)M_PI * 0.25f)
#define GA_POPULATION_SIZE 64
#define GA_RESERVED_PROCESSOR_COUNT 2
#define GA_MAX_EPISODE_STEPS 12000
#define GA_START_POSITION_COUNT 3
#define GA_FIXED_STEP (1.0f / 60.0f)
#define ES_PERTURBATION_STDDEV 0.05f
#define ES_LEARNING_RATE 0.01f
#define ES_ADAM_BETA1 0.9f
#define ES_ADAM_BETA2 0.999f
#define ES_ADAM_EPSILON 1e-8f
#define TRAINING_TREND_HISTORY_COUNT 30
#define WM_GA_GENERATION_COMPLETE (WM_APP + 1)

static_assert(
    NN_GENOME_COUNT ==
        NN_HIDDEN1_COUNT * NN_INPUT_COUNT + NN_HIDDEN1_COUNT +
        NN_HIDDEN2_COUNT * NN_HIDDEN1_COUNT + NN_HIDDEN2_COUNT +
        NN_OUTPUT_COUNT * NN_HIDDEN2_COUNT + NN_OUTPUT_COUNT,
    "NN_GENOME_COUNT must match the neural-network topology");

static_assert(
    GA_POPULATION_SIZE % 2 == 0,
    "Mirrored ES requires an even population size");

#include "track.h"

#define RIGHT_PANE_WIDTH 384
#define RIGHT_PANE_BUTTON_WIDTH 80
#define RIGHT_PANE_BUTTON_HEIGHT 36
#define RIGHT_PANE_BUTTON_GAP 8
#define RIGHT_PANE_TRAIN_SOURCE_WIDTH 216
#define RIGHT_PANE_TRAIN_SOURCE_HEIGHT 28
#define RIGHT_PANE_TRAIN_SOURCE_GAP 12
#define RIGHT_PANE_FITNESS_BUTTON_WIDTH 112
#define RIGHT_PANE_FITNESS_BUTTON_HEIGHT 28
#define RIGHT_PANE_FITNESS_BUTTON_GAP 8
#define TRACK_POINT_PICK_RADIUS 12.0f
#define RUN_CHART_SAMPLE_COUNT 300
#define RUN_CHART_SAMPLE_INTERVAL_MS 100
#define RUN_CHART_ACCELERATION_RANGE 20.0f
#define LAP_TELEMETRY_SAMPLE_COUNT 320

/* ================================================================
   BASIC TYPES
   ================================================================ */

#include "car.h"

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
    float corner_exit_speed_reward;
    float corner_exit_zone_length;
    float corner_exit_curvature_drop_threshold;
    float corner_exit_minimum_curvature;
    float corner_exit_speed_gain_scale;
    float curriculum_progress_scale;
    float curriculum_incomplete_average_speed_reward;
    float curriculum_longitudinal_jerk_penalty;
    float curriculum_pedal_change_penalty;
    float curriculum_pedal_conflict_penalty_per_second;
    float stationary_speed_threshold;
    float stationary_timeout;
} RewardParameters;

typedef struct {
    float hidden1_weights[NN_HIDDEN1_COUNT][NN_INPUT_COUNT];
    float hidden1_bias[NN_HIDDEN1_COUNT];
    float hidden2_weights[NN_HIDDEN2_COUNT][NN_HIDDEN1_COUNT];
    float hidden2_bias[NN_HIDDEN2_COUNT];
    float output_weights[NN_OUTPUT_COUNT][NN_HIDDEN2_COUNT];
    float output_bias[NN_OUTPUT_COUNT];
} NeuralPolicy;

typedef struct RacingEnv {
    const Track *track;
    const CarParameters *parameters;
    const RewardParameters *reward_parameters;
    const VehicleControllerParameters *controller_parameters;
    unsigned int random_state;
    int closest_track_segment;
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
    float last_step_reward;
    int corner_exit_phase;
    float corner_exit_peak_curvature;
    float corner_exit_apex_speed;
    float corner_exit_distance_from_apex;
    CarState car;
} RacingEnv;

typedef struct {
    float lateral_offset;
    float heading_error;
    float fixed_curvature[FIXED_CURVATURE_INPUT_COUNT];
} RacingLineTelemetry;

typedef struct RacingAction {
    float desired_speed;
    float desired_lateral_offset;
    float desired_heading_offset;
} RacingAction;

typedef struct {
    float normalized_speed;
    float encoded_lateral_speed;
    float encoded_longitudinal_acceleration;
    float encoded_lateral_acceleration;
    float encoded_steering_angle;
    float normalized_throttle;
    float normalized_brake;
} RacingHistoryObservation;

typedef struct {
    float encoded_lateral_offset;
    float encoded_forward_offset;
    float encoded_angle;
    float normalized_curvature;
} RacingLookaheadObservation;

typedef struct {
    float normalized_speed;
    float encoded_lateral_speed;
    float encoded_steering_angle;
    float encoded_yaw_rate;
    float encoded_longitudinal_acceleration;
    float encoded_lateral_acceleration;
    RacingHistoryObservation history[CAR_HISTORY_STEP_COUNT];
    float normalized_lidar_distance[MAX_LIDAR_SENSOR_COUNT];
    RacingLookaheadObservation lookahead[MAX_LOOKAHEAD_POINT_COUNT];
    float encoded_lateral_offset;
    float encoded_heading_error;
    float normalized_fixed_curvature[FIXED_CURVATURE_INPUT_COUNT];
} RacingObservation;

static_assert(
    sizeof(RacingHistoryObservation) ==
        CAR_HISTORY_VALUE_COUNT * sizeof(float),
    "RacingHistoryObservation must match CAR_HISTORY_VALUE_COUNT");
static_assert(
    sizeof(RacingObservation) == NN_INPUT_COUNT * sizeof(float),
    "RacingObservation must contain exactly NN_INPUT_COUNT float values");

typedef struct {
    RacingObservation observation;
    float reward;
    float track_progress;
    float progress_reward;
    float time_penalty;
    float pedal_conflict_penalty;
    float steering_change_penalty;
    float lateral_acceleration_penalty;
    float corner_exit_speed_reward;
    float longitudinal_jerk_penalty;
    float pedal_change_penalty;
    float curriculum_pedal_conflict_penalty;
    float terminal_reward;
    int terminated;
    int truncated;
    int left_track;
    int stationary;
    int completed_lap;
} RacingStepResult;

typedef enum {
    TRAINING_FITNESS_STANDARD = 0,
    TRAINING_FITNESS_CORNER_EXIT_SPEED = 1,
    TRAINING_FITNESS_CURRICULUM = 2
} TrainingFitnessFunction;

typedef struct {
    RacingEnv *environment;
    const NeuralPolicy *policy;
    TrainingFitnessFunction fitness_function;
    float curriculum_performance_blend;
    int use_track_segment;
    float track_segment_length;
} EpisodeEvaluator;

typedef struct {
    float fitness;
    float progress_reward;
    float control_penalty;
    float speed_integral;
    float top_speed;
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
    float top_speed;
    float track_progress;
} Genome;

typedef struct {
    float average_fitness;
    float average_speed;
    float average_progress_reward;
    float average_control_penalty;
    float off_track_percentage;
    float lap_completion_percentage;
    float median_track_progress;
    float performance_blend;
} TrainingGenerationSample;

typedef struct {
    int initialized;
    int generation;
    float best_fitness;
    float reported_best_fitness;
    float reported_average_fitness;
    float reported_best_average_speed;
    float reported_best_top_speed;
    float reported_best_track_progress;
    float reported_average_progress_reward;
    float reported_average_control_penalty;
    float reported_off_track_percentage;
    float reported_stationary_percentage;
    float reported_lap_completion_percentage;
    float reported_median_track_progress;
    float reported_generation_elapsed_seconds;
    int completed_generations;
    TrainingGenerationSample history[TRAINING_TREND_HISTORY_COUNT];
    int history_count;
    int history_next;
} GeneticTraining;

typedef struct TrainingContext TrainingContext;

typedef struct {
    int candidate;
    TrainingContext *owner;
    RacingEnv environment;
    NeuralPolicy policy;
    float fitness;
    float average_speed;
    float top_speed;
    float track_progress;
    float average_progress_reward;
    float average_control_penalty;
    int start_finish_left_track;
    int start_finish_stationary;
    int completed_start_finish_lap;
} GaWorkContext;

typedef struct {
    int first_candidate;
    int candidate_count;
    LONG batch_token;
    TrainingContext *owner;
    PTP_WORK work;
} GaBatchWorkContext;

struct TrainingContext {
    const Track *track;
    const CarParameters *parameters;
    const RewardParameters *reward_parameters;
    const VehicleControllerParameters *controller_parameters;
    NeuralPolicy *active_policy;
    HWND completion_window;
    unsigned int random_state;
    Genome population[GA_POPULATION_SIZE];
    Genome best_genome;
    float es_mean[NN_GENOME_COUNT];
    float es_adam_first_moment[NN_GENOME_COUNT];
    float es_adam_second_moment[NN_GENOME_COUNT];
    unsigned int es_optimizer_step;
    GeneticTraining metrics;
    PTP_POOL thread_pool;
    TP_CALLBACK_ENVIRON callback_environment;
    int callback_environment_initialized;
    GaWorkContext work_contexts[GA_POPULATION_SIZE];
    GaBatchWorkContext batch_work_contexts[GA_POPULATION_SIZE];
    int batch_work_count;
    volatile LONG completed_work_count;
    LONG next_batch_token;
    LONG active_batch_token;
    ULONGLONG generation_started_tick;
    int workers_active;
    int discard_active_batch;
    int start_from_random_weights;
    TrainingFitnessFunction fitness_function;
    int use_track_segment;
    int track_segment_start_geojson_index;
    int track_segment_end_geojson_index;
    float track_segment_start_s;
    float track_segment_length;
    int track_segment_selection_stage;
    int track_segment_hover_geojson_index;
    int render_car_during_training;
    float training_preview_forward_progress;
    float curriculum_performance_blend;
    float curriculum_progress_ema;
    float curriculum_completion_ema;
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
    float speed_kmh;
    float acceleration;
} RunChartSample;

typedef struct {
    float speed_kmh;
    float brake;
    float steering_degrees;
    unsigned char valid;
} LapTelemetrySample;

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
    ID2D1SolidColorBrush *d2d_green_brush;
    ID2D1SolidColorBrush *d2d_amber_brush;
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
    RunChartSample run_chart_samples[RUN_CHART_SAMPLE_COUNT];
    int run_chart_sample_count;
    int run_chart_next_sample;
    ULONGLONG run_chart_last_sample_tick;
    int run_chart_sampling;
    LapTelemetrySample current_lap_telemetry[LAP_TELEMETRY_SAMPLE_COUNT];
    LapTelemetrySample last_lap_telemetry[LAP_TELEMETRY_SAMPLE_COUNT];
    int lap_telemetry_sampling;
    int lap_telemetry_last_available;
    int lap_telemetry_observed_lap_count;
    float lap_telemetry_previous_elapsed;
    int lap_telemetry_has_previous_sample;
} RendererContext;

typedef struct {
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
void format_lap_time(float seconds, char *text, size_t text_size);
int right_pane_command_at(const RECT *client, int x, int y);
int right_pane_command_is_disabled(
    int command,
    int training_is_running,
    int workers_are_active);
int track_geojson_point_near_screen(
    const RendererContext *renderer,
    const Track *track,
    const RECT *client,
    int screen_x,
    int screen_y,
    float maximum_distance);

void reward_parameters_initialize_default(RewardParameters *parameters);
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
unsigned int network_random_seed(void);
void racing_env_reset_at(
    RacingEnv *environment,
    float track_s,
    int timing_start_line);
void racing_env_reset(RacingEnv *environment);
void racing_env_observe(
    const RacingEnv *environment,
    RacingObservation *observation);
void racing_env_line_telemetry(
    const RacingEnv *environment,
    RacingLineTelemetry *telemetry);
void racing_env_step(
    RacingEnv *environment,
    const RacingAction *action,
    float elapsed,
    RacingStepResult *result);
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
    HWND completion_window,
    unsigned int random_seed);
int training_configure_track_segment(
    TrainingContext *training,
    int start_geojson_index,
    int end_geojson_index);
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
    int animation_running);
