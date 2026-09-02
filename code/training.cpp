#include "racing.h"

#define ga_population (training->population)
#define ga_best_genome (training->best_genome)
#define ga_training (training->metrics)
#define ga_thread_pool (training->thread_pool)
#define ga_callback_environment (training->callback_environment)
#define ga_callback_environment_initialized \
    (training->callback_environment_initialized)
#define ga_work_contexts (training->work_contexts)
#define ga_batch_work_contexts (training->batch_work_contexts)
#define ga_batch_work_count (training->batch_work_count)
#define ga_completed_work_count (training->completed_work_count)
#define ga_next_batch_token (training->next_batch_token)
#define ga_active_batch_token (training->active_batch_token)
#define ga_workers_active (training->workers_active)
#define ga_discard_active_batch (training->discard_active_batch)
#define training_running (training->running)
#define driving_policy (*training->active_policy)
#define display_track (*environment->track)
#define display_car (environment->car)

int training_configure_track_segment(
    TrainingContext *training,
    int start_geojson_index,
    int end_geojson_index)
{
    float start_s;
    float end_s;
    if (!training || start_geojson_index == end_geojson_index ||
        !track_s_at_geojson_point(
            training->track, start_geojson_index, &start_s) ||
        !track_s_at_geojson_point(
            training->track, end_geojson_index, &end_s))
    {
        return 0;
    }

    float length = track_forward_distance(training->track, start_s, end_s);
    if (length <= 0.0f)
        return 0;

    training->track_segment_start_geojson_index = start_geojson_index;
    training->track_segment_end_geojson_index = end_geojson_index;
    training->track_segment_start_s = start_s;
    training->track_segment_length = length;
    return 1;
}

void training_context_initialize(
    TrainingContext *training,
    const Track *track,
    const CarParameters *parameters,
    const RewardParameters *reward_parameters,
    const VehicleControllerParameters *controller_parameters,
    NeuralPolicy *active_policy,
    HWND completion_window,
    unsigned int random_seed)
{
    memset(training, 0, sizeof(*training));
    training->track = track;
    training->parameters = parameters;
    training->reward_parameters = reward_parameters;
    training->controller_parameters = controller_parameters;
    training->active_policy = active_policy;
    training->completion_window = completion_window;
    training->random_state = random_seed;
    training->track_segment_hover_geojson_index = -1;
    training_configure_track_segment(
        training,
        TRACK_TRAINING_SEGMENT_START_POINT_INDEX,
        TRACK_TRAINING_SEGMENT_END_POINT_INDEX);
    for (int candidate = 0; candidate < GA_POPULATION_SIZE; candidate++) {
        GaWorkContext *context = &training->work_contexts[candidate];
        context->candidate = candidate;
        context->owner = training;
        racing_env_initialize(
            &context->environment,
            track,
            parameters,
            reward_parameters,
            controller_parameters);
    }
}

static float standard_step_fitness(
    const RacingStepResult *step_result)
{
    return step_result->reward;
}


static float corner_exit_speed_step_fitness(
    const RacingStepResult *step_result)
{
    return step_result->reward +
        step_result->corner_exit_speed_reward;
}


static float curriculum_step_fitness(
    const EpisodeEvaluator *evaluator,
    const RacingStepResult *step_result)
{
    const RewardParameters *parameters =
        evaluator->environment->reward_parameters;
    float foundation_terminal_reward = 0.0f;
    if (step_result->left_track || step_result->stationary)
        foundation_terminal_reward = step_result->terminal_reward;
    else if (step_result->completed_lap)
        foundation_terminal_reward = parameters->lap_completion_reward;

    float foundation_fitness =
        parameters->curriculum_progress_scale *
            step_result->progress_reward +
        foundation_terminal_reward;
    float performance_fitness =
        step_result->time_penalty +
        step_result->steering_change_penalty +
        step_result->lateral_acceleration_penalty +
        step_result->corner_exit_speed_reward +
        step_result->longitudinal_jerk_penalty +
        step_result->pedal_change_penalty +
        step_result->curriculum_pedal_conflict_penalty;

    if (step_result->completed_lap) {
        /* terminal_reward also contains the completed-lap average-speed
           reward. Keep it in the performance component so it is introduced
           gradually and is never counted twice. */
        performance_fitness +=
            step_result->terminal_reward -
            parameters->lap_completion_reward;
    }

    return foundation_fitness +
        evaluator->curriculum_performance_blend * performance_fitness;
}


static float selected_step_fitness(
    const EpisodeEvaluator *evaluator,
    const RacingStepResult *step_result)
{
    switch (evaluator->fitness_function) {
    case TRAINING_FITNESS_CURRICULUM:
        return curriculum_step_fitness(evaluator, step_result);
    case TRAINING_FITNESS_CORNER_EXIT_SPEED:
        return corner_exit_speed_step_fitness(step_result);
    case TRAINING_FITNESS_STANDARD:
    default:
        return standard_step_fitness(step_result);
    }
}


static void finalize_episode_fitness(
    const EpisodeEvaluator *evaluator,
    EpisodeEvaluationResult *result)
{
    if (evaluator->fitness_function != TRAINING_FITNESS_CURRICULUM ||
        result->completed_lap || result->steps <= 0)
    {
        return;
    }

    const RacingEnv *environment = evaluator->environment;
    const RewardParameters *parameters = environment->reward_parameters;
    float maximum_speed = environment->parameters->max_speed_kmh / 3.6f;
    if (environment->track->total_length <= 0.0f || maximum_speed <= 0.0f)
        return;

    float target_length = evaluator->use_track_segment
        ? evaluator->track_segment_length
        : environment->track->total_length;
    if (target_length <= 0.0f)
        return;
    float progress_ratio = clamp01(
        result->forward_progress / target_length);
    float average_speed = result->speed_integral /
        (result->steps * GA_FIXED_STEP);
    float average_speed_ratio = fmaxf(0.0f, fminf(
        average_speed / maximum_speed,
        1.0f));
    float survival_factor = result->stationary
        ? 0.0f
        : result->left_track ? 0.25f : 1.0f;

    result->fitness += evaluator->curriculum_performance_blend *
        parameters->curriculum_incomplete_average_speed_reward *
        progress_ratio * progress_ratio *
        average_speed_ratio * survival_factor;
}

static void finalize_track_segment_completion(
    const EpisodeEvaluator *evaluator,
    EpisodeEvaluationResult *result)
{
    const RacingEnv *environment = evaluator->environment;
    const RewardParameters *parameters = environment->reward_parameters;
    float maximum_speed = environment->parameters->max_speed_kmh / 3.6f;
    float elapsed = result->steps * GA_FIXED_STEP;
    float average_speed = elapsed > 0.0f
        ? evaluator->track_segment_length / elapsed
        : 0.0f;
    float speed_ratio = maximum_speed > 0.0f
        ? clamp01(average_speed / maximum_speed)
        : 0.0f;
    float speed_reward = parameters->lap_speed_reward * speed_ratio;

    result->completed_lap = 1;
    result->circuit_time = elapsed;
    result->fitness += parameters->lap_completion_reward;
    result->fitness += evaluator->fitness_function ==
            TRAINING_FITNESS_CURRICULUM
        ? evaluator->curriculum_performance_blend * speed_reward
        : speed_reward;
}


void episode_evaluate(
    EpisodeEvaluator *evaluator,
    float start_s,
    int timing_start_line,
    EpisodeEvaluationResult *result)
{
    RacingEnv *environment = evaluator->environment;
    memset(result, 0, sizeof(*result));
    result->circuit_time = -1.0f;
    racing_env_reset_at(environment, start_s, timing_start_line);

    while (result->steps < GA_MAX_EPISODE_STEPS) {
        RacingObservation observation;
        RacingAction action;
        RacingStepResult step_result;
        racing_env_observe(environment, &observation);
        neural_policy_predict(
            evaluator->policy,
            &observation,
            environment->parameters,
            environment->controller_parameters,
            &action);
        racing_env_step(
            environment, &action, GA_FIXED_STEP, &step_result);
        result->steps++;

        if (evaluator->use_track_segment) {
            if (step_result.completed_lap) {
                step_result.reward -= step_result.terminal_reward;
                step_result.terminal_reward = 0.0f;
                step_result.completed_lap = 0;
                step_result.terminated =
                    step_result.left_track || step_result.stationary;
            }
            if (evaluator->track_segment_length > 0.0f) {
                float progress_scale =
                    display_track.total_length /
                    evaluator->track_segment_length;
                float original_progress_reward =
                    step_result.progress_reward;
                step_result.progress_reward *= progress_scale;
                step_result.reward += step_result.progress_reward -
                    original_progress_reward;
            }
        }

        result->speed_integral += display_car.v_x * GA_FIXED_STEP;
        result->top_speed = fmaxf(result->top_speed, display_car.v_x);
        result->forward_progress += step_result.track_progress;
        result->progress_reward += step_result.progress_reward;
        result->control_penalty +=
            step_result.time_penalty +
            step_result.pedal_conflict_penalty +
            step_result.steering_change_penalty +
            step_result.lateral_acceleration_penalty;
        if (evaluator->fitness_function == TRAINING_FITNESS_CURRICULUM) {
            result->control_penalty +=
                evaluator->curriculum_performance_blend *
                (step_result.longitudinal_jerk_penalty +
                 step_result.pedal_change_penalty +
                 step_result.curriculum_pedal_conflict_penalty);
        }
        result->fitness += selected_step_fitness(
            evaluator,
            &step_result);

        if (evaluator->use_track_segment && !step_result.terminated &&
            result->forward_progress >= evaluator->track_segment_length)
        {
            finalize_track_segment_completion(evaluator, result);
            return;
        }

        if (step_result.completed_lap) {
            result->completed_lap = 1;
            result->circuit_time = display_car.completed_circuit_time;
            finalize_episode_fitness(evaluator, result);
            return;
        }

        if (step_result.terminated) {
            result->left_track = step_result.left_track;
            result->stationary = step_result.stationary;
            finalize_episode_fitness(evaluator, result);
            return;
        }
    }
    finalize_episode_fitness(evaluator, result);
}

static void ga_candidate_evaluate(
    TrainingContext *training,
    GaWorkContext *context)
{
    RacingEnv *environment = &context->environment;
    float total_fitness = 0.0f;
    float total_progress_reward = 0.0f;
    float total_control_penalty = 0.0f;
    int start_finish_episode_steps = 0;
    float start_finish_speed_integral = 0.0f;
    float start_finish_top_speed = 0.0f;
    float start_finish_forward_progress = 0.0f;
    int completed_start_finish_lap = 0;
    int start_finish_left_track = 0;
    int start_finish_stationary = 0;
    /*
     * GA FITNESS CALCULATION
     * ----------------------
     * Full-lap training runs three independent episodes, starting at the
     * start/finish line, Sector 1, and Sector 2. Segment training runs one
     * episode from its configured GeoJSON start point to its end point.
     * An episode is capped at GA_MAX_EPISODE_STEPS fixed simulation steps.
     * Full-lap fitness is the arithmetic mean of its three episodes.
     *
     * Standard mode uses the original environment reward. Corner-exit mode
     * adds one bounded apex-to-exit event reward. Curriculum mode begins with
     * progress, completion, and safety only, then introduces time, completed-
     * lap speed, incomplete-lap average speed, corner exit, jerk, and pedal-
     * oscillation terms through one blend frozen for the whole generation.
     *
     * Average speed, top speed, track progress, and target completion are
     * collected only from the primary episode. The speed values and lap or
     * segment completion are shown in the side pane.
     * Sector-start telemetry is ignored, although all three fitness values
     * still affect selection. Telemetry does not add any extra fitness.
     */
    EpisodeEvaluator evaluator = {
        environment,
        &context->policy,
        training->fitness_function,
        training->curriculum_performance_blend,
        training->use_track_segment,
        training->track_segment_length};
    int episode_count = training->use_track_segment
        ? 1
        : GA_START_POSITION_COUNT;
    for (int start = 0; start < episode_count; start++) {
        float start_s = training->use_track_segment
            ? training->track_segment_start_s
            : 0.0f;
        if (!training->use_track_segment && start > 0) {
            start_s = display_track.has_sectors
                ? display_track.sector_s[start - 1]
                : display_track.total_length *
                  start / GA_START_POSITION_COUNT;
        }

        EpisodeEvaluationResult episode;
        episode_evaluate(
            &evaluator, start_s, start, &episode);

        total_fitness += episode.fitness;
        total_progress_reward += episode.progress_reward;
        total_control_penalty += episode.control_penalty;
        if (start == 0) {
            start_finish_episode_steps = episode.steps;
            start_finish_speed_integral = episode.speed_integral;
            start_finish_top_speed = episode.top_speed;
            start_finish_forward_progress = episode.forward_progress;
            completed_start_finish_lap = episode.completed_lap;
            start_finish_left_track = episode.left_track;
            start_finish_stationary = episode.stationary;
        }
    }

    context->fitness = total_fitness / episode_count;
    context->average_progress_reward =
        total_progress_reward / episode_count;
    context->average_control_penalty =
        total_control_penalty / episode_count;
    context->average_speed = start_finish_episode_steps > 0
        ? start_finish_speed_integral /
          (start_finish_episode_steps * GA_FIXED_STEP) * 3.6f
        : 0.0f;
    context->top_speed = start_finish_top_speed * 3.6f;
    float target_length = training->use_track_segment
        ? training->track_segment_length
        : display_track.total_length;
    context->track_progress = target_length > 0.0f
        ? clamp01(start_finish_forward_progress / target_length) * 100.0f
        : 0.0f;
    context->completed_start_finish_lap = completed_start_finish_lap;
    context->start_finish_left_track = start_finish_left_track;
    context->start_finish_stationary = start_finish_stationary;
}


VOID CALLBACK ga_batch_work_callback(
    PTP_CALLBACK_INSTANCE instance,
    PVOID context_pointer,
    PTP_WORK work)
{
    (void)instance;
    (void)work;
    GaBatchWorkContext *batch =
        (GaBatchWorkContext *)context_pointer;
    TrainingContext *training = batch->owner;

    int end_candidate = batch->first_candidate + batch->candidate_count;
    for (int candidate = batch->first_candidate;
         candidate < end_candidate;
         candidate++)
    {
        ga_candidate_evaluate(training, &ga_work_contexts[candidate]);
    }

    if (InterlockedIncrement(&ga_completed_work_count) ==
        ga_batch_work_count)
    {
        PostMessageA(
            training->completion_window,
            WM_GA_GENERATION_COMPLETE,
            (WPARAM)batch->batch_token,
            0);
    }
}


static DWORD configure_ga_process_cpu_sets(void)
{
    ULONG information_size = 0;
    GetSystemCpuSetInformation(
        NULL, 0, &information_size, GetCurrentProcess(), 0);
    if (information_size == 0)
        return 0;

    unsigned char *buffer =
        (unsigned char *)malloc(information_size);
    if (!buffer)
        return 0;
    if (!GetSystemCpuSetInformation(
            (PSYSTEM_CPU_SET_INFORMATION)buffer,
            information_size,
            &information_size,
            GetCurrentProcess(),
            0))
    {
        free(buffer);
        return 0;
    }

    DWORD available_count = 0;
    for (ULONG offset = 0; offset < information_size;) {
        PSYSTEM_CPU_SET_INFORMATION information =
            (PSYSTEM_CPU_SET_INFORMATION)(buffer + offset);
        if (information->Size == 0 ||
            offset + information->Size > information_size)
        {
            free(buffer);
            return 0;
        }
        if (information->Type == CpuSetInformation &&
            !information->CpuSet.Parked &&
            !information->CpuSet.RealTime &&
            (!information->CpuSet.Allocated ||
             information->CpuSet.AllocatedToTargetProcess))
        {
            available_count++;
        }
        offset += information->Size;
    }

    if (available_count <= GA_RESERVED_PROCESSOR_COUNT) {
        free(buffer);
        return 0;
    }

    DWORD allowed_count = available_count - GA_RESERVED_PROCESSOR_COUNT;
    ULONG *allowed_ids =
        (ULONG *)malloc(sizeof(*allowed_ids) * allowed_count);
    if (!allowed_ids) {
        free(buffer);
        return 0;
    }

    DWORD allowed = 0;
    for (ULONG offset = 0;
         offset < information_size && allowed < allowed_count;)
    {
        PSYSTEM_CPU_SET_INFORMATION information =
            (PSYSTEM_CPU_SET_INFORMATION)(buffer + offset);
        if (information->Type == CpuSetInformation &&
            !information->CpuSet.Parked &&
            !information->CpuSet.RealTime &&
            (!information->CpuSet.Allocated ||
             information->CpuSet.AllocatedToTargetProcess))
        {
            allowed_ids[allowed++] = information->CpuSet.Id;
        }
        offset += information->Size;
    }

    int configured = SetProcessDefaultCpuSets(
        GetCurrentProcess(), allowed_ids, allowed_count);
    free(allowed_ids);
    free(buffer);
    return configured ? allowed_count : 0;
}


int initialize_ga_thread_pool(TrainingContext *training)
{
    if (ga_thread_pool)
        return 1;

    ga_thread_pool = CreateThreadpool(NULL);
    if (!ga_thread_pool)
        return 0;

    DWORD cpu_set_processor_count = configure_ga_process_cpu_sets();
    DWORD worker_count = cpu_set_processor_count;
    if (worker_count == 0) {
        DWORD processor_count =
            GetActiveProcessorCount(ALL_PROCESSOR_GROUPS);
        if (processor_count < 1)
            processor_count = 1;
        worker_count = processor_count > GA_RESERVED_PROCESSOR_COUNT
            ? processor_count - GA_RESERVED_PROCESSOR_COUNT
            : 1;
    }
    if (worker_count > GA_POPULATION_SIZE)
        worker_count = GA_POPULATION_SIZE;
    SetThreadpoolThreadMaximum(ga_thread_pool, worker_count);
    SetThreadpoolThreadMinimum(ga_thread_pool, 1);

    InitializeThreadpoolEnvironment(&ga_callback_environment);
    ga_callback_environment_initialized = 1;
    SetThreadpoolCallbackPool(&ga_callback_environment, ga_thread_pool);

    ga_batch_work_count = (int)worker_count;
    for (int worker = 0; worker < ga_batch_work_count; worker++) {
        GaBatchWorkContext *batch = &ga_batch_work_contexts[worker];
        batch->first_candidate =
            worker * GA_POPULATION_SIZE / ga_batch_work_count;
        int end_candidate =
            (worker + 1) * GA_POPULATION_SIZE / ga_batch_work_count;
        batch->candidate_count = end_candidate - batch->first_candidate;
        batch->owner = training;
        batch->work = CreateThreadpoolWork(
            ga_batch_work_callback,
            batch,
            &ga_callback_environment);
        if (!batch->work) {
            for (int created = 0; created < worker; created++) {
                CloseThreadpoolWork(ga_batch_work_contexts[created].work);
                ga_batch_work_contexts[created].work = NULL;
            }
            ga_batch_work_count = 0;
            DestroyThreadpoolEnvironment(&ga_callback_environment);
            ga_callback_environment_initialized = 0;
            CloseThreadpool(ga_thread_pool);
            ga_thread_pool = NULL;
            return 0;
        }
    }
    return 1;
}


int submit_ga_generation(TrainingContext *training)
{
    if (ga_workers_active || !initialize_ga_thread_pool(training))
        return 0;

    ga_active_batch_token = InterlockedIncrement(&ga_next_batch_token);
    InterlockedExchange(&ga_completed_work_count, 0);
    ga_discard_active_batch = 0;
    ga_workers_active = 1;
    training->generation_started_tick = GetTickCount64();

    for (int candidate = 0; candidate < GA_POPULATION_SIZE; candidate++) {
        GaWorkContext *context = &ga_work_contexts[candidate];
        genome_to_network(
            ga_population[candidate].genes,
            &context->policy);
    }
    for (int worker = 0; worker < ga_batch_work_count; worker++) {
        GaBatchWorkContext *batch = &ga_batch_work_contexts[worker];
        batch->batch_token = ga_active_batch_token;
        SubmitThreadpoolWork(batch->work);
    }
    return 1;
}


void cleanup_ga_thread_pool(TrainingContext *training)
{
    for (int worker = 0; worker < ga_batch_work_count; worker++) {
        if (ga_batch_work_contexts[worker].work) {
            WaitForThreadpoolWorkCallbacks(
                ga_batch_work_contexts[worker].work,
                TRUE);
            CloseThreadpoolWork(ga_batch_work_contexts[worker].work);
            ga_batch_work_contexts[worker].work = NULL;
        }
    }
    ga_batch_work_count = 0;
    if (ga_callback_environment_initialized) {
        DestroyThreadpoolEnvironment(&ga_callback_environment);
        ga_callback_environment_initialized = 0;
    }
    if (ga_thread_pool) {
        CloseThreadpool(ga_thread_pool);
        ga_thread_pool = NULL;
    }
}


static void reset_es_candidate_metrics(Genome *candidate)
{
    candidate->fitness = -FLT_MAX;
    candidate->average_speed = 0.0f;
    candidate->top_speed = 0.0f;
    candidate->track_progress = 0.0f;
}

static void sample_mirrored_es_population(TrainingContext *training)
{
    const int pair_count = GA_POPULATION_SIZE / 2;
    for (int pair = 0; pair < pair_count; pair++) {
        Genome *positive = &ga_population[pair * 2];
        Genome *negative = &ga_population[pair * 2 + 1];
        for (int gene = 0; gene < NN_GENOME_COUNT; gene++) {
            float perturbation = network_random_gaussian(
                &training->random_state) * ES_PERTURBATION_STDDEV;
            positive->genes[gene] =
                training->es_mean[gene] + perturbation;
            negative->genes[gene] =
                training->es_mean[gene] - perturbation;
        }
        reset_es_candidate_metrics(positive);
        reset_es_candidate_metrics(negative);
    }
}

void initialize_ga_population(TrainingContext *training)
{
    memset(&ga_training, 0, sizeof(ga_training));
    training->curriculum_performance_blend = 0.0f;
    training->curriculum_progress_ema = 0.0f;
    training->curriculum_completion_ema = 0.0f;
    training->es_optimizer_step = 0;
    memset(training->es_adam_first_moment, 0,
           sizeof(training->es_adam_first_moment));
    memset(training->es_adam_second_moment, 0,
           sizeof(training->es_adam_second_moment));

    NeuralPolicy randomized_policy;
    const NeuralPolicy *seed_policy = &driving_policy;
    if (training->start_from_random_weights) {
        neural_policy_initialize(
            &randomized_policy,
            &training->random_state);
        seed_policy = &randomized_policy;
    }
    network_to_genome(seed_policy, training->es_mean);
    memcpy(ga_best_genome.genes,
           training->es_mean,
           sizeof(ga_best_genome.genes));
    sample_mirrored_es_population(training);

    ga_training.initialized = 1;
    ga_training.generation = 1;
    ga_training.best_fitness = -FLT_MAX;
    ga_training.reported_best_fitness = -FLT_MAX;
    ga_training.reported_average_fitness = 0.0f;
    ga_best_genome.fitness = -FLT_MAX;
}

static void evolve_es_population(TrainingContext *training)
{
    int ranked[GA_POPULATION_SIZE];
    float utility[GA_POPULATION_SIZE];
    for (int candidate = 0; candidate < GA_POPULATION_SIZE; candidate++)
        ranked[candidate] = candidate;
    for (int rank = 1; rank < GA_POPULATION_SIZE; rank++) {
        int candidate = ranked[rank];
        int insertion = rank;
        while (insertion > 0 &&
               ga_population[ranked[insertion - 1]].fitness >
                   ga_population[candidate].fitness)
        {
            ranked[insertion] = ranked[insertion - 1];
            insertion--;
        }
        ranked[insertion] = candidate;
    }
    for (int rank = 0; rank < GA_POPULATION_SIZE; rank++) {
        utility[ranked[rank]] =
            (float)rank / (GA_POPULATION_SIZE - 1) - 0.5f;
    }

    int best = ranked[GA_POPULATION_SIZE - 1];
    float fitness_sum = 0.0f;
    for (int candidate = 0; candidate < GA_POPULATION_SIZE; candidate++)
        fitness_sum += ga_population[candidate].fitness;
    float generation_best_fitness = ga_population[best].fitness;
    float generation_best_average_speed =
        ga_population[best].average_speed;
    float generation_best_top_speed =
        ga_population[best].top_speed;
    float generation_best_track_progress =
        ga_population[best].track_progress;

    training->es_optimizer_step++;
    float beta1_correction = 1.0f - powf(
        ES_ADAM_BETA1, (float)training->es_optimizer_step);
    float beta2_correction = 1.0f - powf(
        ES_ADAM_BETA2, (float)training->es_optimizer_step);
    const int pair_count = GA_POPULATION_SIZE / 2;
    for (int gene = 0; gene < NN_GENOME_COUNT; gene++) {
        float gradient = 0.0f;
        for (int pair = 0; pair < pair_count; pair++) {
            int positive = pair * 2;
            int negative = positive + 1;
            float epsilon =
                (ga_population[positive].genes[gene] -
                 ga_population[negative].genes[gene]) /
                (2.0f * ES_PERTURBATION_STDDEV);
            gradient +=
                (utility[positive] - utility[negative]) * epsilon;
        }
        gradient /= pair_count * ES_PERTURBATION_STDDEV;

        float *first_moment = &training->es_adam_first_moment[gene];
        float *second_moment = &training->es_adam_second_moment[gene];
        *first_moment = ES_ADAM_BETA1 * *first_moment +
            (1.0f - ES_ADAM_BETA1) * gradient;
        *second_moment = ES_ADAM_BETA2 * *second_moment +
            (1.0f - ES_ADAM_BETA2) * gradient * gradient;
        float corrected_first = *first_moment / beta1_correction;
        float corrected_second = *second_moment / beta2_correction;
        training->es_mean[gene] += ES_LEARNING_RATE * corrected_first /
            (sqrtf(corrected_second) + ES_ADAM_EPSILON);
    }

    ga_training.completed_generations = ga_training.generation;
    ga_training.reported_best_fitness = generation_best_fitness;
    ga_training.reported_average_fitness =
        fitness_sum / GA_POPULATION_SIZE;
    ga_training.reported_best_average_speed =
        generation_best_average_speed;
    ga_training.reported_best_top_speed =
        generation_best_top_speed;
    ga_training.reported_best_track_progress =
        generation_best_track_progress;
    ga_training.generation++;

    sample_mirrored_es_population(training);
}


static void update_curriculum_performance_blend(
    TrainingContext *training)
{
    float sorted_progress[GA_POPULATION_SIZE];
    for (int candidate = 0; candidate < GA_POPULATION_SIZE; candidate++) {
        sorted_progress[candidate] = fmaxf(0.0f, fminf(
            ga_population[candidate].track_progress,
            100.0f));
    }
    for (int index = 1; index < GA_POPULATION_SIZE; index++) {
        float value = sorted_progress[index];
        int insertion = index;
        while (insertion > 0 &&
               sorted_progress[insertion - 1] > value) {
            sorted_progress[insertion] = sorted_progress[insertion - 1];
            insertion--;
        }
        sorted_progress[insertion] = value;
    }

    float median_progress = 0.5f *
        (sorted_progress[GA_POPULATION_SIZE / 2 - 1] +
         sorted_progress[GA_POPULATION_SIZE / 2]);
    float completion_percentage =
        ga_training.reported_lap_completion_percentage;
    if (ga_training.completed_generations == 0) {
        training->curriculum_progress_ema = median_progress;
        training->curriculum_completion_ema = completion_percentage;
    } else {
        const float ema_rate = 0.20f;
        training->curriculum_progress_ema += ema_rate *
            (median_progress - training->curriculum_progress_ema);
        training->curriculum_completion_ema += ema_rate *
            (completion_percentage -
             training->curriculum_completion_ema);
    }

    float progress_readiness = fmaxf(0.0f, fminf(
        (training->curriculum_progress_ema - 70.0f) / 25.0f,
        1.0f));
    float completion_readiness = fmaxf(0.0f, fminf(
        (training->curriculum_completion_ema - 10.0f) / 60.0f,
        1.0f));
    float competence = fmaxf(progress_readiness, completion_readiness);
    float target_blend = competence * competence *
        (3.0f - 2.0f * competence);
    float blend_change = target_blend -
        training->curriculum_performance_blend;
    blend_change = fmaxf(-0.02f, fminf(blend_change, 0.05f));
    training->curriculum_performance_blend = fmaxf(0.0f, fminf(
        training->curriculum_performance_blend + blend_change,
        1.0f));

    ga_training.reported_median_track_progress = median_progress;
}


static void record_training_generation(TrainingContext *training)
{
    TrainingGenerationSample *sample =
        &ga_training.history[ga_training.history_next];
    sample->average_fitness = ga_training.reported_average_fitness;
    sample->average_speed = ga_training.reported_best_average_speed;
    sample->average_progress_reward =
        ga_training.reported_average_progress_reward;
    sample->average_control_penalty =
        ga_training.reported_average_control_penalty;
    sample->off_track_percentage =
        ga_training.reported_off_track_percentage;
    sample->lap_completion_percentage =
        ga_training.reported_lap_completion_percentage;
    sample->median_track_progress =
        ga_training.reported_median_track_progress;
    sample->performance_blend = training->curriculum_performance_blend;

    ga_training.history_next =
        (ga_training.history_next + 1) % TRAINING_TREND_HISTORY_COUNT;
    if (ga_training.history_count < TRAINING_TREND_HISTORY_COUNT)
        ga_training.history_count++;
}


int collect_completed_ga_generation(TrainingContext *training, LONG batch_token)
{
    if (!ga_workers_active || batch_token != ga_active_batch_token)
        return 0;

    ga_workers_active = 0;
    if (ga_discard_active_batch) {
        ga_discard_active_batch = 0;
        if (training_running)
            submit_ga_generation(training);
        return 0;
    }
    if (!training_running)
        return 0;

    ga_training.reported_generation_elapsed_seconds =
        training->generation_started_tick > 0
        ? (float)(GetTickCount64() - training->generation_started_tick) /
            1000.0f
        : 0.0f;

    int completed_lap_count = 0;
    int off_track_count = 0;
    int stationary_count = 0;
    int generation_best_candidate = 0;
    float progress_reward_sum = 0.0f;
    float control_penalty_sum = 0.0f;
    for (int candidate = 0; candidate < GA_POPULATION_SIZE; candidate++) {
        ga_population[candidate].fitness =
            ga_work_contexts[candidate].fitness;
        ga_population[candidate].average_speed =
            ga_work_contexts[candidate].average_speed;
        ga_population[candidate].top_speed =
            ga_work_contexts[candidate].top_speed;
        ga_population[candidate].track_progress =
            ga_work_contexts[candidate].track_progress;
        completed_lap_count +=
            ga_work_contexts[candidate].completed_start_finish_lap != 0;
        off_track_count +=
            ga_work_contexts[candidate].start_finish_left_track != 0;
        stationary_count +=
            ga_work_contexts[candidate].start_finish_stationary != 0;
        progress_reward_sum +=
            ga_work_contexts[candidate].average_progress_reward;
        control_penalty_sum +=
            ga_work_contexts[candidate].average_control_penalty;
        if (ga_population[candidate].fitness >
            ga_population[generation_best_candidate].fitness)
        {
            generation_best_candidate = candidate;
        }
        if (training->fitness_function != TRAINING_FITNESS_CURRICULUM &&
            ga_population[candidate].fitness > ga_training.best_fitness) {
            ga_training.best_fitness = ga_population[candidate].fitness;
            ga_best_genome = ga_population[candidate];
        }
    }
    if (training->fitness_function == TRAINING_FITNESS_CURRICULUM) {
        ga_best_genome = ga_population[generation_best_candidate];
        ga_training.best_fitness = ga_best_genome.fitness;
    }
    ga_training.reported_lap_completion_percentage =
        100.0f * completed_lap_count / GA_POPULATION_SIZE;
    ga_training.reported_off_track_percentage =
        100.0f * off_track_count / GA_POPULATION_SIZE;
    ga_training.reported_stationary_percentage =
        100.0f * stationary_count / GA_POPULATION_SIZE;
    ga_training.reported_average_progress_reward =
        progress_reward_sum / GA_POPULATION_SIZE;
    ga_training.reported_average_control_penalty =
        control_penalty_sum / GA_POPULATION_SIZE;

    if (training->fitness_function == TRAINING_FITNESS_CURRICULUM)
        update_curriculum_performance_blend(training);

    evolve_es_population(training);
    record_training_generation(training);
    return 1;
}


#undef display_car
#undef display_track
#undef driving_policy
#undef training_running
#undef ga_discard_active_batch
#undef ga_workers_active
#undef ga_active_batch_token
#undef ga_next_batch_token
#undef ga_completed_work_count
#undef ga_batch_work_count
#undef ga_batch_work_contexts
#undef ga_work_contexts
#undef ga_callback_environment_initialized
#undef ga_callback_environment
#undef ga_thread_pool
#undef ga_training
#undef ga_best_genome
#undef ga_population
