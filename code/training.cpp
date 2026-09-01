#include "racing.h"

#define ga_population (training->population)
#define ga_next_population (training->next_population)
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

    float progress_ratio = fmaxf(0.0f, fminf(
        result->forward_progress / environment->track->total_length,
        1.0f));
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
     * Each genome runs three independent episodes, starting at the
     * start/finish line, Sector 1, and Sector 2. An episode is capped at
     * GA_MAX_EPISODE_STEPS fixed simulation steps. The final genome fitness
     * is the arithmetic mean of the three episode fitness values.
     *
     * Standard mode uses the original environment reward. Corner-exit mode
     * adds one bounded apex-to-exit event reward. Curriculum mode begins with
     * progress, completion, and safety only, then introduces time, completed-
     * lap speed, incomplete-lap average speed, corner exit, jerk, and pedal-
     * oscillation terms through one blend frozen for the whole generation.
     *
     * Average speed, top speed, track progress, and lap completion are
     * collected only from the start/finish episode. The speed values and lap
     * completion are shown in the side pane. A genome counts as completing a
     * lap once.
     * Sector-start telemetry is ignored, although all three fitness values
     * still affect selection. Telemetry does not add any extra fitness.
     */
    EpisodeEvaluator evaluator = {
        environment,
        &context->policy,
        training->fitness_function,
        training->curriculum_performance_blend};
    for (int start = 0; start < GA_START_POSITION_COUNT; start++) {
        float start_s = 0.0f;
        if (start > 0) {
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

    context->fitness = total_fitness / GA_START_POSITION_COUNT;
    context->average_progress_reward =
        total_progress_reward / GA_START_POSITION_COUNT;
    context->average_control_penalty =
        total_control_penalty / GA_START_POSITION_COUNT;
    context->average_speed = start_finish_episode_steps > 0
        ? start_finish_speed_integral /
          (start_finish_episode_steps * GA_FIXED_STEP) * 3.6f
        : 0.0f;
    context->top_speed = start_finish_top_speed * 3.6f;
    context->track_progress = display_track.total_length > 0.0f
        ? start_finish_forward_progress /
          display_track.total_length * 100.0f
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


void initialize_ga_population(TrainingContext *training)
{
    memset(&ga_training, 0, sizeof(ga_training));
    training->curriculum_performance_blend = 0.0f;
    training->curriculum_progress_ema = 0.0f;
    training->curriculum_completion_ema = 0.0f;
    NeuralPolicy randomized_policy;
    const NeuralPolicy *seed_policy = &driving_policy;
    if (training->start_from_random_weights) {
        neural_policy_initialize(
            &randomized_policy,
            &training->random_state);
        seed_policy = &randomized_policy;
    }
    network_to_genome(seed_policy, ga_population[0].genes);
    ga_population[0].fitness = -FLT_MAX;
    ga_population[0].average_speed = 0.0f;
    ga_population[0].top_speed = 0.0f;
    ga_population[0].track_progress = 0.0f;
    memcpy(ga_best_genome.genes,
           ga_population[0].genes,
           sizeof(ga_best_genome.genes));

    for (int candidate = 1; candidate < GA_POPULATION_SIZE; candidate++) {
        for (int gene = 0; gene < NN_GENOME_COUNT; gene++) {
            ga_population[candidate].genes[gene] =
                ga_population[0].genes[gene] +
                network_random_gaussian(&training->random_state) * 0.15f;
        }
        ga_population[candidate].fitness = -FLT_MAX;
        ga_population[candidate].average_speed = 0.0f;
        ga_population[candidate].top_speed = 0.0f;
        ga_population[candidate].track_progress = 0.0f;
    }

    ga_training.initialized = 1;
    ga_training.generation = 1;
    ga_training.best_fitness = -FLT_MAX;
    ga_training.reported_best_fitness = -FLT_MAX;
    ga_training.reported_average_fitness = 0.0f;
    ga_best_genome.fitness = -FLT_MAX;
}


void evolve_ga_population(TrainingContext *training)
{
    int ranked[GA_POPULATION_SIZE];
    for (int candidate = 0; candidate < GA_POPULATION_SIZE; candidate++)
        ranked[candidate] = candidate;
    for (int rank = 0; rank < GA_ELITE_COUNT; rank++) {
        int best_rank = rank;
        for (int candidate_rank = rank + 1;
             candidate_rank < GA_POPULATION_SIZE;
             candidate_rank++)
        {
            if (ga_population[ranked[candidate_rank]].fitness >
                ga_population[ranked[best_rank]].fitness)
            {
                best_rank = candidate_rank;
            }
        }
        int swap = ranked[rank];
        ranked[rank] = ranked[best_rank];
        ranked[best_rank] = swap;
    }
    int best = ranked[0];

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

    if (training->fitness_function == TRAINING_FITNESS_CURRICULUM ||
        ga_population[best].fitness > ga_training.best_fitness)
        ga_training.best_fitness = ga_population[best].fitness;
    genome_to_network(ga_population[best].genes, &driving_policy);

    for (int elite = 0; elite < GA_ELITE_COUNT; elite++)
        ga_next_population[elite] = ga_population[ranked[elite]];

    /* Preserve coherent networks: every child starts as an exact copy of
       one elite, then explores locally through mutation. Distributing
       children round-robin gives each elite equal representation. */
    for (int child = GA_ELITE_COUNT; child < GA_POPULATION_SIZE; child++) {
        int elite = (child - GA_ELITE_COUNT) % GA_ELITE_COUNT;
        int parent = ranked[elite];
        ga_next_population[child] = ga_population[parent];
        for (int gene = 0; gene < NN_GENOME_COUNT; gene++) {
            if (network_random_unit(&training->random_state) <
                GA_MUTATION_PROBABILITY)
            {
                ga_next_population[child].genes[gene] +=
                    network_random_gaussian(&training->random_state) *
                    GA_MUTATION_SCALE;
            }
        }
        ga_next_population[child].fitness = -FLT_MAX;
        ga_next_population[child].average_speed = 0.0f;
        ga_next_population[child].top_speed = 0.0f;
        ga_next_population[child].track_progress = 0.0f;
    }

    for (int candidate = 0; candidate < GA_POPULATION_SIZE; candidate++)
        ga_population[candidate] = ga_next_population[candidate];

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

    evolve_ga_population(training);
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
#undef ga_next_population
#undef ga_population
