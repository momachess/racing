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
    HWND completion_window)
{
    memset(training, 0, sizeof(*training));
    training->track = track;
    training->parameters = parameters;
    training->reward_parameters = reward_parameters;
    training->controller_parameters = controller_parameters;
    training->active_policy = active_policy;
    training->completion_window = completion_window;
    training->random_state = 0x6d2b79f5u;
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
        result->forward_progress += step_result.track_progress;
        result->progress_reward += step_result.progress_reward;
        result->control_penalty +=
            step_result.time_penalty +
            step_result.pedal_conflict_penalty +
            step_result.steering_change_penalty +
            step_result.lateral_acceleration_penalty;
        result->fitness += step_result.reward;

        if (step_result.completed_lap) {
            result->completed_lap = 1;
            result->circuit_time = display_car.completed_circuit_time;
            return;
        }

        if (step_result.terminated) {
            result->left_track = step_result.left_track;
            result->stationary = step_result.stationary;
            return;
        }
    }
}

VOID CALLBACK ga_candidate_work_callback(
    PTP_CALLBACK_INSTANCE instance,
    PVOID context_pointer,
    PTP_WORK work)
{
    (void)instance;
    (void)work;
    GaWorkContext *context = (GaWorkContext *)context_pointer;
    TrainingContext *training = context->owner;
    RacingEnv *environment = &context->environment;
    float total_fitness = 0.0f;
    float total_progress_reward = 0.0f;
    float total_control_penalty = 0.0f;
    int start_finish_episode_steps = 0;
    float start_finish_speed_integral = 0.0f;
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
     * Progress is normalized by track length and scaled so one forward
     * circuit contributes 10.0.
     * Small time, pedal-conflict, steering-change, and excess-lateral-load
     * penalties shape control quality. Leaving the track or remaining below
     * the stationary speed threshold terminates with a -1.0 reward. A valid
     * circuit adds 5.0 plus up to 5.0 according to average lap speed as a
     * fraction of the car's physical maximum speed. There are no training
     * phases or frozen reference times, so fitness stays comparable across
     * generations.
     *
     * Average speed, track progress, and lap completion are collected only
     * from the start/finish episode for side-pane telemetry. A genome counts
     * as completing a lap once.
     * Sector-start telemetry is ignored, although all three fitness values
     * still affect selection. Telemetry does not add any extra fitness.
     */
    EpisodeEvaluator evaluator = {
        environment,
        &context->policy};
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
    context->track_progress = display_track.total_length > 0.0f
        ? start_finish_forward_progress /
          display_track.total_length * 100.0f
        : 0.0f;
    context->completed_start_finish_lap = completed_start_finish_lap;
    context->start_finish_left_track = start_finish_left_track;
    context->start_finish_stationary = start_finish_stationary;

    if (InterlockedIncrement(&ga_completed_work_count) ==
        GA_POPULATION_SIZE)
    {
        PostMessageA(
            training->completion_window,
            WM_GA_GENERATION_COMPLETE,
            (WPARAM)context->batch_token,
            0);
    }
}


int initialize_ga_thread_pool(TrainingContext *training)
{
    if (ga_thread_pool)
        return 1;

    ga_thread_pool = CreateThreadpool(NULL);
    if (!ga_thread_pool)
        return 0;

    DWORD processor_count = GetActiveProcessorCount(ALL_PROCESSOR_GROUPS);
    if (processor_count < 1)
        processor_count = 1;
    SetThreadpoolThreadMaximum(ga_thread_pool, processor_count);
    SetThreadpoolThreadMinimum(ga_thread_pool, 1);

    InitializeThreadpoolEnvironment(&ga_callback_environment);
    ga_callback_environment_initialized = 1;
    SetThreadpoolCallbackPool(&ga_callback_environment, ga_thread_pool);

    for (int candidate = 0; candidate < GA_POPULATION_SIZE; candidate++) {
        ga_work_contexts[candidate].candidate = candidate;
        ga_work_contexts[candidate].work = CreateThreadpoolWork(
            ga_candidate_work_callback,
            &ga_work_contexts[candidate],
            &ga_callback_environment);
        if (!ga_work_contexts[candidate].work) {
            for (int created = 0; created < candidate; created++) {
                CloseThreadpoolWork(ga_work_contexts[created].work);
                ga_work_contexts[created].work = NULL;
            }
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

    for (int candidate = 0; candidate < GA_POPULATION_SIZE; candidate++) {
        GaWorkContext *context = &ga_work_contexts[candidate];
        context->batch_token = ga_active_batch_token;
        genome_to_network(
            ga_population[candidate].genes,
            &context->policy);
        SubmitThreadpoolWork(context->work);
    }
    return 1;
}


void cleanup_ga_thread_pool(TrainingContext *training)
{
    for (int candidate = 0; candidate < GA_POPULATION_SIZE; candidate++) {
        if (ga_work_contexts[candidate].work) {
            WaitForThreadpoolWorkCallbacks(
                ga_work_contexts[candidate].work,
                TRUE);
            CloseThreadpoolWork(ga_work_contexts[candidate].work);
            ga_work_contexts[candidate].work = NULL;
        }
    }
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
    network_to_genome(&driving_policy, ga_population[0].genes);
    ga_population[0].fitness = -FLT_MAX;
    ga_population[0].average_speed = 0.0f;
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
        ga_population[candidate].track_progress = 0.0f;
    }

    ga_training.initialized = 1;
    ga_training.generation = 1;
    ga_training.best_fitness = -FLT_MAX;
    ga_training.reported_best_fitness = -FLT_MAX;
    ga_training.reported_average_fitness = 0.0f;
    ga_best_genome.fitness = -FLT_MAX;
}


int tournament_select(TrainingContext *training)
{
    int best = (int)(network_random_unit(&training->random_state) * GA_POPULATION_SIZE) %
        GA_POPULATION_SIZE;
    for (int round = 1; round < GA_TOURNAMENT_SIZE; round++) {
        int candidate = (int)(network_random_unit(&training->random_state) * GA_POPULATION_SIZE) %
            GA_POPULATION_SIZE;
        if (ga_population[candidate].fitness > ga_population[best].fitness)
            best = candidate;
    }
    return best;
}


void evolve_ga_population(TrainingContext *training)
{
    int best = 0;
    int second = 1;
    if (ga_population[second].fitness > ga_population[best].fitness) {
        int swap = best;
        best = second;
        second = swap;
    }
    for (int candidate = 2; candidate < GA_POPULATION_SIZE; candidate++) {
        if (ga_population[candidate].fitness > ga_population[best].fitness) {
            second = best;
            best = candidate;
        } else if (ga_population[candidate].fitness >
                   ga_population[second].fitness)
        {
            second = candidate;
        }
    }

    float fitness_sum = 0.0f;
    for (int candidate = 0; candidate < GA_POPULATION_SIZE; candidate++)
        fitness_sum += ga_population[candidate].fitness;
    float generation_best_fitness = ga_population[best].fitness;
    float generation_best_average_speed =
        ga_population[best].average_speed;
    float generation_best_track_progress =
        ga_population[best].track_progress;

    if (ga_population[best].fitness > ga_training.best_fitness)
        ga_training.best_fitness = ga_population[best].fitness;
    genome_to_network(ga_population[best].genes, &driving_policy);

    ga_next_population[0] = ga_population[best];
    ga_next_population[1] = ga_population[second];
    for (int child = GA_ELITE_COUNT; child < GA_POPULATION_SIZE; child++) {
        int parent1 = tournament_select(training);
        int parent2 = tournament_select(training);
        for (int gene = 0; gene < NN_GENOME_COUNT; gene++) {
            float value = network_random_unit(&training->random_state) < 0.5f
                ? ga_population[parent1].genes[gene]
                : ga_population[parent2].genes[gene];
            if (network_random_unit(&training->random_state) < GA_MUTATION_PROBABILITY)
                value += network_random_gaussian(&training->random_state) * GA_MUTATION_SCALE;
            ga_next_population[child].genes[gene] = value;
        }
        ga_next_population[child].fitness = -FLT_MAX;
        ga_next_population[child].average_speed = 0.0f;
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
    ga_training.reported_best_track_progress =
        generation_best_track_progress;
    ga_training.generation++;
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

    int completed_lap_count = 0;
    int off_track_count = 0;
    int stationary_count = 0;
    float progress_reward_sum = 0.0f;
    float control_penalty_sum = 0.0f;
    for (int candidate = 0; candidate < GA_POPULATION_SIZE; candidate++) {
        ga_population[candidate].fitness =
            ga_work_contexts[candidate].fitness;
        ga_population[candidate].average_speed =
            ga_work_contexts[candidate].average_speed;
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
        if (ga_population[candidate].fitness > ga_training.best_fitness) {
            ga_training.best_fitness = ga_population[candidate].fitness;
            ga_best_genome = ga_population[candidate];
        }
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
#undef ga_work_contexts
#undef ga_callback_environment_initialized
#undef ga_callback_environment
#undef ga_thread_pool
#undef ga_training
#undef ga_best_genome
#undef ga_next_population
#undef ga_population
