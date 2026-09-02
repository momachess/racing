#include "racing.h"

#define display_track (application->track)
#define display_car (application->environment.car)
#define driving_policy (application->policy)
#define ga_population (application->training.population)
#define ga_best_genome (application->training.best_genome)
#define ga_training (application->training.metrics)
#define animation_running (application->animation_running)
#define training_running (application->training.running)
#define ga_workers_active (application->training.workers_active)
#define ga_discard_active_batch (application->training.discard_active_batch)
#define d2d_target (application->renderer.d2d_target)
#define track_zoom (application->renderer.track_zoom)
#define track_pan_x (application->renderer.track_pan_x)
#define track_pan_y (application->renderer.track_pan_y)
#define track_panning (application->renderer.track_panning)
#define track_pan_start_x (application->renderer.track_pan_start_x)
#define track_pan_start_y (application->renderer.track_pan_start_y)
#define track_pan_origin_x (application->renderer.track_pan_origin_x)
#define track_pan_origin_y (application->renderer.track_pan_origin_y)
#define command_hover (application->renderer.command_hover)
#define segment_selection_stage \
    (application->training.track_segment_selection_stage)
#define segment_hover_index \
    (application->training.track_segment_hover_geojson_index)

void format_lap_time(
    float seconds,
    char *text,
    size_t text_size)
{
    int total_milliseconds =
        (int)(seconds * 1000.0f + 0.5f);
    int minutes = total_milliseconds / 60000;
    int remaining = total_milliseconds % 60000;
    int whole_seconds = remaining / 1000;
    int milliseconds = remaining % 1000;

    snprintf(
        text,
        text_size,
        "%02d.%02d.%03d",
        minutes,
        whole_seconds,
        milliseconds);
}

int right_pane_command_at(
    const RECT *client,
    int x,
    int y)
{
    int pane_left = client->right - RIGHT_PANE_WIDTH;
    int group_width = RIGHT_PANE_BUTTON_WIDTH * 4 +
        RIGHT_PANE_BUTTON_GAP * 3;
    int group_left = pane_left +
        (RIGHT_PANE_WIDTH - group_width) / 2;
    int button_top = client->bottom - 60;

    int source_left = pane_left +
        (RIGHT_PANE_WIDTH - RIGHT_PANE_TRAIN_SOURCE_WIDTH) / 2;
    int source_top = button_top -
        RIGHT_PANE_TRAIN_SOURCE_GAP -
        RIGHT_PANE_TRAIN_SOURCE_HEIGHT;

    int scope_left = source_left;
    int scope_top = source_top -
        RIGHT_PANE_TRAIN_SOURCE_GAP -
        RIGHT_PANE_TRAIN_SOURCE_HEIGHT;

    int training_car_left = source_left;
    int training_car_top = scope_top -
        RIGHT_PANE_TRAIN_SOURCE_GAP -
        RIGHT_PANE_TRAIN_SOURCE_HEIGHT;

    int fitness_width = RIGHT_PANE_FITNESS_BUTTON_WIDTH * 3 +
        RIGHT_PANE_FITNESS_BUTTON_GAP * 2;
    int fitness_left = pane_left +
        (RIGHT_PANE_WIDTH - fitness_width) / 2;
    int fitness_top = training_car_top -
        RIGHT_PANE_TRAIN_SOURCE_GAP -
        RIGHT_PANE_FITNESS_BUTTON_HEIGHT;
    if (y >= fitness_top &&
        y < fitness_top + RIGHT_PANE_FITNESS_BUTTON_HEIGHT)
    {
        int fitness_offset = x - fitness_left;
        if (fitness_offset >= 0) {
            int fitness_column = fitness_offset /
                (RIGHT_PANE_FITNESS_BUTTON_WIDTH +
                 RIGHT_PANE_FITNESS_BUTTON_GAP);
            if (fitness_column >= 0 && fitness_column < 3 &&
                fitness_offset %
                    (RIGHT_PANE_FITNESS_BUTTON_WIDTH +
                     RIGHT_PANE_FITNESS_BUTTON_GAP) <
                    RIGHT_PANE_FITNESS_BUTTON_WIDTH)
            {
                return 5 + fitness_column;
            }
        }
    }

    if (x >= source_left &&
        x < source_left + RIGHT_PANE_TRAIN_SOURCE_WIDTH &&
        y >= source_top &&
        y < source_top + RIGHT_PANE_TRAIN_SOURCE_HEIGHT)
    {
        return 4;
    }

    if (x >= scope_left &&
        x < scope_left + RIGHT_PANE_TRAIN_SOURCE_WIDTH &&
        y >= scope_top &&
        y < scope_top + RIGHT_PANE_TRAIN_SOURCE_HEIGHT)
    {
        return 8;
    }

    if (x >= training_car_left &&
        x < training_car_left + RIGHT_PANE_TRAIN_SOURCE_WIDTH &&
        y >= training_car_top &&
        y < training_car_top + RIGHT_PANE_TRAIN_SOURCE_HEIGHT)
    {
        return 9;
    }

    if (y < button_top || y >= button_top + RIGHT_PANE_BUTTON_HEIGHT)
        return -1;

    int offset = x - group_left;
    if (offset < 0)
        return -1;

    int column = offset /
        (RIGHT_PANE_BUTTON_WIDTH + RIGHT_PANE_BUTTON_GAP);
    if (column < 0 || column >= 4 ||
        offset % (RIGHT_PANE_BUTTON_WIDTH + RIGHT_PANE_BUTTON_GAP) >=
            RIGHT_PANE_BUTTON_WIDTH)
    {
        return -1;
    }

    return column;
}

int right_pane_command_is_disabled(
    int command,
    int training_is_running,
    int workers_are_active)
{
    if (command >= 4 && command <= 8)
        return training_is_running || workers_are_active;
    if (command == 0 && !training_is_running && workers_are_active)
        return 1;
    return 0;
}


/* ================================================================
   VECTOR FUNCTIONS
   ================================================================ */

static void reset_training_preview(ApplicationContext *application)
{
    float start_s = application->training.use_track_segment
        ? application->training.track_segment_start_s
        : 0.0f;
    racing_env_reset_at(
        &application->environment,
        start_s,
        TIMING_LINE_START_FINISH);
    application->training.training_preview_forward_progress = 0.0f;
}

void toggle_ga_training(ApplicationContext *application)
{
    if (training_running) {
        training_running = 0;
        if (ga_training.initialized)
            genome_to_network(ga_best_genome.genes, &driving_policy);
        racing_env_reset(&application->environment);
        return;
    }

    animation_running = 0;
    training_running = 1;
    if (!ga_training.initialized)
        initialize_ga_population(&application->training);
    genome_to_network(ga_best_genome.genes, &driving_policy);
    reset_training_preview(application);
    if (!ga_workers_active && !submit_ga_generation(&application->training)) {
        training_running = 0;
        return;
    }
}


void save_network(ApplicationContext *application)
{
    FILE *file = fopen("../data/best_network.bin", "wb");
    if (!file)
        return;

    const char magic[8] = {'R', 'A', 'C', 'E', 'N', 'N', '6', '\0'};
    int gene_count = NN_GENOME_COUNT;
    float genes[NN_GENOME_COUNT];
    if (ga_best_genome.fitness > -FLT_MAX)
        memcpy(genes, ga_best_genome.genes, sizeof(genes));
    else
        network_to_genome(&driving_policy, genes);

    (void)(fwrite(magic, sizeof(magic), 1, file) == 1 &&
        fwrite(&gene_count, sizeof(gene_count), 1, file) == 1 &&
        fwrite(genes, sizeof(genes), 1, file) == 1);
    fclose(file);
}


void load_network(ApplicationContext *application)
{
    FILE *file = fopen("../data/best_network.bin", "rb");
    if (!file)
        return;

    char magic[8];
    int gene_count = 0;
    float genes[NN_GENOME_COUNT];
    int ok = fread(magic, sizeof(magic), 1, file) == 1 &&
        fread(&gene_count, sizeof(gene_count), 1, file) == 1 &&
        fread(genes, sizeof(genes), 1, file) == 1;
    fclose(file);
    const char expected[8] = {'R', 'A', 'C', 'E', 'N', 'N', '6', '\0'};
    if (!ok || memcmp(magic, expected, sizeof(magic)) != 0 ||
        gene_count != NN_GENOME_COUNT)
        return;

    training_running = 0;
    animation_running = 0;
    if (ga_workers_active)
        ga_discard_active_batch = 1;
    genome_to_network(genes, &driving_policy);
    memcpy(ga_best_genome.genes, genes, sizeof(genes));
    ga_best_genome.fitness = 0.0f;
    memset(&ga_training, 0, sizeof(ga_training));
    racing_env_reset(&application->environment);
}


void toggle_autonomous_car(ApplicationContext *application)
{
    if (training_running) {
        training_running = 0;
        if (ga_training.initialized)
            genome_to_network(ga_best_genome.genes, &driving_policy);
        racing_env_reset(&application->environment);
    }
    if (!animation_running && car_has_left_track(&application->environment))
        racing_env_reset(&application->environment);
    animation_running = !animation_running;
}


LRESULT CALLBACK window_procedure(
    HWND window,
    UINT message,
    WPARAM wparam,
    LPARAM lparam)
{
    ApplicationContext *application = (ApplicationContext *)GetWindowLongPtrA(
        window, GWLP_USERDATA);
    if (message == WM_NCCREATE) {
        CREATESTRUCTA *create = (CREATESTRUCTA *)lparam;
        application = (ApplicationContext *)create->lpCreateParams;
        SetWindowLongPtrA(
            window, GWLP_USERDATA, (LONG_PTR)application);
    }
    if (!application)
        return DefWindowProcA(window, message, wparam, lparam);

    switch (message) {
    case WM_SIZE:
        if (d2d_target) {
            d2d_target->Resize(
                D2D1::SizeU(
                    LOWORD(lparam),
                    HIWORD(lparam)));
        }
        return 0;

    case WM_COMMAND:
        if (HIWORD(wparam) != BN_CLICKED)
            return 0;

        switch (LOWORD(wparam)) {
        case 100:
            if (segment_selection_stage != 0)
                break;
            toggle_ga_training(application);
            break;
        case 101:
            toggle_autonomous_car(application);
            break;
        case 102:
            save_network(application);
            break;
        case 103:
            load_network(application);
            break;
        case 104:
            if (training_running || ga_workers_active)
                break;
            application->training.start_from_random_weights =
                !application->training.start_from_random_weights;
            ga_training.initialized = 0;
            break;
        case 105:
        case 106:
        case 107:
            if (training_running || ga_workers_active)
                break;
            {
                TrainingFitnessFunction selected =
                    (TrainingFitnessFunction)(LOWORD(wparam) - 105);
                if (application->training.fitness_function != selected) {
                    application->training.fitness_function = selected;
                    ga_training.initialized = 0;
                }
            }
            break;
        case 108:
            if (training_running || ga_workers_active)
                break;
            if (segment_selection_stage != 0) {
                segment_selection_stage = 0;
                segment_hover_index = -1;
            } else if (application->training.use_track_segment) {
                application->training.use_track_segment = 0;
                ga_training.initialized = 0;
            } else {
                animation_running = 0;
                segment_selection_stage = 1;
                segment_hover_index = -1;
            }
            break;
        case 109:
            application->training.render_car_during_training =
                !application->training.render_car_during_training;
            if (training_running &&
                application->training.render_car_during_training)
            {
                genome_to_network(ga_best_genome.genes, &driving_policy);
                reset_training_preview(application);
            }
            break;
        default:
            break;
        }

        InvalidateRect(window, NULL, FALSE);
        return 0;
    case WM_KEYDOWN:
        if (wparam == VK_ESCAPE && segment_selection_stage != 0) {
            segment_selection_stage = 0;
            segment_hover_index = -1;
            InvalidateRect(window, NULL, FALSE);
            return 0;
        }
        break;
    case WM_MOUSEWHEEL:
        {
            RECT client;
            GetClientRect(window, &client);
            POINT cursor = {
                (LONG)(short)LOWORD(lparam),
                (LONG)(short)HIWORD(lparam)};
            ScreenToClient(window, &cursor);
            int map_width = client.right - RIGHT_PANE_WIDTH;

            if (cursor.x >= 0 && cursor.x < map_width)
            {
                int delta = GET_WHEEL_DELTA_WPARAM(wparam);
                float factor = delta > 0 ? 1.15f : 0.87f;

                track_zoom *= factor;

                if (track_zoom < 0.25f)
                    track_zoom = 0.25f;
                if (track_zoom > 12.0f)
                    track_zoom = 12.0f;

                if (segment_selection_stage != 0) {
                    int point = track_geojson_point_near_screen(
                        &application->renderer,
                        &display_track,
                        &client,
                        cursor.x,
                        cursor.y,
                        TRACK_POINT_PICK_RADIUS);
                    segment_hover_index = point;
                }

                InvalidateRect(window, NULL, FALSE);
            }
        }
        return 0;

    case WM_LBUTTONDOWN:
        {
            RECT client;
            GetClientRect(window, &client);
            int x = (int)(short)LOWORD(lparam);
            int y = (int)(short)HIWORD(lparam);
            int map_width = client.right - RIGHT_PANE_WIDTH;

            if (segment_selection_stage != 0 && x >= 0 && x < map_width) {
                int geojson_index = track_geojson_point_near_screen(
                    &application->renderer,
                    &display_track,
                    &client,
                    x,
                    y,
                    TRACK_POINT_PICK_RADIUS);
                if (geojson_index >= 0) {
                    if (segment_selection_stage == 1) {
                        application->training
                            .track_segment_start_geojson_index =
                            geojson_index;
                        segment_selection_stage = 2;
                    } else if (geojson_index != application->training
                                   .track_segment_start_geojson_index &&
                               training_configure_track_segment(
                                   &application->training,
                                   application->training
                                       .track_segment_start_geojson_index,
                                   geojson_index))
                    {
                        application->training.use_track_segment = 1;
                        segment_selection_stage = 0;
                        segment_hover_index = -1;
                        ga_training.initialized = 0;
                    }
                    InvalidateRect(window, NULL, FALSE);
                }
                return 0;
            }

            if (x >= 0 && x < map_width)
            {
                track_panning = 1;
                track_pan_start_x = x;
                track_pan_start_y = y;
                track_pan_origin_x = track_pan_x;
                track_pan_origin_y = track_pan_y;
                SetCapture(window);
            }
        }
        return 0;

    case WM_MOUSEMOVE:
        {
            RECT client;
            GetClientRect(window, &client);
            int x = (int)(short)LOWORD(lparam);
            int y = (int)(short)HIWORD(lparam);
            int map_width = client.right - RIGHT_PANE_WIDTH;
            int previous_segment_hover = segment_hover_index;
            if (segment_selection_stage != 0 &&
                x >= 0 && x < map_width)
            {
                segment_hover_index = track_geojson_point_near_screen(
                    &application->renderer,
                    &display_track,
                    &client,
                    x,
                    y,
                    TRACK_POINT_PICK_RADIUS);
            } else if (segment_selection_stage != 0) {
                segment_hover_index = -1;
            }
            int previous_hover = command_hover;
            command_hover = 0;
            int command = right_pane_command_at(&client, x, y);
            if (command >= 0 && right_pane_command_is_disabled(
                    command, training_running, ga_workers_active))
                command = -1;
            if (command >= 0)
                command_hover = command + 100;

            if (previous_hover != command_hover ||
                previous_segment_hover != segment_hover_index)
                InvalidateRect(window, NULL, FALSE);
        }
        if (track_panning && (wparam & MK_LBUTTON)) {
            track_pan_x = track_pan_origin_x +
                (int)(short)LOWORD(lparam) - track_pan_start_x;
            track_pan_y = track_pan_origin_y +
                (int)(short)HIWORD(lparam) - track_pan_start_y;
            InvalidateRect(window, NULL, FALSE);
        }
        return 0;

    case WM_LBUTTONUP:
        track_panning = 0;
        ReleaseCapture();
        {
            RECT client;
            GetClientRect(window, &client);
            int x = (int)(short)LOWORD(lparam);
            int y = (int)(short)HIWORD(lparam);

            int command = right_pane_command_at(&client, x, y);
            if (command >= 0 && right_pane_command_is_disabled(
                    command, training_running, ga_workers_active))
                command = -1;
            if (command >= 0)
                SendMessageA(
                    window,
                    WM_COMMAND,
                    MAKEWPARAM(command + 100, BN_CLICKED),
                    0);
        }
        return 0;

    case WM_GA_GENERATION_COMPLETE:
        if (collect_completed_ga_generation(&application->training, (LONG)wparam)) {
            if (application->training.render_car_during_training) {
                /* Switch the live preview to the latest best controller
                   without resetting its physical state or lap progress. */
                genome_to_network(ga_best_genome.genes, &driving_policy);
            }
            render_direct2d(&application->renderer, window, &application->environment, &application->training, animation_running);
            if (training_running && !submit_ga_generation(&application->training)) {
                training_running = 0;
                InvalidateRect(window, NULL, FALSE);
            }
        }
        return 0;

    case WM_PAINT:
    {
        PAINTSTRUCT paint;
        BeginPaint(window, &paint);
        render_direct2d(&application->renderer, window, &application->environment, &application->training, animation_running);
        EndPaint(window, &paint);
        return 0;
    }

    case WM_DESTROY:
        training_running = 0;
        cleanup_ga_thread_pool(&application->training);
        renderer_context_shutdown(&application->renderer);
        PostQuitMessage(0);
        return 0;
    }

    return DefWindowProcA(
        window,
        message,
        wparam,
        lparam);
}


int WINAPI WinMain(
    HINSTANCE instance,
    HINSTANCE previous_instance,
    LPSTR command_line,
    int show_command)
{
    ApplicationContext *application = (ApplicationContext *)calloc(
        1, sizeof(*application));
    if (!application)
        return 1;

    (void)previous_instance;
    (void)command_line;

    car_parameters_initialize_default(&application->car_parameters);
    reward_parameters_initialize_default(&application->reward_parameters);
    vehicle_controller_parameters_initialize_default(
        &application->controller_parameters);
    renderer_context_initialize(&application->renderer);

    if (!load_track_geojson(
            &display_track,
            "../data/gb-1948.geojson"))
    {
        MessageBoxA(
            NULL,
            "Could not load ../data/gb-1948.geojson.",
            "Racing track viewer",
            MB_OK | MB_ICONERROR);
        renderer_context_shutdown(&application->renderer);
        free(application);
        return 1;
    }

    racing_env_initialize(
        &application->environment,
        &application->track,
        &application->car_parameters,
        &application->reward_parameters,
        &application->controller_parameters);

    const char *class_name =
        "RacingTrackViewerWindow";

    WNDCLASSA window_class;
    memset(&window_class, 0, sizeof(window_class));
    window_class.lpfnWndProc = window_procedure;
    window_class.hInstance = instance;
    window_class.lpszClassName = class_name;
    window_class.hCursor = LoadCursor(NULL, IDC_ARROW);
    window_class.hbrBackground =
        (HBRUSH)(COLOR_WINDOW + 1);

    if (!RegisterClassA(&window_class)) {
        renderer_context_shutdown(&application->renderer);
        free(application);
        return 1;
    }

    application->environment.random_state = network_random_seed();
    neural_policy_initialize(
        &application->policy,
        &application->environment.random_state);
    racing_env_reset(&application->environment);

    HWND window = CreateWindowExA(
        0,
        class_name,
        "Racing track viewer",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        1920,
        1080,
        NULL,
        NULL,
        instance,
        application);

    if (!window) {
        cleanup_ga_thread_pool(&application->training);
        renderer_context_shutdown(&application->renderer);
        free(application);
        return 1;
    }

    training_context_initialize(
        &application->training,
        &application->track,
        &application->car_parameters,
        &application->reward_parameters,
        &application->controller_parameters,
        &application->policy,
        window,
        application->environment.random_state);

    ShowWindow(window, show_command);
    UpdateWindow(window);

    LARGE_INTEGER performance_frequency;
    LARGE_INTEGER previous_counter;
    QueryPerformanceFrequency(&performance_frequency);
    QueryPerformanceCounter(&previous_counter);
    double frame_ticks = ANIMATION_INTERVAL *
        (double)performance_frequency.QuadPart;
    double next_frame_counter =
        (double)previous_counter.QuadPart + frame_ticks;

    MSG message = {};
    int was_simulation_running = 0;
    int quit = 0;

    while (!quit) {
        while (PeekMessageA(&message, NULL, 0, 0, PM_REMOVE)) {
            if (message.message == WM_QUIT) {
                quit = 1;
                break;
            }

            TranslateMessage(&message);
            DispatchMessageA(&message);
        }

        if (quit)
            break;

        LARGE_INTEGER current_counter;
        QueryPerformanceCounter(&current_counter);
        int training_preview_running = training_running &&
            application->training.render_car_during_training;
        int simulation_running = animation_running || training_preview_running;

        if (simulation_running && !was_simulation_running) {
            previous_counter = current_counter;
            next_frame_counter =
                (double)current_counter.QuadPart + frame_ticks;
        }

        if (simulation_running && was_simulation_running &&
            (double)current_counter.QuadPart >= next_frame_counter)
        {
            float elapsed = (float)(
                (double)(current_counter.QuadPart -
                         previous_counter.QuadPart) /
                (double)performance_frequency.QuadPart);
            if (elapsed > 0.1f)
                elapsed = 0.1f;

            previous_counter = current_counter;
            RacingObservation observation;
            RacingAction action;
            RacingStepResult step_result;
            racing_env_observe(&application->environment, &observation);
            neural_policy_predict(
                &application->policy,
                &observation,
                &application->car_parameters,
                &application->controller_parameters,
                &action);
            racing_env_step(
                &application->environment,
                &action,
                elapsed,
                &step_result);
            if (training_preview_running) {
                application->training.training_preview_forward_progress +=
                    step_result.track_progress;
                int completed_segment =
                    application->training.use_track_segment &&
                    application->training.training_preview_forward_progress >=
                        application->training.track_segment_length;
                int crashed = step_result.left_track ||
                    step_result.stationary;
                if (crashed || completed_segment) {
                    genome_to_network(
                        ga_best_genome.genes,
                        &driving_policy);
                    reset_training_preview(application);
                } else if (step_result.completed_lap) {
                    /* Adopt the latest best controller at the lap boundary,
                       preserving the car state so the preview continues into
                       the next lap without teleporting to start/finish. */
                    genome_to_network(
                        ga_best_genome.genes,
                        &driving_policy);
                }
            } else if (step_result.left_track) {
                animation_running = 0;
                display_car.v_x = 0.0f;
                display_car.v_y = 0.0f;
                display_car.throttle = 0.0f;
                display_car.brake = 0.0f;
                display_car.steering_command = 0.0f;
            }
            render_direct2d(&application->renderer, window, &application->environment, &application->training, animation_running);

            do {
                next_frame_counter += frame_ticks;
            } while (next_frame_counter <=
                     (double)current_counter.QuadPart);
        }

        was_simulation_running = simulation_running;

        DWORD wait_milliseconds = INFINITE;
        if (simulation_running) {
            LARGE_INTEGER wait_counter;
            QueryPerformanceCounter(&wait_counter);
            double remaining_ticks = next_frame_counter -
                (double)wait_counter.QuadPart;
            if (remaining_ticks <= 0.0) {
                wait_milliseconds = 0;
            } else {
                double remaining_milliseconds = remaining_ticks *
                    1000.0 / (double)performance_frequency.QuadPart;
                wait_milliseconds = (DWORD)ceil(
                    remaining_milliseconds);
            }
        }

        MsgWaitForMultipleObjectsEx(
            0,
            NULL,
            wait_milliseconds,
            QS_ALLINPUT,
            MWMO_INPUTAVAILABLE);
    }
    int exit_code = (int)message.wParam;
    free(application);
    return exit_code;
}
