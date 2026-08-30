#include "racing.h"

#define app_window (application->window)
#define status_message (application->status_message)
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

void set_status_message(
    ApplicationContext *application,
    const char *text)
{
    snprintf(
        status_message,
        sizeof(status_message),
        "%s",
        text);

    if (app_window)
        InvalidateRect(app_window, NULL, FALSE);
}

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

int right_pane_command_is_disabled(int command)
{
    (void)command;
    return 0;
}


/* ================================================================
   VECTOR FUNCTIONS
   ================================================================ */

void toggle_ga_training(ApplicationContext *application)
{
    if (training_running) {
        training_running = 0;
        if (ga_training.initialized)
            genome_to_network(ga_best_genome.genes, &driving_policy);
        racing_env_reset(&application->environment);
        set_status_message(application, "Genetic training stopped; best network activated.");
        return;
    }

    animation_running = 0;
    training_running = 1;
    if (!ga_training.initialized)
        initialize_ga_population(&application->training);
    if (!ga_workers_active && !submit_ga_generation(&application->training)) {
        training_running = 0;
        set_status_message(application, "Could not start the Windows GA thread pool.");
        return;
    }
    set_status_message(application, "Parallel genetic training running.");
}


void save_network(ApplicationContext *application)
{
    FILE *file = fopen("../data/best_network.bin", "wb");
    if (!file) {
        set_status_message(application, "Could not open ../data/best_network.bin for writing.");
        return;
    }

    const char magic[8] = {'R', 'A', 'C', 'E', 'N', 'N', '4', '\0'};
    int gene_count = NN_GENOME_COUNT;
    float genes[NN_GENOME_COUNT];
    if (ga_best_genome.fitness > -FLT_MAX)
        memcpy(genes, ga_best_genome.genes, sizeof(genes));
    else
        network_to_genome(&driving_policy, genes);

    int ok = fwrite(magic, sizeof(magic), 1, file) == 1 &&
        fwrite(&gene_count, sizeof(gene_count), 1, file) == 1 &&
        fwrite(genes, sizeof(genes), 1, file) == 1;
    fclose(file);
    set_status_message(application, ok
        ? "Best neural network saved."
        : "Failed while saving the neural network.");
}


void load_network(ApplicationContext *application)
{
    FILE *file = fopen("../data/best_network.bin", "rb");
    if (!file) {
        set_status_message(application, "No saved neural network was found.");
        return;
    }

    char magic[8];
    int gene_count = 0;
    float genes[NN_GENOME_COUNT];
    int ok = fread(magic, sizeof(magic), 1, file) == 1 &&
        fread(&gene_count, sizeof(gene_count), 1, file) == 1 &&
        fread(genes, sizeof(genes), 1, file) == 1;
    fclose(file);
    const char expected[8] = {'R', 'A', 'C', 'E', 'N', 'N', '4', '\0'};
    if (!ok || memcmp(magic, expected, sizeof(magic)) != 0 ||
        gene_count != NN_GENOME_COUNT)
    {
        set_status_message(application, "Saved neural-network file is invalid.");
        return;
    }

    training_running = 0;
    animation_running = 0;
    if (ga_workers_active)
        ga_discard_active_batch = 1;
    genome_to_network(genes, &driving_policy);
    memcpy(ga_best_genome.genes, genes, sizeof(genes));
    ga_best_genome.fitness = 0.0f;
    memset(&ga_training, 0, sizeof(ga_training));
    racing_env_reset(&application->environment);
    set_status_message(application, "Neural network loaded.");
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
    set_status_message(application, 
        animation_running
        ? "Neural-network control running."
        : "Neural-network control stopped.");
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
        application->window = window;
    }
    if (!application)
        return DefWindowProcA(window, message, wparam, lparam);

    switch (message) {
    case WM_CREATE:
        app_window = window;
        return 0;

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
        default:
            break;
        }

        InvalidateRect(window, NULL, FALSE);
        return 0;
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

                InvalidateRect(window, NULL, FALSE);
            }
        }
        return 0;

    case WM_LBUTTONDOWN:
        {
            RECT client;
            GetClientRect(window, &client);

            if ((int)(short)LOWORD(lparam) <
                client.right / 2)
            {
                track_panning = 1;
                track_pan_start_x = (int)(short)LOWORD(lparam);
                track_pan_start_y = (int)(short)HIWORD(lparam);
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
            int previous_hover = command_hover;
            command_hover = 0;
            int command = right_pane_command_at(&client, x, y);
            if (command >= 0 && right_pane_command_is_disabled(command))
                command = -1;
            if (command >= 0)
                command_hover = command + 100;

            if (previous_hover != command_hover)
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
            if (command >= 0 && right_pane_command_is_disabled(command))
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
            render_direct2d(&application->renderer, window, &application->environment, &application->training, animation_running, status_message);
            if (training_running && !submit_ga_generation(&application->training)) {
                training_running = 0;
                set_status_message(application, 
                    "Could not submit the next GA generation.");
            }
        }
        return 0;

    case WM_PAINT:
    {
        PAINTSTRUCT paint;
        BeginPaint(window, &paint);
        render_direct2d(&application->renderer, window, &application->environment, &application->training, animation_running, status_message);
        EndPaint(window, &paint);
        return 0;
    }

    case WM_DESTROY:
        training_running = 0;
        cleanup_ga_thread_pool(&application->training);
        renderer_context_shutdown(&application->renderer);
        app_window = NULL;
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
    snprintf(
        status_message,
        sizeof(status_message),
        "%s",
        "Random untrained neural network ready.");

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
        window);

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
        int simulation_running = animation_running;

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
            if (step_result.left_track) {
                animation_running = 0;
                display_car.v_x = 0.0f;
                display_car.v_y = 0.0f;
                display_car.throttle = 0.0f;
                display_car.brake = 0.0f;
                display_car.steering_command = 0.0f;
                set_status_message(application, 
                    "Run stopped: the car left the track.");
            }
            render_direct2d(&application->renderer, window, &application->environment, &application->training, animation_running, status_message);

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
