# Racing reinforcement-learning implementation plan

Status: Ready for execution  
Target project: `C:\Users\Massimo\Documents\Playground\racing`  
Companion design: `output/pdf/racing_rl_ppo_sac_refactor_design.pdf`  
Algorithms: PPO and SAC  
Primary library: RLtools, pinned revision  
Acceleration: optional NVIDIA CUDA, with a required CPU fallback

## 1. Objective

Refactor the current C++20 Win32 racing simulation from a genetic-algorithm trainer into an architecture that:

- trains continuous-control policies with both PPO and SAC;
- uses one algorithm-neutral racing environment contract;
- supports headless, deterministic, batched simulation;
- trains neural networks on CPU or NVIDIA CUDA through RLtools;
- keeps the existing Direct2D viewer as an inference and telemetry client;
- saves versioned, reproducible checkpoints;
- retains the current GA path temporarily as a regression oracle.

The first usable milestone is a CPU PPO policy that learns measurable forward progress. The final milestone is a reproducible PPO-versus-SAC comparison with CUDA acceleration, deterministic evaluation, and viewer playback.

## 2. Execution rules

These rules apply throughout implementation.

- Preserve existing behavior until parity tests prove the extracted environment is equivalent.
- Keep unrelated user changes intact.
- Make CMake the canonical build definition. Do not hand-maintain CUDA dependencies in the existing `.vcxproj` as the long-term solution.
- Require x64 for RL and CUDA targets.
- Keep simulation/core headers free of Windows, Direct2D, RLtools, and CUDA types.
- Keep RLtools includes inside the adapter and trainer implementation boundary.
- Treat time-limit exhaustion as truncation, not terminal failure.
- Keep the final observation of every terminated or truncated transition.
- Use fixed-step simulation for training and evaluation.
- Do not tune rewards and algorithms simultaneously during correctness work.
- Record every run's resolved configuration, seed, source revision, RLtools revision, backend, and hardware.
- Do not remove the GA trainer until PPO parity, checkpoint loading, and viewer inference all pass.

## 3. Fixed design decisions

| Decision | Selected design |
|---|---|
| Language | C++20 |
| Build system | CMake generating Visual Studio projects |
| RL library | RLtools at a pinned commit or release |
| Algorithms | PPO first, then SAC |
| Observation v1 | Existing 64-float observation |
| Action v1 | Three normalized continuous high-level controller targets |
| Initial control rate | 60 Hz, `dt = 1/60 s` |
| Physics integration | Preserve `PHYSICS_MAX_STEP = 0.01 s` |
| Environment device | CPU initially |
| Neural-network device | CPU and optional CUDA |
| Evaluation action | Deterministic `tanh(mean)` |
| Checkpoint policy | Versioned bundle with manifest and normalization state |
| Default training interface | Headless CLI |
| UI responsibility | Playback, telemetry, checkpoint load; no optimizer ownership |

## 4. Current baseline

The current source consists of:

- `code/racing.h`: shared simulation, GA, rendering, and Win32 declarations;
- `code/simulation.cpp`: track parsing/geometry, observation, policy inference, controller, physics, reward, reset, and step;
- `code/training.cpp`: GA evaluation, Windows thread pool, selection, crossover, mutation, and metrics;
- `code/main.cpp`: Win32 application, GA controls, checkpoint I/O, and real-time stepping;
- `code/rendering.cpp`: Direct2D rendering and GA/run telemetry;
- `code/racing.vcxproj`: existing Visual Studio build;
- `data/gb-1948.geojson`: track;
- `data/best_network.bin`: legacy `RACENN4` GA policy.

Important current dimensions:

- observation: 64 floats;
- action: 3 high-level targets;
- policy: `64 -> 32 -> 16 -> 3` deterministic network;
- GA population: 64;
- legacy training step: `1/60 s`;
- maximum episode: 12,000 steps, or 200 seconds;
- training start locations: start/finish and two sectors.

## 5. Target source layout

```text
racing/
  CMakeLists.txt
  CMakePresets.json
  cmake/
    Dependencies.cmake
    RacingOptions.cmake
  code/
    core/
      math.h
      track.h
      track.cpp
      track_loader.h
      track_loader.cpp
    env/
      racing_types.h
      observation.h
      observation.cpp
      reward.h
      reward.cpp
      racing_env.h
      racing_env.cpp
      vector_racing_env.h
      vector_racing_env.cpp
    control/
      vehicle_controller.h
      vehicle_controller.cpp
    rl/
      rl_config.h
      rl_config.cpp
      training_metrics.h
      policy_runtime.h
      policy_runtime.cpp
      checkpoint.h
      checkpoint.cpp
      rltools_env_adapter.h
      ppo_trainer.h
      ppo_trainer.cpp
      sac_trainer.h
      sac_trainer.cpp
    app/
      application.h
      main_win32.cpp
      rendering.h
      rendering.cpp
    tools/
      train_main.cpp
      evaluate_main.cpp
      benchmark_main.cpp
    legacy/
      ga_policy.h
      ga_policy.cpp
      ga_training.h
      ga_training.cpp
  config/
    environment_v1.json
    reward_v1.json
    ppo_baseline.json
    sac_baseline.json
    evaluation_v1.json
  data/
    checkpoints/
    runs/
  tests/
    test_reset.cpp
    test_step_parity.cpp
    test_observation.cpp
    test_action_mapping.cpp
    test_reward.cpp
    test_termination.cpp
    test_checkpoint.cpp
    test_vector_env.cpp
    test_rl_math.cpp
```

The exact number of files may be reduced during implementation when two modules remain naturally cohesive. Dependency boundaries are more important than the literal file count.

## 6. Milestone overview

| ID | Milestone | Depends on | Exit result |
|---|---|---|---|
| M0 | Freeze baseline | None | Repeatable GA/environment reference data |
| M1 | CMake and test shell | M0 | Existing application builds through CMake |
| M2 | Extract headless environment | M1 | Windows-free fixed-step environment with parity tests |
| M3 | Vectorize environment | M2 | Deterministic batched CPU simulation and benchmark |
| M4 | Integrate RLtools CPU | M3 | Adapter and minimal learning smoke test |
| M5 | Implement PPO | M4 | PPO learns and checkpoints on CPU |
| M6 | Implement SAC | M4, M5 environment stability | SAC learns and checkpoints on CPU |
| M7 | Add CUDA | M5, M6 | PPO and SAC neural updates run on NVIDIA GPU |
| M8 | Refactor viewer | M5 checkpoint format | Viewer loads and runs PPO/SAC policies |
| M9 | Tune and compare | M7, M8 | Multi-seed evaluation report and selected deployment policy |
| M10 | Retire legacy GA | M9 | GA removed or isolated as an optional tool |

## 7. M0 - Freeze the current baseline

Goal: create evidence that allows later refactors to distinguish intentional changes from regressions.

### Tasks

- [ ] Record the current Release x64 build command and toolchain version.
- [ ] Record the current executable's behavior with `data/best_network.bin`.
- [ ] Add a baseline metadata file under `data/baselines/legacy_ga/`.
- [ ] Capture deterministic action traces for a fixed policy and fixed reset states.
- [ ] Capture state, observation, reward components, termination, and lap timing for fixed action sequences.
- [ ] Record random-policy, zero-action, and legacy-policy evaluation metrics.
- [ ] Copy or checksum the legacy `RACENN4` checkpoint.
- [ ] Record the exact reward defaults and vehicle/controller constants.
- [ ] Confirm whether existing uncommitted changes are present before editing overlapping files.

### Baseline trace record

Each trace row should contain:

```text
step, position_x, position_y, heading, velocity_x, velocity_y,
steering_angle, yaw_rate, throttle, brake, steering_command,
track_s, observation[64], action[3], reward_total,
reward_progress, reward_time, reward_pedal_conflict,
reward_steering_change, reward_lateral, reward_terminal,
terminated, truncated, left_track, stationary, completed_lap
```

### Acceptance criteria

- [ ] A baseline run can be regenerated from one documented command.
- [ ] Repeated same-seed traces are identical on the same build.
- [ ] Legacy checkpoint checksum and format metadata are stored.
- [ ] Baseline metrics are available for later PPO/SAC comparisons.

### Stop condition

Do not start structural extraction if the current simulation is not repeatable under fixed elapsed time and fixed inputs. Diagnose the nondeterminism first.

## 8. M1 - Introduce CMake and the test shell

Goal: create canonical x64 builds without changing application behavior.

### Files

- Create `CMakeLists.txt`.
- Create `CMakePresets.json`.
- Create `cmake/Dependencies.cmake`.
- Create `cmake/RacingOptions.cmake`.
- Create an initially empty `tests/` target.
- Leave `code/racing.vcxproj` available during migration.

### CMake options

```cmake
option(RACING_BUILD_VIEWER "Build the Win32 Direct2D viewer" ON)
option(RACING_BUILD_TRAINING "Build headless RL tools" ON)
option(RACING_BUILD_TESTS "Build unit and regression tests" ON)
option(RACING_ENABLE_CUDA "Enable CUDA RL backends" OFF)
option(RACING_ENABLE_GA_LEGACY "Build the legacy GA trainer" ON)
```

### Initial targets

- `racing`: existing Win32 application;
- `racing_core`: temporary static library target as files are extracted;
- `racing_tests`: test executable or CTest collection.

### Verification commands

```powershell
cmake -S . -B build-cmake -A x64 -DRACING_ENABLE_CUDA=OFF
cmake --build build-cmake --config Debug
cmake --build build-cmake --config Release
ctest --test-dir build-cmake -C Debug --output-on-failure
```

### Acceptance criteria

- [ ] Debug x64 and Release x64 viewer builds succeed.
- [ ] Existing track and checkpoint relative paths still resolve, or are replaced with executable/config-root resolution.
- [ ] Launch behavior matches the existing Visual Studio project.
- [ ] CMake emits clear errors for unsupported Win32 RL configurations.

### Rollback point

No source reorganization should be included in the first CMake change. If CMake fails, the original `.slnx/.vcxproj` path remains usable.

## 9. M2 - Extract the headless environment

Goal: separate deterministic simulation from Win32, rendering, GA, and neural-network implementation.

### 9.1 Split types

- [ ] Move `Vec2`, `Track`, and track geometry declarations to `core/`.
- [ ] Move `CarParameters`, `RewardParameters`, `VehicleControllerParameters`, `CarHistorySample`, and `CarState` to `env/racing_types.h`.
- [ ] Move `RacingObservation`, `RacingAction`, and `RacingStepResult` to environment headers.
- [ ] Remove `windows.h`, Direct2D, and DirectWrite includes from all core/environment headers.
- [ ] Keep Windows-only types in `app/` and `legacy/ga_training.*`.

### 9.2 Extract track functions

- [ ] Move vector math and centerline/boundary queries to `core/math.h` and `core/track.cpp`.
- [ ] Move GeoJSON loading to `core/track_loader.cpp`.
- [ ] Preserve start/finish and sector initialization behavior.
- [ ] Add track validation for point counts, length, sectors, and boundaries.

### 9.3 Extract controller

- [ ] Move `vehicle_controller_update` and its private helpers to `control/vehicle_controller.cpp`.
- [ ] Preserve filtering, PI speed control, lateral error, and heading error behavior.
- [ ] Make controller state explicitly environment-owned.
- [ ] Unit-test mapping at zero, minimum, and maximum targets.

### 9.4 Define the environment API

Target interface:

```cpp
struct EnvConfig {
    float physics_max_step = 0.01f;
    float control_step = 1.0f / 60.0f;
    int action_repeat = 1;
    int max_episode_steps = 12000;
};

struct ResetOptions {
    float track_s = 0.0f;
    int timing_start_line = 0;
    float initial_speed = 0.0f;
    float lateral_offset = 0.0f;
    float heading_offset = 0.0f;
};

struct StepResult {
    RacingObservation next_observation;
    float reward = 0.0f;
    bool terminated = false;
    bool truncated = false;
    RewardTerms terms{};
    EpisodeInfo info{};
};

class RacingEnv {
public:
    RacingObservation reset(uint64_t seed, const ResetOptions& options);
    StepResult step(const NormalizedAction& action);
    const CarState& state() const;
};
```

### 9.5 Normalize the action contract

Use policy actions in `[-1, 1]^3`:

```cpp
desired_speed = 0.5f * (action[0] + 1.0f) * maximum_speed;
desired_lateral_offset = action[1] * maximum_lateral_offset;
desired_heading_offset = action[2] * maximum_heading_offset;
```

- [ ] Clamp every normalized component to `[-1, 1]` at the environment boundary.
- [ ] Keep the physical `RacingAction` internal to the environment/controller boundary.
- [ ] Add action schema identifier `high-level-v1`.
- [ ] Do not add direct throttle/brake/steering control in this milestone.

### 9.6 Version the observation

- [ ] Preserve all 64 current features as `obs-v1`.
- [ ] Replace duplicated macro offsets with named index constants or a schema builder.
- [ ] Assert exactly 64 features are written.
- [ ] Assert finite output values.
- [ ] Keep environment encoding deterministic.
- [ ] Put running mean/variance normalization outside the environment.

Observation groups:

| Group | Count |
|---|---:|
| Instantaneous dynamics | 6 |
| Five history samples | 35 |
| Lidar | 7 |
| Four lookahead points | 16 |
| Total | 64 |

### 9.7 Correct episode semantics

- [ ] `left_track`, `stationary`, and `completed_lap` set `terminated = true`.
- [ ] Maximum episode length sets `truncated = true` and `terminated = false`.
- [ ] Preserve the final observation before resetting.
- [ ] Ensure exactly one terminal outcome receives precedence.
- [ ] Keep lap completion a successful termination in v1.

### 9.8 Separate simulation and rendering clocks

- [ ] Environment `step()` always advances `control_step * action_repeat`.
- [ ] Physics still subdivides using `physics_max_step`.
- [ ] Viewer rendering frequency no longer changes simulation results.
- [ ] Viewer may render fewer or interpolated frames without changing environment steps.

### Tests

- `test_reset.cpp`
- `test_step_parity.cpp`
- `test_observation.cpp`
- `test_action_mapping.cpp`
- `test_reward.cpp`
- `test_termination.cpp`

### Acceptance criteria

- [ ] `racing_core` and environment targets compile without Windows headers.
- [ ] Same-seed resets are identical.
- [ ] Fixed action traces match the M0 baseline within documented float tolerance.
- [ ] Reward total equals the sum of reward terms.
- [ ] Forward/backward start-line wrap progress is correct.
- [ ] Time limit produces truncation and retains the final observation.
- [ ] Viewer still runs through an adapter to the extracted environment.

## 10. M3 - Build the vectorized environment

Goal: collect transitions from many independent CPU environments efficiently and deterministically.

### API sketch

```cpp
class VectorRacingEnv {
public:
    VectorRacingEnv(size_t count, const SharedEnvData&, const EnvConfig&);
    void reset_all(uint64_t base_seed);
    BatchStepResult step(std::span<const NormalizedAction> actions);
    void reset_finished(std::span<const size_t> indices);
};
```

### Ownership

- Shared read-only: track geometry, vehicle parameters, reward parameters, controller parameters.
- Per environment: car state, controller state, history, RNG, episode counters, reset options.
- Per batch: observations, actions, rewards, flags, final observations, metrics.

### Tasks

- [ ] Implement independent deterministic RNG streams derived from a base seed and environment index.
- [ ] Use contiguous `[environment][feature]` observation storage.
- [ ] Use contiguous `[environment][action]` action storage.
- [ ] Reset completed environments immediately after their final transition is captured.
- [ ] Keep terminal and reset observations distinct.
- [ ] Parallelize stable environment ranges across worker threads.
- [ ] Avoid one callback allocation per step.
- [ ] Add single-thread mode for deterministic debugging.
- [ ] Add `racing_benchmark.exe`.

### Benchmark matrix

Measure 1, 8, 32, 64, 128, and 256 environments with:

- one worker;
- half logical processors;
- all logical processors;
- 60 Hz decisions;
- optional action repeat 2 after parity.

Record:

- environment steps/s;
- microseconds per environment step;
- observation-generation share;
- track-query share;
- worker synchronization share;
- memory use.

### Acceptance criteria

- [ ] Environment zero matches standalone RacingEnv for the same seed/actions.
- [ ] No state or RNG contamination occurs between environments.
- [ ] Threaded and single-threaded modes produce equivalent per-environment traces.
- [ ] 128 environments execute stably for at least one million aggregate steps.
- [ ] Throughput report identifies the current CPU bottleneck.

## 11. M4 - Integrate RLtools on CPU

Goal: establish the dependency and adapter boundary before implementing the full racing trainers.

### Dependency tasks

- [ ] Select and record an RLtools commit or release.
- [ ] Prefer a vendored submodule or CMake dependency with a pinned immutable revision.
- [ ] Record the RLtools license and attribution requirements.
- [ ] Add an offline/reproducible dependency path for later builds.
- [ ] Keep RLtools headers out of `core/`, `env/`, `control/`, and `app/`.

### Adapter tasks

- [ ] Define observation dimension 64 and action dimension 3.
- [ ] Map RLtools environment state to project-owned RacingEnv state.
- [ ] Map action matrices to normalized actions.
- [ ] Return rewards and termination state without losing truncation information.
- [ ] Implement final-observation handling required by the algorithm loop.
- [ ] Add compile-time checks for dimensions.

### Smoke test

Before the full track environment:

- [ ] Run an RLtools CPU example unchanged.
- [ ] Implement a tiny deterministic project-local environment.
- [ ] Confirm policy parameters change and reward improves.
- [ ] Confirm save/load yields identical deterministic actions.

### Acceptance criteria

- [ ] RLtools revision is pinned and printed at startup.
- [ ] CPU example and local smoke environment train successfully.
- [ ] Racing adapter compiles without exposing RLtools types to the UI.
- [ ] Build works with `RACING_ENABLE_CUDA=OFF`.

## 12. M5 - Implement PPO

Goal: produce a correct, reproducible PPO baseline before introducing replay-based training.

### 12.1 PPO model

Actor:

```text
64 normalized observations
-> 128 tanh
-> 128 tanh
-> 3 means
+ 3 trainable log standard deviations
```

Critic:

```text
64 normalized observations
-> 128 tanh
-> 128 tanh
-> 1 state value
```

Tasks:

- [ ] Use separate actor and critic trunks initially.
- [ ] Use a tanh-squashed Gaussian actor.
- [ ] Apply the tanh Jacobian correction to log probabilities.
- [ ] Use deterministic `tanh(mean)` for evaluation.
- [ ] Add finite-value checks for means, log standard deviations, actions, values, and losses.

### 12.2 Rollout storage

Store for every transition:

- observation;
- normalized action;
- reward;
- terminated;
- truncated;
- true final observation when an episode ends;
- old log probability;
- old value;
- advantage;
- return target;
- environment/episode identity for diagnostics.

### 12.3 GAE semantics

```text
delta_t = reward_t
          + gamma * bootstrap_mask * V(final_or_next_observation)
          - V(observation_t)

advantage_t = delta_t
              + gamma * lambda * continuation_mask * advantage_(t+1)
```

- `bootstrap_mask = 0` for true termination;
- `bootstrap_mask = 1` for truncation and ordinary continuation;
- recursion stops across every episode boundary;
- truncations bootstrap from the true final observation, not the reset observation.

### 12.4 PPO baseline configuration

```json
{
  "algorithm": "ppo",
  "parallel_environments": 128,
  "rollout_horizon": 256,
  "gamma": 0.995,
  "gae_lambda": 0.95,
  "clip_epsilon": 0.20,
  "learning_rate": 0.0003,
  "epochs": 10,
  "minibatch_size": 2048,
  "value_coefficient": 0.5,
  "entropy_coefficient": 0.003,
  "max_gradient_norm": 0.5,
  "target_kl": 0.02
}
```

### 12.5 PPO loss and diagnostics

- [ ] Normalize advantages over the full rollout batch.
- [ ] Shuffle indices before each optimization epoch.
- [ ] Implement clipped policy objective.
- [ ] Implement value loss and optional value clipping.
- [ ] Implement entropy bonus.
- [ ] Clip gradient norm.
- [ ] Stop epochs early when approximate KL exceeds target.
- [ ] Log policy loss, value loss, entropy, approximate KL, clip fraction, explained variance, action standard deviation, and gradient norm.

### 12.6 PPO math tests

- [ ] New/old policy ratio is exactly one before an update.
- [ ] Advantage normalization produces approximately zero mean and unit standard deviation.
- [ ] Clipped loss selects the correct branch for positive and negative advantages.
- [ ] GAE hand-calculated sequences match implementation.
- [ ] Termination and truncation produce different bootstrap behavior.
- [ ] Tanh log-probability correction matches a numeric reference.

### 12.7 PPO training gates

Run in this order:

1. tiny deterministic environment;
2. fixed racing start with short horizon;
3. three legacy starts;
4. randomized centerline starts;
5. full evaluation suite.

### PPO acceptance criteria

- [ ] CPU PPO learns the tiny environment.
- [ ] Training return and explained variance move in the expected direction.
- [ ] PPO exceeds random and zero-action racing baselines in at least 4 of 5 seeds.
- [ ] No NaN/Inf occurs in a one-million-step run.
- [ ] Checkpoint resume continues from the stored optimizer and step state.
- [ ] Inference-only checkpoint produces identical deterministic actions after reload.

## 13. M6 - Implement SAC

Goal: add a sample-efficient off-policy algorithm using the same environment, action, reward, checkpoint, and evaluation contracts.

### 13.1 SAC models

Actor:

```text
64 observations -> 256 ReLU -> 256 ReLU
-> 3 means + 3 log standard deviations
```

Critics:

```text
67 observation/action values -> 256 ReLU -> 256 ReLU -> scalar Q
```

Required networks:

- actor;
- critic Q1;
- critic Q2;
- target Q1;
- target Q2;
- automatic entropy temperature `alpha`.

### 13.2 Replay buffer

Initial capacity: 1,000,000 transitions.

Store:

- observation[64];
- action[3];
- reward;
- next/final observation[64];
- terminated;
- truncated;
- optional episode ID and insertion step for diagnostics.

Tasks:

- [ ] Use a preallocated ring buffer.
- [ ] Sample uniformly initially.
- [ ] Keep replay on host initially.
- [ ] Upload sampled contiguous batches to the selected neural-network device.
- [ ] Record memory allocation and actual bytes per transition.
- [ ] Defer prioritized replay until the baseline is stable.

### 13.3 SAC baseline configuration

```json
{
  "algorithm": "sac",
  "parallel_environments": 64,
  "replay_capacity": 1000000,
  "warmup_transitions": 20000,
  "batch_size": 1024,
  "gamma": 0.995,
  "target_tau": 0.005,
  "actor_learning_rate": 0.0003,
  "critic_learning_rate": 0.0003,
  "temperature_learning_rate": 0.0003,
  "automatic_temperature": true,
  "target_entropy": -3.0,
  "update_to_data_ratio": 1
}
```

### 13.4 SAC correctness tasks

- [ ] Clamp log standard deviation to a safe configured range such as `[-20, 2]`.
- [ ] Use reparameterized actor samples.
- [ ] Apply the tanh Jacobian correction.
- [ ] Use minimum of twin target critics.
- [ ] Prevent gradients from entering target networks.
- [ ] Polyak-update targets after critic optimization.
- [ ] Tune alpha automatically toward target entropy.
- [ ] Bootstrap across truncation, not true termination.
- [ ] Start updates only after replay warm-up.

### 13.5 SAC diagnostics

Log:

- critic Q1 and Q2 losses;
- actor loss;
- alpha loss and alpha value;
- policy entropy;
- Q means, standard deviations, and extrema;
- target Q distribution;
- replay fill and sampled transition age;
- action saturation;
- update-to-data ratio;
- gradient norms.

### 13.6 SAC math tests

- [ ] Target critics remain gradient-free.
- [ ] Twin critics use identical target values.
- [ ] Polyak update matches hand-calculated parameters.
- [ ] Automatic alpha update moves in the correct direction.
- [ ] Replay ring wrap preserves valid transition ordering and capacity.
- [ ] Same sampled batch produces repeatable CPU loss/updates.

### SAC acceptance criteria

- [ ] CPU SAC learns the tiny environment.
- [ ] SAC exceeds random and zero-action racing baselines in at least 4 of 5 seeds.
- [ ] Replay can fill, wrap, save, and resume without corruption.
- [ ] Q values remain finite during a one-million-step run.
- [ ] Inference-only checkpoint reproduces deterministic actor actions.

## 14. M7 - Add NVIDIA CUDA

Goal: accelerate neural-network inference and optimization while preserving the CPU environment and CPU fallback.

### 14.1 Build detection

- [ ] Detect CUDA Toolkit through CMake.
- [ ] Validate compatibility between MSVC, CUDA Toolkit, driver, and GPU architecture.
- [ ] Compile CUDA-specific trainer units only when enabled.
- [ ] Print GPU name, compute capability, driver/runtime versions, precision, and backend.
- [ ] Fail clearly when `device=cuda` is requested but unavailable.
- [ ] Continue to support `device=cpu` in CUDA-enabled builds.

### 14.2 Initial hybrid design

```text
CPU worker environments
    -> contiguous observation/action/transition batches
    -> RLtools CUDA networks and optimizers
    -> action batches returned to CPU environments
```

PPO:

- collect rollout on CPU;
- upload the complete rollout tensors once per update;
- retain tensors on GPU across minibatch epochs.

SAC:

- keep replay on host initially;
- sample contiguous batches;
- upload each training batch;
- keep networks, optimizers, targets, and alpha on GPU.

### 14.3 Optimization order

Only apply after measurement:

1. ordinary contiguous transfers;
2. page-locked staging buffers;
3. asynchronous transfers and streams;
4. larger batched inference intervals if semantically acceptable;
5. device replay buffer for SAC;
6. GPU environment rewrite only if CPU simulation is the proven dominant bottleneck.

### 14.4 CPU/GPU parity test

For a frozen batch and identical initial parameters:

- compare actor output;
- compare value/Q output;
- compare each loss;
- compare gradients;
- compare one optimizer update;
- document tolerances rather than requiring bitwise equality.

### 14.5 Performance report

Record separately:

- environment collection time;
- observation/action transfer time;
- inference time;
- loss and backward time;
- optimizer time;
- evaluation time;
- total environment steps/s;
- total wall-clock time to a fixed evaluation threshold.

### CUDA acceptance criteria

- [ ] PPO and SAC both complete training updates on the NVIDIA GPU.
- [ ] CPU/GPU frozen-batch parity is within documented tolerance.
- [ ] CUDA failures do not corrupt checkpoints.
- [ ] At the configured batch size, CUDA optimizer updates are faster than CPU.
- [ ] End-to-end bottlenecks are reported even if overall speedup is small.
- [ ] CPU-only build and tests still pass.

## 15. M8 - Checkpoints, configuration, telemetry, and viewer

### 15.1 Configuration

Create resolved JSON configurations for:

- environment;
- vehicle/controller;
- reward;
- PPO;
- SAC;
- evaluation;
- device and logging.

At startup:

- parse configuration;
- validate ranges and dimensions;
- resolve defaults;
- write the fully resolved configuration into the run directory;
- print a concise summary.

### 15.2 Checkpoint bundle

Suggested structure:

```text
checkpoint_000100000/
  manifest.json
  policy.bin
  training_state.bin
  normalization.bin
  config.json
  metrics.csv
```

`manifest.json` must include:

- format version;
- algorithm;
- observation schema and dimension;
- action schema and dimension;
- model topology;
- seed;
- environment steps and optimizer updates;
- source revision;
- RLtools revision;
- build type;
- CPU/CUDA backend and hardware description;
- file checksums where practical.

### 15.3 Save behavior

- [ ] Write to a temporary sibling directory/file.
- [ ] Flush and close every artifact.
- [ ] Validate the temporary checkpoint by reopening it.
- [ ] Atomically rename to the final checkpoint name.
- [ ] Keep `latest` and `best` pointers or copies recoverable.
- [ ] Never overwrite the only valid checkpoint in place.

### 15.4 Observation normalization

Store:

- sample count;
- mean[64];
- variance or second moment[64];
- epsilon and clipping limit;
- schema version.

Rules:

- update statistics during training collection;
- freeze statistics during evaluation and viewer playback;
- restore exactly on resume;
- reject schema/dimension mismatches.

### 15.5 Viewer refactor

- [ ] Replace GA-specific application state with algorithm-neutral `PolicyRuntime` and `TrainingSnapshot`.
- [ ] Load PPO or SAC actor weights through the same inference interface.
- [ ] Validate observation/action schemas before activation.
- [ ] Use deterministic actions.
- [ ] Run the environment at fixed 60 Hz independent of render timing.
- [ ] Show algorithm, backend, environment steps, evaluation return, lap completion, best median lap, and checkpoint name.
- [ ] Keep detailed PPO/SAC optimizer metrics in the headless logs; show only useful summaries in the UI.
- [ ] If live training display is desired, read immutable metrics/snapshots from the trainer process or a safe queue.

### Acceptance criteria

- [ ] Checkpoint round-trip preserves deterministic actions and normalization state.
- [ ] Resume restores optimizer, replay/rollout-relevant state, counters, and alpha where applicable.
- [ ] Viewer loads both PPO and SAC policy checkpoints.
- [ ] Viewer rejects incompatible or corrupted checkpoints with a clear message.
- [ ] Rendering frequency does not change physics outcomes.

## 16. M9 - Reward curriculum, tuning, and comparison

Do not start broad tuning before environment and algorithm correctness gates pass.

### 16.1 Reward v1

Initial components:

| Term | Initial setting |
|---|---:|
| Forward progress per lap | +10.0 |
| Time penalty | -0.01 per second |
| Steering-change weight | -0.002 |
| Excess lateral penalty | preserve current scale |
| Pedal conflict | preserve current scale |
| Off-track terminal | -2.0 |
| Stationary terminal | -1.0 |
| Lap completion | +2.0 initially |

Every component must be logged separately.

### 16.2 Curriculum

| Stage | Training distribution | Promotion gate |
|---|---|---|
| A - move | Fixed starts, easy initial orientation | >95% non-stationary in evaluation |
| B - remain | Fixed starts and sector reach | >80% reach the next sector |
| C - finish | Three legacy starts | >60% full-lap completion |
| D - generalize | Random track position and small offsets | Stable hold-out completion |
| E - race | Full task and lap-time optimization | Median lap improves without tail collapse |

### 16.3 Domain randomization

Add gradually and only after fixed-task learning:

- initial track position;
- initial speed;
- lateral and heading offsets;
- mass;
- engine power;
- lateral acceleration limit;
- small observation noise.

Keep final evaluation distributions fixed and separate from training.

### 16.4 Fair algorithm comparison

PPO and SAC must use:

- the same observation/action schemas;
- the same reward profile;
- the same training start distribution;
- the same environment-step budget;
- the same evaluation cadence;
- the same fixed evaluation seeds and starts;
- the same deterministic evaluation behavior;
- reported wall-clock time and hardware.

Do not compare only the best seed or training return.

### 16.5 Evaluation suite

Minimum matrix:

- 10 fixed seeds;
- start/finish, sector 1, and sector 2;
- randomized-start hold-out set;
- deterministic actor;
- no observation-statistics updates;
- no exploration noise.

Report:

- return median and 10th/90th percentiles;
- lap completion rate with confidence interval;
- lap-time median and percentiles;
- sector reach;
- off-track rate;
- stationary rate;
- progress;
- action saturation and smoothness;
- training environment steps;
- wall-clock training time.

### Final learning acceptance criteria

- [ ] Both algorithms exceed random and zero-action baselines in at least 4 of 5 training seeds.
- [ ] At least one algorithm reaches 80% lap completion on the fixed-start suite.
- [ ] Hold-out completion drops by no more than 15 percentage points.
- [ ] Reported improvements are not solely caused by reward shaping; task metrics improve too.
- [ ] A deployment policy is selected using completion confidence and lap-time distribution.

## 17. M10 - Legacy GA disposition

Only execute after M9 acceptance.

Options:

1. retain `legacy/ga_*` behind `RACING_ENABLE_GA_LEGACY` as a benchmark;
2. move it to a standalone `racing_ga_legacy` tool;
3. remove it after archiving baseline data and format documentation.

Tasks:

- [ ] Confirm no viewer or checkpoint path depends on `Genome`, `NeuralPolicy`, or `TrainingContext`.
- [ ] Archive `RACENN4` format documentation and loader if historical policies matter.
- [ ] Remove GA fields from renderer telemetry.
- [ ] Remove `WM_GA_GENERATION_COMPLETE` from the main application.
- [ ] Remove obsolete population constants and flat-genome conversions from shared headers.
- [ ] Re-run the complete CPU/CUDA, PPO/SAC, viewer, and checkpoint test matrix.

## 18. Required test matrix

| Area | Debug CPU | Release CPU | Release CUDA |
|---|:---:|:---:|:---:|
| Reset and step determinism | Required | Required | N/A for CPU env |
| Observation/action schema | Required | Required | Required adapter parity |
| Reward and termination | Required | Required | Required batch parity |
| Vector environment | Required | Required | Required hybrid path |
| Checkpoint round trip | Required | Required | Required |
| PPO math | Required | Required | CPU/GPU parity |
| SAC math and replay | Required | Required | CPU/GPU parity |
| Viewer checkpoint load | Required | Required | Required when CUDA build exists |
| Long smoke run | Optional | Required | Required |

## 19. Verification command set

Expected commands after the targets exist:

```powershell
# Configure CPU
cmake --preset windows-x64-cpu

# Build and test CPU
cmake --build --preset windows-x64-cpu-release
ctest --preset windows-x64-cpu-release --output-on-failure

# Configure CUDA
cmake --preset windows-x64-cuda

# Build and test CUDA
cmake --build --preset windows-x64-cuda-release
ctest --preset windows-x64-cuda-release --output-on-failure

# Environment benchmark
.\build\windows-x64-cpu\Release\racing_benchmark.exe `
  --config config\environment_v1.json `
  --env-counts 1,8,32,64,128,256

# PPO smoke training
.\build\windows-x64-cpu\Release\racing_train.exe `
  --algo ppo `
  --device cpu `
  --config config\ppo_baseline.json `
  --seed 1 `
  --steps 100000

# SAC smoke training
.\build\windows-x64-cpu\Release\racing_train.exe `
  --algo sac `
  --device cpu `
  --config config\sac_baseline.json `
  --seed 1 `
  --steps 100000

# CUDA PPO
.\build\windows-x64-cuda\Release\racing_train.exe `
  --algo ppo `
  --device cuda `
  --config config\ppo_baseline.json `
  --seed 1

# CUDA SAC
.\build\windows-x64-cuda\Release\racing_train.exe `
  --algo sac `
  --device cuda `
  --config config\sac_baseline.json `
  --seed 1

# Frozen evaluation
.\build\windows-x64-cuda\Release\racing_evaluate.exe `
  --checkpoint data\checkpoints\best `
  --config config\evaluation_v1.json
```

Preset and executable paths may change during implementation. Keep this section synchronized with the actual CMake targets.

## 20. Telemetry schema

Every metrics row should include common fields:

```text
wall_time_seconds
algorithm
device
seed
environment_steps
optimizer_updates
episodes
training_return_mean
evaluation_return_mean
evaluation_completion_rate
evaluation_lap_time_median
off_track_rate
stationary_rate
progress_mean
environment_steps_per_second
```

PPO adds:

```text
policy_loss
value_loss
entropy
approximate_kl
clip_fraction
explained_variance
action_std_mean
gradient_norm
```

SAC adds:

```text
critic_q1_loss
critic_q2_loss
actor_loss
alpha_loss
alpha
entropy
q1_mean
q2_mean
target_q_mean
replay_size
sample_age_mean
update_to_data_ratio
```

Reward terms remain separate columns.

## 21. Known risks and decision triggers

| Risk | Trigger | Required response |
|---|---|---|
| RLtools API mismatch | Required PPO/SAC feature cannot map cleanly | Stop and document exact mismatch; adapt behind project boundary or pin a different verified revision |
| CUDA gives no wall-clock gain | Environment collection dominates profile | Keep hybrid CPU/GPU design; optimize environment before considering GPU physics |
| Reward hacking | Return improves while completion/progress degrades | Inspect traces and component metrics; revise one reward term at a time |
| Non-Markov observation | Critic error stays high across algorithms | Add explicit lateral/heading error or test recurrence as a controlled schema-v2 experiment |
| Action saturation | High fraction near +/-1 with oscillation | Check scaling/log-probability first, then tune ranges or entropy |
| PPO instability | KL/clip fraction spikes or entropy collapses | Reduce learning rate/epochs, validate advantage and log-probability math |
| SAC divergence | Q values grow without bound | Validate reward scale, bootstrap masks, target detachment, and alpha before tuning |
| Replay memory pressure | Allocation or paging harms performance | Pack records, reduce configurable capacity, measure before device replay |
| Checkpoint corruption | Interrupted save cannot resume | Enforce temporary write, validation, and atomic rename |
| UI race | Viewer reads mutable trainer state | Separate processes or publish immutable snapshots only |

## 22. Execution progress log

Update this section after each implementation session.

| Date | Milestone | Work completed | Verification | Next action |
|---|---|---|---|---|
| - | - | Plan created | Document review only | Begin M0 baseline |

## 23. Decision log

Record decisions that change this plan.

| ID | Date | Decision | Reason | Affected milestones |
|---|---|---|---|---|
| D-001 | 2026-09-04 | Preserve high-level three-target action for v1 | Minimizes simultaneous environment and control redesign | M2-M9 |
| D-002 | 2026-09-04 | Implement PPO before SAC | Simplifies initial rollout and environment validation | M5-M6 |
| D-003 | 2026-09-04 | Keep CPU simulation initially | Current scalar physics is branch-heavy; CUDA value must be measured | M3, M7 |
| D-004 | 2026-09-04 | Use CMake as canonical build | Needed for reproducible RLtools/CUDA integration | M1 onward |

## 24. Definition of done

The refactor is complete when all of the following are true:

- [ ] Core/environment code contains no Win32, Direct2D, RLtools, or CUDA dependency.
- [ ] Headless deterministic fixed-step training and evaluation executables exist.
- [ ] PPO and SAC are selectable through configuration/CLI and share one environment contract.
- [ ] Both algorithms train on CPU; both neural-network paths train on CUDA when enabled.
- [ ] Termination, truncation, final observation, PPO GAE, and SAC targets are tested.
- [ ] Versioned checkpoints support inference, resume, validation, and atomic saving.
- [ ] Observation-normalization state is saved, restored, and frozen for evaluation.
- [ ] Direct2D viewer loads and plays PPO and SAC policies deterministically.
- [ ] Fixed multi-seed evaluation compares PPO and SAC fairly.
- [ ] At least one policy reaches the agreed completion and robustness targets.
- [ ] Build, test, benchmark, train, evaluate, and viewer commands are documented and reproducible.
- [ ] Legacy GA is either cleanly optional or intentionally retired with its baseline archived.

