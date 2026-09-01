# Racing AI Project Roadmap

Complete the single-car learning foundation before introducing opponents.

## 1. Add controller-state observations

Include the following controller state in the neural-network observation:

- Filtered target speed
- Filtered target lateral offset
- Filtered target heading offset
- Normalized speed-integral state

These values currently affect the car but are hidden from the neural network. Adding them will increase the input count from 70 to 74 and require the saved-network format to advance to `RACENN7`.

## 2. Freeze the observation design and retrain

Avoid additional input changes during this phase. Train from fresh weights and evaluate:

- Lap-completion percentage
- Off-track percentage
- Best and average lap speed
- Apex consistency
- Lap-time variance

Target at least 80–90% lap completion before adding traffic.

## 3. Add persistent training metrics

Write one CSV row per generation containing:

- Best and average fitness
- Lap-completion percentage
- Off-track percentage
- Average and top speed
- Generation elapsed time
- Mutation probability and scale

Persistent metrics will show whether changes consistently improve learning instead of merely producing better-looking individual runs.

## 4. Add a separate evaluation mode

Evaluate the best network without mutation across fixed starting positions. Training fitness alone is noisy and can hide regressions.

## 5. Improve single-car race-time optimization

Once lap completion is reliable, emphasize:

- Faster lap completion
- Corner-exit speed
- Smooth braking and acceleration
- Smaller off-track and excessive-slip penalties

## 6. Introduce non-learning traffic

Start with one predictable scripted car. Add opponent observations for:

- Relative longitudinal position
- Relative lateral position
- Relative speed
- Closing speed
- Estimated time to collision

## 7. Use a traffic curriculum

Introduce traffic in increasing levels of difficulty:

1. Stationary obstacle
2. Slow car on the racing line
3. Moving car at constant speed
4. Multiple cars
5. Grid starts and complete races

## 8. Replace lap fitness with race objectives

Reward:

- Finishing position
- Progress relative to opponents
- Clean overtakes
- Race completion

Penalize:

- Collisions
- Contact while blocking
- Leaving the track

## Immediate next step

Implement controller-state observations before starting the next long training run. This closes the largest remaining observability gap while the observation format is still being revised.
