# game_controller

Controller nodes that decide *what* command to send to the game, as opposed
to `game_bridge` which only knows *how* to send it. A controller subscribes
to the integrated classifier signal (`/integrated/raw`, `ros2neuro_msgs/NeuroControl`)
and publishes decided commands on `/game/command` (`std_msgs/String`,
`INPUT_A`/`INPUT_B`/`INPUT_C`/`INPUT_D`) -- the same topic `game_bridge`
subscribes to. See the top-level repo for how the two fit together.

> `/integrated/raw` is `ros2neuro_msgs/NeuroControl` as published by
> `ros2neuro_integrator` with the `ros2neuro_integrator_buffer` plugin (the
> only integrator plugin in this workspace) -- `values[0]`/`values[1]` are
> two independent per-class accumulator buffers (one per MI class, each
> clipped to `[0, 1]`), not a single merged probability. All three
> controllers below that consume it (`two_class_threshold_controller`,
> `no_dead_zone_threshold_controller`, and `training_controller` in
> evaluation modality) derive a single `[0, 1]` position from them as
> `values[0] / (values[0] + values[1])` before applying their own
> thresholds -- see `_derive_position`/`derive_position` in each.
>
> `values[0]` (`class_a`) is the numerator **on purpose**: it has to be the
> class that drives the position towards `1`, because `1` is what the wheel
> (`SingleWheel::input2angle`) renders on the visual **left**, and
> `training_controller`'s `class2direction`/`is_target_hit` independently
> fix `classes[0]` (the first id in the `classes` launch arg) as
> `Direction::Left` (the blue threshold during calibration -- see
> `ros2neuro_feedback_wheel`'s `SingleWheel.cpp`, `Direction::Left` uses the
> `royalblue` line). So `values[0]` and `classes[0]` have to be *the same
> physical MI class* for calibration's "aim for blue" to mean the same
> thing as control's "rotate left" and evaluation's `Direction::Left` --
> this is a convention the classifier feeding `/integrated/raw` (not yet in
> this repo) has to uphold, nothing here can verify it at runtime.

## Design

This is a hybrid `ament_cmake_python` package (not pure `ament_python`) --
`two_class_threshold_controller`/`dummy_keyboard_controller` are still
plain Python nodes, but `training_controller` is C++, so the package needs
CMake to build it. Practically this only matters if you're changing the
build files: Python nodes are installed via CMake's `install(PROGRAMS ...
RENAME ...)` (see `CMakeLists.txt`) rather than `setup.py` entry_points, so
each one needs a `#!/usr/bin/env python3` shebang.

- **`base_controller.py`** -- `BaseController(Node, ABC)`. Owns the
  subscription to `integrated_topic` and the publisher to `command_topic`
  (both ROS2 parameters, defaulting to `/integrated/raw` and `/game/command`).
  Subclasses implement `on_integrated(msg)` and call `self.send_command(cmd)`,
  which validates the command is one of the four valid strings before
  publishing.
- **`two_class_threshold_controller.py`** -- `TwoClassThresholdController`.
  Derives a single `[0, 1]` position from the two per-class values (see note
  above) and maps it to a command via four thresholds, splitting `[0, 1]`
  into five zones:

  | Probability range                       | Command          |
  | ---------------------------------------- | ---------------- |
  | `< th_extreme_right`                     | `INPUT_B` (right)  |
  | `[th_extreme_right, th_right)`           | *(nothing)*       |
  | `[th_right, th_left)`                    | `INPUT_C` (forward) |
  | `[th_left, th_extreme_left)`             | *(nothing)*       |
  | `>= th_extreme_left`                     | `INPUT_A` (left)   |

  `class_a` (`values[0]`) dominant pushes the derived probability towards
  `1`, which the wheel (see `ros2neuro_feedback_wheel`'s README) renders on
  the visual **left** -- so it maps to `INPUT_A` ("rotate left" in
  Brainski2, see the game's `PROTOCOL.md`); `class_b` (`values[1]`)
  dominant pushes towards `0`, visual **right**, `INPUT_B` ("rotate
  right"). See the top-of-file note above for why `class_a` has to be
  `values[0]` (and hence `classes[0]`) specifically, not an arbitrary
  choice.

  Defaults: `th_extreme_right=0.3`, `th_right=0.4`, `th_left=0.6`,
  `th_extreme_left=0.7`. All four are ROS2 parameters, overridable from the
  launch file (see below) or any params YAML -- and adjustable **live**,
  ROS2's equivalent of ROS1's `dynamic_reconfigure`: no separate mechanism
  needed, any ROS2 node's parameters are already live-settable, so this one
  just needs the values re-read on every message (rather than cached once at
  startup) plus a validation callback that rejects any set that would break
  `th_extreme_right < th_right <= th_left < th_extreme_left`. Two ways to
  change them at runtime:
  ```bash
  ros2 param set /two_class_threshold_controller th_extreme_right 0.25
  # or, GUI sliders:
  ros2 run rqt_reconfigure rqt_reconfigure
  ```

  Two more parameters, independent of the thresholds above:

  - **`command_period_sec`** (default `0.5`) -- while the probability stays
    in the same command zone, that command is re-sent at most once per this
    period. This is a non-blocking cooldown (a timestamp comparison on every
    incoming message), not a `time.sleep`, so the node keeps processing
    messages and parameter changes during the wait. Crossing into a
    *different* command zone (e.g. `INPUT_C` -> `INPUT_A`) always sends
    immediately regardless of the cooldown, which then restarts from that
    new send. Passing through the dead zone and back into the *same* zone
    does **not** count as a zone change -- the cooldown started by the
    earlier send of that command keeps running.
  - **`with_reset`** (default `false`) -- when true, the controller watches
    for the integrated probability saturating exactly at `0.0` or `1.0` and,
    when it does, calls the integrator's `reset` service
    (`std_srvs/srv/Empty`, name configurable via `reset_service_name`,
    default `/integrator/reset`) so it drops back towards the neutral `0.5`
    zone instead of staying pinned at the extreme. The service call is used
    rather than a parameter set because it also makes the integrator publish
    a fresh control message for the reset, exactly as it does on a
    neuroevent-triggered reset. The call is fire-and-forget (async, no
    blocking); if the service isn't available yet the reset is skipped with
    a warning log rather than blocking `on_integrated`.

  It also relays every message it receives, unchanged, onto `control_topic`
  (param, default `/game_controller/control`) -- this is what the passive
  `wheel` node (package `ros2neuro_feedback_wheel`, see its README)
  subscribes to for visualization. The wheel never reads `/integrated/raw`
  directly; this node is the single place deciding what it shows.

  Launched via `controlWithDeathZone.launch.py`.
- **`no_dead_zone_threshold_controller.py`** -- `NoDeadZoneThresholdController`.
  Same parameters, same `_derive_position`, and the same `command_period_sec`
  cooldown/relay-onto-`control_topic`/`with_reset` behaviour as
  `two_class_threshold_controller`, but two differences in how the four
  thresholds are used:

  | Probability range     | Command   |
  | ---------------------- | --------- |
  | `< th_right`           | `INPUT_B` (right) |
  | `[th_right, th_left)`  | `INPUT_C` (forward) |
  | `>= th_left`           | `INPUT_A` (left)  |

  - **No dead zone** -- `th_right`/`th_left` alone split `[0, 1]` into
    three commands with no gap, so a command is always sent (no `(nothing)`
    zone).
  - **Reset at the outer thresholds** -- when `with_reset` is true, the
    integrator `reset` service is called as soon as the probability reaches
    `th_extreme_right` or `th_extreme_left` (i.e. `probability <=
    th_extreme_right` or `probability >= th_extreme_left`), instead of
    waiting for the absolute `0.0`/`1.0` extremes.

  `th_extreme_right < th_right <= th_left < th_extreme_left` is still
  enforced by the same validation, since `th_extreme_right`/`th_extreme_left`
  still have to sit outside the `INPUT_B`/`INPUT_A` command zones for the
  reset to make sense.

  Launched via `controlNoDeadZone.launch.py`.
- **`training_controller`** (C++, `src/training_controller.cpp`) -- the
  calibration/evaluation orchestrator. Owns everything a training session
  needs: trial sequencing (`TrialSequence`), fake-feedback generation in
  `calibration` modality (`Autopilot`/`LinearPilot`/`SinePilot`), and
  hit/miss/timeout detection -- all ported from what used to be
  `ros2neuro_feedback_wheel`'s `TrainingWheel`, now living here instead
  since it's orchestration logic, not display logic. Draws nothing itself.

  Parameters: `modality` (`calibration`|`evaluation`), `classes` (2 or 3
  class ids, e.g. `[773, 771]` or `[773, 771, 783]` for Left/Right[/Forward]),
  `trials` (trial count per class, same length as `classes`), `thresholds`
  (exactly 2 values -- this node's own hit-detection thresholds, independent
  of `two_class_threshold_controller`'s 4-threshold zones, since the
  calibration/evaluation paradigm is a different one: symmetric Left/Right
  crossing, with `Forward` meaning "stayed centered until timeout" rather
  than a third threshold), `control_topic`/`event_topic`/`probability_topic`
  (same `control_topic` the wheel listens to; `probability_topic`, default
  `/integrated/raw`, is only *subscribed* to in `evaluation` modality), plus
  the `duration.*` trial-timing parameters (not exposed by
  `training.launch.py`, see below -- pass a params YAML directly to
  `ros2 run` to tune them).

  In `calibration` modality it generates the current position locally
  (autopilot) and publishes it on `control_topic`, standing in for the
  integrator. In `evaluation` modality it subscribes to `probability_topic`
  (the real integrator output, two per-class values -- same derivation as
  `two_class_threshold_controller`, see note above) and relays the derived
  position onward to `control_topic` instead -- in both modalities the wheel
  only ever reads `control_topic`.

  It never resets the integrator itself -- that's configured directly on
  the integrator node (`integrator.reset_event`, see
  `launchers_bci/evaluation_pipeline.launch.xml`) to fire on the `CFeedback`
  event below, so each trial's accumulator starts neutral right when
  continuous feedback begins, rather than carrying over bias from the
  previous trial.

  In `evaluation` modality it also recreates `probability_sub_` (the
  `probability_topic` subscription) at that same `CFeedback` moment, right
  after resetting `current_input_`. Reason: nothing spins the executor
  during `Start`/`Fixation`/cue (they're plain sleeps), so that
  subscription's ROS queue can build up a backlog of up to its depth (10)
  in stale messages -- e.g. still reflecting the *previous* trial's
  saturated buffer, from before the integrator reset above has even run.
  Resetting `current_input_`/`has_new_input_` only clears local state, not
  that backlog; the loop's first `spin_some()` would otherwise deliver
  those stale messages and feed `is_target_hit()` a leftover value instead
  of this trial's real starting point -- observed in practice as trials
  resolving in ~2ms instead of the hundreds of ms a real threshold crossing
  takes. Recreating the subscription discards the backlog outright.

  It publishes `/neuroevent` for `Start`, `Fixation`, cue-by-class-id,
  `CFeedback`, and the outcome (`Hit`/`Miss`, plus `<class id>+Command` when
  a real zone was reached, for the wheel to color the "boom" correctly) --
  the *same* event-id vocabulary the passive wheel (`mode: training`)
  reacts to, see `ros2neuro_feedback_wheel`'s `Wheel.h`. A bare `Miss` with
  no preceding `<class>+Command` in that trial means a timeout (no zone
  reached in time), shown by the wheel as a generic boom.

  Between the control loop ending (the wheel has just been told the value
  that put it on the threshold) and publishing those outcome events, it
  sleeps 20ms -- `control_topic` and `event_topic` are two independent
  topics with no cross-topic delivery-order guarantee, and without this
  margin they can be published close enough together that the wheel's
  boom appears before the wheel has visibly reached the threshold. Once the
  boom hides (`outcome + Off`), it explicitly publishes `0.5` on
  `control_topic` so the wheel's center line returns to its starting
  position exactly as it would for a real "undecided" classifier output --
  the wheel itself has no reset logic (see its README).
- **`dummy_keyboard_controller.py`** -- `DummyKeyboardController`. Ignores
  `/integrated/raw` (its `on_integrated` is a no-op) and instead reads arrow
  keys from the terminal: Left `-> INPUT_A`, Right `-> INPUT_B`, Up `-> INPUT_C`,
  Down `-> INPUT_D`. Exists to exercise `game_bridge` and the game itself
  without a real acquisition/classification pipeline running.

  **Must be started with `ros2 run`, not `ros2 launch`.** `ros2 launch` does
  not connect child processes to a real terminal on stdin, so the raw
  keypress reading (`termios`) fails there -- confirmed in practice, not just
  in theory (a mis-set-up run throws `termios.error: (25, 'Inappropriate
  ioctl for device')`; the node now catches that and raises a clear error
  telling you to use `ros2 run` instead). There is deliberately no launch
  file for this node; there's nothing to parametrize anyway (it always talks
  to the default `/game/command`).

## Launching: interactive vs. non-interactive nodes

This split is permanent, not a workaround for now:

- **Non-interactive nodes** (`two_class_threshold_controller`, `game_bridge`,
  and eventually the real acquisition/classification pipeline) never need a
  terminal, so they belong in launch files and can be combined into one. The
  **[`game_bringup`](../game_bringup)** package does exactly that:
  `ros2 launch game_bringup bringup.launch.py` starts the bridge and the
  threshold controller together, in one command.
- **`dummy_keyboard_controller`** reads the keyboard, so it can *never* go in
  a launch file (see note above) -- it always needs its own `ros2 run` in an
  interactive terminal, for as long as it exists. This is not specific to
  today's setup; it'll still be true after the dummy is replaced by a real
  controller (at which point you stop using it at all, rather than working
  around the limitation).

## Running

```bash
# threshold controller (with dead zone) alone, defaults
ros2 launch game_controller controlWithDeathZone.launch.py

# threshold controller (with dead zone) alone, overriding thresholds
ros2 launch game_controller controlWithDeathZone.launch.py th_extreme_right:=0.25 th_extreme_left:=0.75

# threshold controller (with dead zone) alone, overriding the send cooldown and enabling integrator reset
ros2 launch game_controller controlWithDeathZone.launch.py command_period_sec:=0.2 with_reset:=true

# no-dead-zone threshold controller alone, defaults (reset at th_extreme_right/th_extreme_left if with_reset:=true)
ros2 launch game_controller controlNoDeadZone.launch.py with_reset:=true

# threshold controller + game_bridge together (see game_bringup)
ros2 launch game_bringup bringup.launch.py

# dummy keyboard controller -- ros2 run only, see note above
ros2 run game_controller dummy_keyboard_controller

# calibration/evaluation training session, with the passive wheel attached
ros2 launch game_controller training.launch.py
ros2 launch game_controller training.launch.py modality:=evaluation classes:="[773, 771, 783]" trials:="[10, 10, 5]"
```

## Testing the dummy locally, end to end

Requires `game_bridge` and the `cortical-peaks-challenge-2026` game already
set up (see the top-level repo README). Four terminals:

```bash
# 1: game server
cd cortical-peaks-challenge-2026 && uv run just run server

# 2: spectator (optional but useful) -- in the window: CONNECT TO SERVER,
#    127.0.0.1 / 5000, CONNECT
cd cortical-peaks-challenge-2026 && uv run just run spectator

# 3: the bridge, pointed at the local server
source install/setup.bash
ros2 launch game_bridge bridge.launch.py target:=local

# 4: the dummy controller
source install/setup.bash
ros2 run game_controller dummy_keyboard_controller
```

In the server window's **CONNECTIONS** tab, pick a game for the connected
player -- it'll show up under whatever `display_name` is set in
`game_bridge`'s `config/local.yaml` (`"UniPD"` by default, change it freely).
Once the countdown ends, focus the dummy's terminal and use the arrow keys --
you should see the game react in the spectator window.

## Verifying game_bridge actually sends the right command

Three independent checks, from cheapest to most authoritative:

1. **What the controller published.** `ros2 topic echo /game/command` in a
   spare terminal -- confirms the controller side is emitting the command you
   expect for a given key/probability.
2. **What game_bridge sent on the wire.** Bump the bridge's log level to see
   the `Sent CMD <cmd> to <address>` line it logs for every forwarded command
   (note: `--log-level` is a `ros2 launch` option, not `--ros-args` -- that
   flag is only for `ros2 run`):
   ```bash
   ros2 launch game_bridge bridge.launch.py target:=local --log-level debug
   ```
   Compare this against what `/game/command` showed in step 1 -- they should
   match 1:1.
3. **What the server actually received and accepted.** The authoritative
   check, since it also confirms the command wasn't dropped (wrong session
   token, countdown still running, etc). Run the server with `--audit`:
   ```bash
   uv run just run server --audit
   ```
   and inspect the JSON Lines log it writes to `data/audit/` -- each accepted
   input is recorded there with its timestamp and content.
