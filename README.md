# game_controller

Controller nodes that decide *what* command to send to the game, as opposed
to `game_bridge` which only knows *how* to send it. A controller subscribes
to the integrated classifier signal (`/integrated/raw`, `ros2neuro_msgs/NeuroControl`)
and publishes decided commands on `/game/command` (`std_msgs/String`,
`INPUT_A`/`INPUT_B`/`INPUT_C`/`INPUT_D`) -- the same topic `game_bridge`
subscribes to. See the top-level repo for how the two fit together.

> `/integrated/raw` isn't defined anywhere else in the workspace yet, so the
> choice of `ros2neuro_msgs/NeuroControl` (its `values[0]` field) as the
> message carrying the integrated probability is an assumption made here,
> not something pulled from an existing integrator node. If the real
> integrator ends up publishing something else, only `base_controller.py`
> needs to change.

## Design

- **`base_controller.py`** -- `BaseController(Node, ABC)`. Owns the
  subscription to `integrated_topic` and the publisher to `command_topic`
  (both ROS2 parameters, defaulting to `/integrated/raw` and `/game/command`).
  Subclasses implement `on_integrated(msg)` and call `self.send_command(cmd)`,
  which validates the command is one of the four valid strings before
  publishing.
- **`two_class_threshold_controller.py`** -- `TwoClassThresholdController`.
  Reads the integrated probability (`msg.values[0]`) and maps it to a command
  via four thresholds, splitting `[0, 1]` into five zones:

  | Probability range                     | Command          |
  | -------------------------------------- | ---------------- |
  | `< threshold_1`                        | `INPUT_A`         |
  | `[threshold_1, threshold_2)`           | *(nothing)*       |
  | `[threshold_2, threshold_3)`           | `INPUT_C` (forward) |
  | `[threshold_3, threshold_4)`           | *(nothing)*       |
  | `>= threshold_4`                       | `INPUT_B`         |

  Defaults: `threshold_1=0.3`, `threshold_2=0.4`, `threshold_3=0.6`,
  `threshold_4=0.7`. All four are ROS2 parameters, overridable from the
  launch file (see below) or any params YAML -- and adjustable **live**,
  ROS2's equivalent of ROS1's `dynamic_reconfigure`: no separate mechanism
  needed, any ROS2 node's parameters are already live-settable, so this one
  just needs the values re-read on every message (rather than cached once at
  startup) plus a validation callback that rejects any set that would break
  `threshold_1 < threshold_2 <= threshold_3 < threshold_4`. Two ways to
  change them at runtime:
  ```bash
  ros2 param set /two_class_threshold_controller threshold_1 0.25
  # or, GUI sliders:
  ros2 run rqt_reconfigure rqt_reconfigure
  ```
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

To start bridge + dummy together with a single command anyway (bridge in the
background, dummy in the foreground of the same terminal), use
[`scripts/run_dummy_session.sh`](../../scripts/run_dummy_session.sh) at the
repo root:

```bash
# from the repo root
source install/setup.bash
./scripts/run_dummy_session.sh          # target=local
./scripts/run_dummy_session.sh host     # target=host
```

## Running

```bash
# threshold controller alone, defaults
ros2 launch game_controller two_class_threshold.launch.py

# threshold controller alone, overriding thresholds
ros2 launch game_controller two_class_threshold.launch.py threshold_1:=0.25 threshold_4:=0.75

# threshold controller + game_bridge together (see game_bringup)
ros2 launch game_bringup bringup.launch.py

# dummy keyboard controller -- ros2 run only, see note above
ros2 run game_controller dummy_keyboard_controller
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

(Terminals 3+4 can be replaced by one: `./scripts/run_dummy_session.sh` from
the repo root, see above.)

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
