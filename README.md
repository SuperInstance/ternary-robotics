# ternary-robotics

Robotics control with ternary actuator commands (reverse/stop/forward), differential drive kinematics, ternary sensor arrays, and BFS path planning on ternary obstacle grids.

## Why This Exists

Robotics control doesn't always need fine-grained analog outputs. Many tasks — wall-following, obstacle avoidance, binary approach/retreat — work fine with three states: go forward, stop, or reverse. This crate models actuators, sensors, and path planning as ternary decisions. The result is a control framework that's simpler to reason about, easier to compose, and naturally maps to {-1, 0, +1} for integration with ternary decision systems.

## Core Concepts

- **TernaryActuator** — Reverse (−1), Stop (0), Forward (+1). Can apply to a continuous speed value with a configurable delta.
- **DifferentialDrive** — Two ternary actuators (left and right wheel) driving a differential drive robot. Computes linear velocity `(v_left + v_right) / 2` and angular velocity `(v_right − v_left) / wheel_base`.
- **TernarySensor** — Reads a continuous value in a [min, max] range and classifies it as below mid-range (Reverse), near mid-range (Stop), or above mid-range (Forward). The deadband is 10% of the range on each side of the midpoint.
- **TernarySensorArray** — Multiple sensors read simultaneously. `majority()` returns the most common ternary classification across all sensors.
- **PathPlanner** — A grid where each cell is Free, Blocked, or Unknown. BFS pathfinding avoids Blocked cells and treats Unknown as passable (optimistic planning).
- **RobotState** — Tracks position (x, y), heading (theta), speed, and timestamp. Updates via differential drive odometry: `x += linear × cos(theta) × dt`, `y += linear × sin(theta) × dt`, `theta += angular × dt`.
- **TernaryObstacle** — Free (traversable), Blocked (impassable), Unknown (potentially blocked). This ternary classification allows planning under uncertainty.

## Quick Start

```toml
# Cargo.toml
[dependencies]
ternary-robotics = "0.1"
```

```rust
use ternary_robotics::*;

fn main() {
    // Differential drive
    let mut drive = DifferentialDrive::new(0.3, 0.05, 1.0);
    drive.set_left(TernaryActuator::Forward);
    drive.set_right(TernaryActuator::Forward);
    let (lin, ang) = drive.compute_velocities();
    println!("Driving forward: linear={}, angular={}", lin, ang);

    // Turn in place
    drive.set_left(TernaryActuator::Reverse);
    drive.set_right(TernaryActuator::Forward);
    let (_, ang) = drive.compute_velocities();
    println!("Turning: angular={}", ang);

    // Sensor array
    let mut sensors = TernarySensorArray::new(3, 0.0, 10.0);
    sensors.read_all(&[2.0, 5.0, 8.0]);
    println!("Majority: {:?}", sensors.majority());

    // Path planning
    let mut planner = PathPlanner::new(10, 10);
    planner.set_obstacle(GridPos::new(5, 5), TernaryObstacle::Blocked);
    if let Some(path) = planner.find_path(GridPos::new(0, 0), GridPos::new(9, 9)) {
        println!("Path length: {}", path.len());
    }
}
```

## API Overview

| Type | Description |
|------|-------------|
| `TernaryActuator` | Reverse/Stop/Forward with speed application |
| `DifferentialDrive` | Two-wheeled drive with velocity computation |
| `TernarySensor` | Continuous reading → ternary classification |
| `TernarySensorArray` | Multiple sensors with majority voting |
| `GridPos` | Integer (x, y) position on a grid |
| `TernaryObstacle` | Free/Blocked/Unknown cell classification |
| `PathPlanner` | BFS pathfinding on a ternary obstacle grid |
| `RobotState` | Position, heading, speed with odometry updates |

## How It Works

**Differential drive kinematics** are the standard unicycle-to-differential mapping. Left and right wheel speeds are `actuator_value × max_speed`. Linear velocity is the average; angular velocity is the difference divided by wheel base. When both actuators are Forward, the robot goes straight. When they differ, it turns. Reverse/Forward gives the tightest turn (spinning in place).

**Sensor classification** divides the [min, max] range at the midpoint with a 10% deadband. Readings within 10% of center classify as Stop (neutral). Below the deadband is Reverse; above is Forward. This hysteresis prevents rapid oscillation at the boundary.

**Path planning** uses BFS (breadth-first search) on a 4-connected grid. Blocked cells are impassable; Unknown cells are treated as traversable (optimistic assumption). BFS guarantees the shortest path in terms of cell count.

**Odometry** uses the Euler integration approximation: position updates use the new heading after the angular velocity is applied. This is less accurate than midpoint integration but simpler. For small dt values the error is negligible; for large dt or high angular velocity, consider a smaller timestep.

**Navigation** (`navigate_toward`) computes the angle to the target, compares it to the current heading, and returns Forward if within ±0.3 radians of the target direction, or Stop otherwise. This is a simple "point and shoot" controller — no proportional control, no obstacle avoidance built into the navigation decision.

## Known Limitations

- **BFS path planning is unweighted.** All moves cost the same. No diagonal movement, no terrain cost, no heuristic acceleration (A*).
- **No obstacle avoidance in navigation.** `navigate_toward` only considers heading alignment. It doesn't check for obstacles along the path.
- **Sensor deadband is hard-coded at 10%.** Not configurable per sensor. If your application needs different sensitivity, you'd need to adjust the range or post-process the classification.
- **Odometry uses Euler integration.** Accumulates drift over time, especially during turns. No correction mechanism.
- **No kinematic limits enforcement.** `apply_to_speed` clamps to [−1, 1] but doesn't enforce acceleration limits. Real robots have maximum acceleration and jerk constraints.

## Use Cases

- **Educational robotics** — A simple control framework for teaching differential drive, sensor fusion, and path planning without the complexity of continuous control.
- **Simulated swarm robots** — Each robot controlled by ternary commands, with sensor arrays providing ternary world perception. Simple to reason about and compose.
- **Ternary decision pipelines** — Bridge between higher-level ternary reasoning (voting, language model predictions) and physical actuation. A vote of For/Against/Abstain maps directly to Forward/Reverse/Stop.

## Ecosystem Context

Part of the SuperInstance ternary crate family. `ternary-robotics` sits at the physical interaction layer. It can be driven by decisions from `ternary-voting` or `ternary-language-model`, execute programs compiled with `ternary-compiler-v2`, and log sensor states to `ternary-database`. The `TernaryActuator` type uses the standard {-1, 0, +1} encoding.

## License

MIT
