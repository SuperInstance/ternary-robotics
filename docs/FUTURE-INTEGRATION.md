# Future Integration: ternary-robotics

## Current State
Provides robotics control with ternary actuator commands (reverse/stop/forward), differential drive kinematics, ternary PID control, waypoint navigation on ternary grids, and obstacle avoidance with ternary proximity sensors.

## Integration Opportunities

### With ternary-swarm (Multi-Robot Coordination)
Multiple robots in a room need swarm coordination. `ternary-robotics` provides individual robot control; `ternary-swarm` provides the collective behavior. `TernaryActuator` commands for each robot are determined by swarm rules: separation (don't collide), alignment (move in the same direction), cohesion (stay in formation).

### With ternary-geometry (Spatial Navigation)
Robot navigation needs spatial reasoning. `TernaryPoint` for waypoints, `manhattan_distance` for path cost estimation, Voronoi diagrams for coverage planning — each robot covers its Voronoi cell. `lee_distance` handles wrapping environments (robots that exit one edge enter from the opposite).

### With ternary-hardware (Physical Deployment)
On Jetson/ESP32, ternary robotics compiles to real hardware. `TernaryActuator::apply_to_speed()` maps to PWM motor signals. `DifferentialDrive::compute_velocities()` feeds into odometry. The ternary PID controller outputs {-1, 0, +1} correction signals — simple enough for microcontroller implementation.

## Potential in Mature Systems
In room-as-codespace, physical robots (Jetson-powered) navigate real spaces that correspond to PLATO rooms. A robot in the "engine monitoring" room moves physically in the engine bay while its digital twin exists in the Codespace. Ternary actuator commands bridge the digital-physical divide: the Codespace decides {-1, 0, +1}, the robot executes.

## Cross-Pollination Ideas
- Ternary PID as a universal controller — works for temperature, speed, light, any measurable quantity
- Waypoint navigation on ternary grids as room-to-room navigation for physical robots
- Obstacle avoidance with ternary proximity sensors for safe human-robot interaction

## Dependencies for Next Steps
- Integration with ternary-swarm for multi-robot coordination
- Integration with ternary-geometry for spatial planning
- Jetson deployment testing with real actuators
