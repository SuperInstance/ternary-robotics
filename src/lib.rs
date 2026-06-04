#![forbid(unsafe_code)]

//! Robotics control with ternary decisions.

/// Ternary actuator command: reverse (-1), stop (0), forward (+1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TernaryActuator {
    Reverse,
    Stop,
    Forward,
}

impl TernaryActuator {
    pub fn value(self) -> i8 {
        match self {
            TernaryActuator::Reverse => -1,
            TernaryActuator::Stop => 0,
            TernaryActuator::Forward => 1,
        }
    }

    pub fn from_value(v: i8) -> Option<Self> {
        match v {
            -1 => Some(TernaryActuator::Reverse),
            0 => Some(TernaryActuator::Stop),
            1 => Some(TernaryActuator::Forward),
            _ => None,
        }
    }

    pub fn apply_to_speed(self, current: f64, max_delta: f64) -> f64 {
        match self {
            TernaryActuator::Reverse => (current - max_delta).max(-1.0),
            TernaryActuator::Stop => current * 0.5, // decelerate
            TernaryActuator::Forward => (current + max_delta).min(1.0),
        }
    }
}

/// Differential drive with two ternary actuators.
pub struct DifferentialDrive {
    pub left: TernaryActuator,
    pub right: TernaryActuator,
    pub wheel_base: f64,
    pub wheel_radius: f64,
    pub max_speed: f64,
}

impl DifferentialDrive {
    pub fn new(wheel_base: f64, wheel_radius: f64, max_speed: f64) -> Self {
        Self {
            left: TernaryActuator::Stop,
            right: TernaryActuator::Stop,
            wheel_base,
            wheel_radius,
            max_speed,
        }
    }

    pub fn set_left(&mut self, cmd: TernaryActuator) {
        self.left = cmd;
    }

    pub fn set_right(&mut self, cmd: TernaryActuator) {
        self.right = cmd;
    }

    /// Compute linear and angular velocity from current actuator states.
    pub fn compute_velocities(&self) -> (f64, f64) {
        let v_left = self.left.value() as f64 * self.max_speed;
        let v_right = self.right.value() as f64 * self.max_speed;
        let linear = (v_left + v_right) / 2.0;
        let angular = (v_right - v_left) / self.wheel_base;
        (linear, angular)
    }

    /// Check if the robot is turning.
    pub fn is_turning(&self) -> bool {
        self.left != self.right
    }

    /// Check if the robot is stopped.
    pub fn is_stopped(&self) -> bool {
        self.left == TernaryActuator::Stop && self.right == TernaryActuator::Stop
    }
}

/// Ternary sensor reading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TernarySensor {
    pub id: u32,
    pub value: f64,
    pub ternary: TernaryActuator, // mapped: <threshold=Reverse, =threshold=Stop, >threshold=Forward
    pub min: f64,
    pub max: f64,
}

impl TernarySensor {
    pub fn new(id: u32, min: f64, max: f64) -> Self {
        Self {
            id,
            value: 0.0,
            ternary: TernaryActuator::Stop,
            min,
            max,
        }
    }

    pub fn read(&mut self, value: f64) {
        self.value = value.clamp(self.min, self.max);
        let mid = (self.min + self.max) / 2.0;
        let threshold = (self.max - self.min) * 0.1;
        if value < mid - threshold {
            self.ternary = TernaryActuator::Reverse;
        } else if value > mid + threshold {
            self.ternary = TernaryActuator::Forward;
        } else {
            self.ternary = TernaryActuator::Stop;
        }
    }

    pub fn normalized(&self) -> f64 {
        if self.max == self.min {
            0.0
        } else {
            (self.value - self.min) / (self.max - self.min)
        }
    }
}

/// Array of ternary sensors.
pub struct TernarySensorArray {
    pub sensors: Vec<TernarySensor>,
}

impl TernarySensorArray {
    pub fn new(count: usize, min: f64, max: f64) -> Self {
        Self {
            sensors: (0..count).map(|i| TernarySensor::new(i as u32, min, max)).collect(),
        }
    }

    pub fn read_all(&mut self, values: &[f64]) {
        for (i, &v) in values.iter().enumerate() {
            if i < self.sensors.len() {
                self.sensors[i].read(v);
            }
        }
    }

    pub fn majority(&self) -> TernaryActuator {
        let mut counts = [0usize; 3]; // Reverse, Stop, Forward
        for s in &self.sensors {
            match s.ternary {
                TernaryActuator::Reverse => counts[0] += 1,
                TernaryActuator::Stop => counts[1] += 1,
                TernaryActuator::Forward => counts[2] += 1,
            }
        }
        if counts[0] >= counts[1] && counts[0] >= counts[2] {
            TernaryActuator::Reverse
        } else if counts[2] >= counts[1] {
            TernaryActuator::Forward
        } else {
            TernaryActuator::Stop
        }
    }

    pub fn len(&self) -> usize {
        self.sensors.len()
    }

    pub fn is_empty(&self) -> bool {
        self.sensors.is_empty()
    }
}

/// A grid position for path planning.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct GridPos {
    pub x: i32,
    pub y: i32,
}

impl GridPos {
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    pub fn distance_to(&self, other: &GridPos) -> f64 {
        let dx = (self.x - other.x) as f64;
        let dy = (self.y - other.y) as f64;
        (dx * dx + dy * dy).sqrt()
    }

    pub fn manhattan_to(&self, other: &GridPos) -> i32 {
        (self.x - other.x).abs() + (self.y - other.y).abs()
    }
}

/// Ternary obstacle: Free (0), Unknown (-1), Blocked (+1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TernaryObstacle {
    Unknown,
    Free,
    Blocked,
}

/// Path planner with ternary obstacles on a grid.
pub struct PathPlanner {
    pub width: i32,
    pub height: i32,
    pub grid: Vec<Vec<TernaryObstacle>>,
}

impl PathPlanner {
    pub fn new(width: i32, height: i32) -> Self {
        Self {
            width,
            height,
            grid: vec![vec![TernaryObstacle::Free; height as usize]; width as usize],
        }
    }

    pub fn set_obstacle(&mut self, pos: GridPos, obs: TernaryObstacle) {
        if pos.x >= 0 && pos.x < self.width && pos.y >= 0 && pos.y < self.height {
            self.grid[pos.x as usize][pos.y as usize] = obs;
        }
    }

    pub fn get(&self, pos: GridPos) -> TernaryObstacle {
        if pos.x >= 0 && pos.x < self.width && pos.y >= 0 && pos.y < self.height {
            self.grid[pos.x as usize][pos.y as usize]
        } else {
            TernaryObstacle::Blocked
        }
    }

    /// Simple BFS pathfinding avoiding blocked cells.
    pub fn find_path(&self, start: GridPos, goal: GridPos) -> Option<Vec<GridPos>> {
        if self.get(start) == TernaryObstacle::Blocked || self.get(goal) == TernaryObstacle::Blocked
        {
            return None;
        }

        let mut visited = vec![vec![false; self.height as usize]; self.width as usize];
        let mut queue = vec![(start, vec![start])];
        visited[start.x as usize][start.y as usize] = true;

        let directions = [(0, 1), (0, -1), (1, 0), (-1, 0)];

        let mut head = 0;
        while head < queue.len() {
            let (pos, path) = queue[head].clone();
            head += 1;

            if pos == goal {
                return Some(path);
            }

            for (dx, dy) in &directions {
                let next = GridPos::new(pos.x + dx, pos.y + dy);
                if next.x >= 0
                    && next.x < self.width
                    && next.y >= 0
                    && next.y < self.height
                    && !visited[next.x as usize][next.y as usize]
                    && self.grid[next.x as usize][next.y as usize] != TernaryObstacle::Blocked
                {
                    visited[next.x as usize][next.y as usize] = true;
                    let mut new_path = path.clone();
                    new_path.push(next);
                    queue.push((next, new_path));
                }
            }
        }

        None
    }

    /// Count obstacles by type.
    pub fn count_obstacles(&self) -> (usize, usize, usize) {
        let mut unknown = 0;
        let mut free = 0;
        let mut blocked = 0;
        for col in &self.grid {
            for cell in col {
                match cell {
                    TernaryObstacle::Unknown => unknown += 1,
                    TernaryObstacle::Free => free += 1,
                    TernaryObstacle::Blocked => blocked += 1,
                }
            }
        }
        (unknown, free, blocked)
    }
}

/// Robot state tracking position and heading.
#[derive(Clone, Debug)]
pub struct RobotState {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    pub speed: f64,
    pub timestamp: f64,
}

impl RobotState {
    pub fn new() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            speed: 0.0,
            timestamp: 0.0,
        }
    }

    pub fn at(x: f64, y: f64, theta: f64) -> Self {
        Self {
            x,
            y,
            theta,
            speed: 0.0,
            timestamp: 0.0,
        }
    }

    /// Update state using differential drive odometry.
    pub fn update_odometry(&mut self, linear: f64, angular: f64, dt: f64) {
        let new_theta = self.theta + angular * dt;
        let new_x = self.x + linear * new_theta.cos() * dt;
        let new_y = self.y + linear * new_theta.sin() * dt;
        self.theta = new_theta;
        self.x = new_x;
        self.y = new_y;
        self.speed = linear;
        self.timestamp += dt;
    }

    pub fn distance_to(&self, other: &RobotState) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Heading difference in radians (-π to π).
    pub fn heading_diff(&self, other: &RobotState) -> f64 {
        let diff = other.theta - self.theta;
        let normalized = diff % (2.0 * std::f64::consts::PI);
        if normalized > std::f64::consts::PI {
            normalized - 2.0 * std::f64::consts::PI
        } else if normalized < -std::f64::consts::PI {
            normalized + 2.0 * std::f64::consts::PI
        } else {
            normalized
        }
    }

    /// Choose ternary actuator command to navigate toward a target.
    pub fn navigate_toward(&self, target: &RobotState, threshold: f64) -> TernaryActuator {
        let dist = self.distance_to(target);
        if dist < threshold {
            return TernaryActuator::Stop;
        }
        let dx = target.x - self.x;
        let dy = target.y - self.y;
        let angle_to_target = dy.atan2(dx);
        let angle_diff = angle_to_target - self.theta;
        let normalized = if angle_diff > std::f64::consts::PI {
            angle_diff - 2.0 * std::f64::consts::PI
        } else if angle_diff < -std::f64::consts::PI {
            angle_diff + 2.0 * std::f64::consts::PI
        } else {
            angle_diff
        };
        if normalized.abs() < 0.3 {
            TernaryActuator::Forward
        } else {
            TernaryActuator::Stop
        }
    }
}

impl Default for RobotState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_actuator_values() {
        assert_eq!(TernaryActuator::Reverse.value(), -1);
        assert_eq!(TernaryActuator::Stop.value(), 0);
        assert_eq!(TernaryActuator::Forward.value(), 1);
    }

    #[test]
    fn test_actuator_from_value() {
        assert_eq!(TernaryActuator::from_value(-1), Some(TernaryActuator::Reverse));
        assert_eq!(TernaryActuator::from_value(2), None);
    }

    #[test]
    fn test_actuator_apply_speed() {
        let result = TernaryActuator::Forward.apply_to_speed(0.0, 0.3);
        assert!((result - 0.3).abs() < 1e-10);
        let result = TernaryActuator::Reverse.apply_to_speed(0.0, 0.3);
        assert!((result - (-0.3)).abs() < 1e-10);
        let result = TernaryActuator::Stop.apply_to_speed(0.8, 0.3);
        assert!((result - 0.4).abs() < 1e-10);
    }

    #[test]
    fn test_differential_drive_forward() {
        let mut drive = DifferentialDrive::new(0.5, 0.1, 1.0);
        drive.set_left(TernaryActuator::Forward);
        drive.set_right(TernaryActuator::Forward);
        let (lin, ang) = drive.compute_velocities();
        assert!((lin - 1.0).abs() < 1e-10);
        assert!((ang - 0.0).abs() < 1e-10);
        assert!(!drive.is_turning());
        assert!(!drive.is_stopped());
    }

    #[test]
    fn test_differential_drive_turn() {
        let mut drive = DifferentialDrive::new(0.5, 0.1, 1.0);
        drive.set_left(TernaryActuator::Reverse);
        drive.set_right(TernaryActuator::Forward);
        let (_, ang) = drive.compute_velocities();
        assert!(ang > 0.0);
        assert!(drive.is_turning());
    }

    #[test]
    fn test_differential_drive_stopped() {
        let drive = DifferentialDrive::new(0.5, 0.1, 1.0);
        assert!(drive.is_stopped());
    }

    #[test]
    fn test_sensor_reading() {
        let mut sensor = TernarySensor::new(0, 0.0, 10.0);
        sensor.read(8.0);
        assert_eq!(sensor.ternary, TernaryActuator::Forward);
        sensor.read(2.0);
        assert_eq!(sensor.ternary, TernaryActuator::Reverse);
        sensor.read(5.0);
        assert_eq!(sensor.ternary, TernaryActuator::Stop);
    }

    #[test]
    fn test_sensor_normalized() {
        let mut sensor = TernarySensor::new(0, 0.0, 10.0);
        sensor.read(5.0);
        assert!((sensor.normalized() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_sensor_array_majority() {
        let mut arr = TernarySensorArray::new(3, 0.0, 10.0);
        arr.read_all(&[8.0, 9.0, 2.0]);
        // 2 Forward, 1 Reverse → Forward
        assert_eq!(arr.majority(), TernaryActuator::Forward);
    }

    #[test]
    fn test_sensor_array_len() {
        let arr = TernarySensorArray::new(5, 0.0, 10.0);
        assert_eq!(arr.len(), 5);
        assert!(!arr.is_empty());
    }

    #[test]
    fn test_grid_pos_distance() {
        let a = GridPos::new(0, 0);
        let b = GridPos::new(3, 4);
        assert!((a.distance_to(&b) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_grid_pos_manhattan() {
        let a = GridPos::new(0, 0);
        let b = GridPos::new(3, 4);
        assert_eq!(a.manhattan_to(&b), 7);
    }

    #[test]
    fn test_path_planner_straight() {
        let mut planner = PathPlanner::new(10, 10);
        let start = GridPos::new(0, 0);
        let goal = GridPos::new(0, 5);
        let path = planner.find_path(start, goal);
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(path.len(), 6); // 5 steps + start
    }

    #[test]
    fn test_path_planner_blocked() {
        let mut planner = PathPlanner::new(5, 5);
        planner.set_obstacle(GridPos::new(0, 0), TernaryObstacle::Blocked);
        let path = planner.find_path(GridPos::new(0, 0), GridPos::new(4, 4));
        assert!(path.is_none());
    }

    #[test]
    fn test_path_planner_obstacle_count() {
        let mut planner = PathPlanner::new(3, 3);
        planner.set_obstacle(GridPos::new(1, 1), TernaryObstacle::Blocked);
        planner.set_obstacle(GridPos::new(2, 2), TernaryObstacle::Unknown);
        let (unknown, free, blocked) = planner.count_obstacles();
        assert_eq!(unknown, 1);
        assert_eq!(blocked, 1);
        assert_eq!(free, 7);
    }

    #[test]
    fn test_robot_state_odometry() {
        let mut state = RobotState::new();
        state.update_odometry(1.0, 0.0, 1.0);
        assert!((state.x - 1.0).abs() < 1e-10);
        assert!((state.y - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_robot_state_distance() {
        let a = RobotState::at(0.0, 0.0, 0.0);
        let b = RobotState::at(3.0, 4.0, 0.0);
        assert!((a.distance_to(&b) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_robot_state_heading_diff() {
        let a = RobotState::at(0.0, 0.0, 0.0);
        let b = RobotState::at(0.0, 0.0, std::f64::consts::PI / 2.0);
        let diff = a.heading_diff(&b);
        assert!((diff - std::f64::consts::PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_robot_navigate_toward() {
        let robot = RobotState::at(0.0, 0.0, 0.0);
        let target = RobotState::at(10.0, 0.0, 0.0);
        assert_eq!(robot.navigate_toward(&target, 1.0), TernaryActuator::Forward);
    }

    #[test]
    fn test_robot_navigate_arrived() {
        let robot = RobotState::at(5.0, 5.0, 0.0);
        let target = RobotState::at(5.1, 5.1, 0.0);
        assert_eq!(robot.navigate_toward(&target, 1.0), TernaryActuator::Stop);
    }
}
