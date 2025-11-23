// Copyright (c) 2025 Kevin Thomas
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

//! Robot controller state machine and command execution.
//!
//! This module implements the main robot controller logic, managing:
//! - Current speed state
//! - Motor command execution
//! - Safety timeout monitoring
//! - Speed adjustment logic
//!
//! # Speed Control
//!
//! Speed is represented as a 16-bit PWM duty cycle value:
//! - Minimum: 6,553 (~10%)
//! - Default: 32,768 (50%)
//! - Maximum: 65,535 (100%)
//! - Step size: 6,553 (~10% increments)
//!
//! # Safety Features
//!
//! - Automatic motor stop after 800ms without commands
//! - Speed limits to prevent motor damage
//! - Reduced speed for turning maneuvers

use crate::command::Command;
use crate::motor::MotorController;
use defmt::info;

/// Speed increment/decrement value (approximately 10% of max PWM)
const SPEED_STEP: u16 = 6553;

/// Default robot speed (50% PWM duty cycle)
const DEFAULT_SPEED: u16 = 32768;

/// Minimum allowed speed (10% PWM duty cycle)
const MIN_SPEED: u16 = SPEED_STEP;

/// Maximum allowed speed (100% PWM duty cycle)
const MAX_SPEED: u16 = 65535;

/// Maximum milliseconds without command before auto-stop
const SAFETY_TIMEOUT_MS: u32 = 800;

/// Robot controller state machine.
///
/// Manages the robot's operational state including current speed,
/// motor control, and safety features.
pub struct RobotController {
    /// Current motor speed (PWM duty cycle 0-65535)
    speed: u16,
    /// Safety timeout counter in milliseconds
    timeout_counter: u32,
}

impl RobotController {
    /// Creates a new robot controller with default settings.
    ///
    /// # Returns
    ///
    /// A new `RobotController` instance with:
    /// - Speed set to 50% (32,768)
    /// - Timeout counter reset to 0
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let controller = RobotController::new();
    /// ```
    pub fn new() -> Self {
        Self {
            speed: DEFAULT_SPEED,
            timeout_counter: 0,
        }
    }

    /// Returns the current motor speed.
    ///
    /// # Returns
    ///
    /// Current PWM duty cycle value (0-65535)
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let speed = controller.speed();
    /// println!("Current speed: {}", speed);
    /// ```
    #[allow(dead_code)]
    pub fn speed(&self) -> u16 {
        self.speed
    }

    /// Resets the safety timeout counter.
    ///
    /// Should be called whenever a valid command is received to prevent
    /// automatic safety stop.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// controller.reset_timeout();
    /// ```
    pub fn reset_timeout(&mut self) {
        self.timeout_counter = 0;
    }

    /// Increments the safety timeout counter.
    ///
    /// Should be called every millisecond when no command is received.
    /// When counter exceeds `SAFETY_TIMEOUT_MS`, motors should be stopped.
    ///
    /// # Returns
    ///
    /// `true` if safety timeout has been exceeded, `false` otherwise
    ///
    /// # Examples
    ///
    /// ```ignore
    /// if controller.increment_timeout() {
    ///     motors.stop();
    /// }
    /// ```
    pub fn increment_timeout(&mut self) -> bool {
        self.timeout_counter += 1;
        if self.timeout_counter > SAFETY_TIMEOUT_MS {
            self.timeout_counter = 0;
            true
        } else {
            false
        }
    }

    /// Executes a robot command.
    ///
    /// Processes the given command and controls motors accordingly.
    /// Motion commands use current speed, turning commands use 50% of current speed.
    ///
    /// # Arguments
    ///
    /// * `command` - The command to execute
    /// * `motors` - Motor controller instance
    ///
    /// # Examples
    ///
    /// ```ignore
    /// controller.execute_command(Command::Forward, &mut motors);
    /// ```
    pub fn execute_command(&mut self, command: Command, motors: &mut MotorController) {
        match command {
            Command::Forward => {
                motors.forward(self.speed);
                info!("Forward");
            }
            Command::Backward => {
                motors.backward(self.speed);
                info!("Backward");
            }
            Command::Left => {
                // Use half speed for turning precision
                motors.left(self.speed / 2);
                info!("Left");
            }
            Command::Right => {
                // Use half speed for turning precision
                motors.right(self.speed / 2);
                info!("Right");
            }
            Command::Stop => {
                motors.stop();
                info!("Stop");
            }
            Command::SpeedReset => {
                self.speed = DEFAULT_SPEED;
                info!("Speed reset: {}", self.speed);
            }
            Command::SpeedUp => {
                if self.speed <= MAX_SPEED - SPEED_STEP {
                    self.speed += SPEED_STEP;
                }
                info!("Speed up: {}", self.speed);
            }
            Command::SpeedDown => {
                if self.speed >= MIN_SPEED + SPEED_STEP {
                    self.speed -= SPEED_STEP;
                }
                info!("Speed down: {}", self.speed);
            }
            Command::Unknown => {
                info!("Unknown command received");
            }
        }
    }
}

/// Provides default initialization for `RobotController`.
///
/// Creates a controller with default settings (50% speed, timeout reset).
/// This implementation allows using `RobotController::default()` or
/// deriving `Default` in structs that contain a `RobotController`.
///
/// # Examples
///
/// ```ignore
/// let controller = RobotController::default();
/// // Equivalent to:
/// let controller = RobotController::new();
/// ```
impl Default for RobotController {
    fn default() -> Self {
        Self::new()
    }
}
