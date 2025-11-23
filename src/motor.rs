// Copyright (c) 2025 Kevin Thomas
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

//! Motor control module for dual DC motor H-bridge driver.
//!
//! This module provides a high-level interface for controlling two DC motors
//! using PWM-based speed control and directional control via H-bridge driver pins.
//!
//! # Hardware Interface
//!
//! The motor controller uses:
//! - 2 PWM channels for independent speed control of left and right motors
//! - 4 GPIO pins for H-bridge direction control (2 per motor)
//!
//! # Motor Pin Configuration
//!
//! ## Left Motor
//! - PWM: PWMA (GPIO 16)
//! - Forward: AIN1 (GPIO 18)
//! - Backward: AIN2 (GPIO 17)
//!
//! ## Right Motor
//! - PWM: PWMB (GPIO 21)
//! - Forward: BIN1 (GPIO 19)
//! - Backward: BIN2 (GPIO 20)
//!
//! # Examples
//!
//! ```ignore
//! use motor::MotorController;
//!
//! let mut motors = MotorController::new(
//!     pwm_left,
//!     pwm_right,
//!     left_fwd,
//!     left_back,
//!     right_fwd,
//!     right_back,
//! );
//!
//! // Move forward at 50% speed
//! motors.forward(32768);
//!
//! // Stop all motors
//! motors.stop();
//! ```

use embassy_rp::gpio::Output;
use embassy_rp::pwm::{Pwm, SetDutyCycle};

/// Dual DC motor controller with H-bridge driver support.
///
/// Provides methods for controlling two DC motors with independent PWM speed control
/// and direction control. All direction changes are synchronized to prevent motor
/// driver damage from simultaneous forward/backward activation.
///
/// # Safety
///
/// The controller ensures that opposing direction pins are never enabled
/// simultaneously, which could damage the H-bridge driver or motors.
pub struct MotorController {
    /// PWM controller for left motor speed
    pwm_left: Pwm<'static>,
    /// PWM controller for right motor speed
    pwm_right: Pwm<'static>,
    /// Left motor forward direction pin (AIN1)
    left_fwd: Output<'static>,
    /// Left motor backward direction pin (AIN2)
    left_back: Output<'static>,
    /// Right motor forward direction pin (BIN1)
    right_fwd: Output<'static>,
    /// Right motor backward direction pin (BIN2)
    right_back: Output<'static>,
}

impl MotorController {
    /// Creates a new motor controller instance.
    ///
    /// # Arguments
    ///
    /// * `pwm_left` - PWM controller for left motor (PWMA)
    /// * `pwm_right` - PWM controller for right motor (PWMB)
    /// * `left_fwd` - GPIO output for left motor forward (AIN1)
    /// * `left_back` - GPIO output for left motor backward (AIN2)
    /// * `right_fwd` - GPIO output for right motor forward (BIN1)
    /// * `right_back` - GPIO output for right motor backward (BIN2)
    ///
    /// # Returns
    ///
    /// A new `MotorController` instance with all motors stopped.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let motors = MotorController::new(
    ///     pwm_left,
    ///     pwm_right,
    ///     left_fwd_pin,
    ///     left_back_pin,
    ///     right_fwd_pin,
    ///     right_back_pin,
    /// );
    /// ```
    pub fn new(
        pwm_left: Pwm<'static>,
        pwm_right: Pwm<'static>,
        left_fwd: Output<'static>,
        left_back: Output<'static>,
        right_fwd: Output<'static>,
        right_back: Output<'static>,
    ) -> Self {
        Self {
            pwm_left,
            pwm_right,
            left_fwd,
            left_back,
            right_fwd,
            right_back,
        }
    }

    /// Stops both motors immediately.
    ///
    /// Sets PWM duty cycle to zero and disables all direction pins to ensure
    /// motors are completely stopped and no current flows through the H-bridge.
    ///
    /// # Safety
    ///
    /// This method is safe to call at any time and will not damage the motors
    /// or H-bridge driver.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// motors.stop();
    /// ```
    pub fn stop(&mut self) {
        self.pwm_left.set_duty_cycle(0).ok();
        self.pwm_right.set_duty_cycle(0).ok();
        self.left_fwd.set_low();
        self.left_back.set_low();
        self.right_fwd.set_low();
        self.right_back.set_low();
    }

    /// Moves the robot forward at the specified speed.
    ///
    /// Sets both motors to forward direction with matching PWM duty cycle
    /// for straight-line forward motion.
    ///
    /// # Arguments
    ///
    /// * `speed` - PWM duty cycle value (0-65535)
    ///   - 0: Stopped
    ///   - 32768: 50% speed
    ///   - 65535: Maximum speed
    ///
    /// # Examples
    ///
    /// ```ignore
    /// // Move forward at 50% speed
    /// motors.forward(32768);
    ///
    /// // Move forward at full speed
    /// motors.forward(65535);
    /// ```
    pub fn forward(&mut self, speed: u16) {
        self.pwm_left.set_duty_cycle(speed).ok();
        self.pwm_right.set_duty_cycle(speed).ok();
        self.left_fwd.set_high();
        self.left_back.set_low();
        self.right_fwd.set_high();
        self.right_back.set_low();
    }

    /// Moves the robot backward at the specified speed.
    ///
    /// Sets both motors to reverse direction with matching PWM duty cycle
    /// for straight-line backward motion.
    ///
    /// # Arguments
    ///
    /// * `speed` - PWM duty cycle value (0-65535)
    ///   - 0: Stopped
    ///   - 32768: 50% speed
    ///   - 65535: Maximum speed
    ///
    /// # Examples
    ///
    /// ```ignore
    /// // Move backward at 50% speed
    /// motors.backward(32768);
    /// ```
    pub fn backward(&mut self, speed: u16) {
        self.pwm_left.set_duty_cycle(speed).ok();
        self.pwm_right.set_duty_cycle(speed).ok();
        self.left_fwd.set_low();
        self.left_back.set_high();
        self.right_fwd.set_low();
        self.right_back.set_high();
    }

    /// Turns the robot left (counter-clockwise) at the specified speed.
    ///
    /// Left motor reverses while right motor moves forward, creating an
    /// in-place rotation to the left.
    ///
    /// # Arguments
    ///
    /// * `speed` - PWM duty cycle value (0-65535)
    ///   - Higher values create faster rotation
    ///   - Recommended: Use 50% of forward speed for controlled turns
    ///
    /// # Examples
    ///
    /// ```ignore
    /// // Turn left at 25% speed for precise control
    /// motors.left(16384);
    /// ```
    pub fn left(&mut self, speed: u16) {
        self.pwm_left.set_duty_cycle(speed).ok();
        self.pwm_right.set_duty_cycle(speed).ok();
        self.left_fwd.set_low();
        self.left_back.set_high();
        self.right_fwd.set_high();
        self.right_back.set_low();
    }

    /// Turns the robot right (clockwise) at the specified speed.
    ///
    /// Left motor moves forward while right motor reverses, creating an
    /// in-place rotation to the right.
    ///
    /// # Arguments
    ///
    /// * `speed` - PWM duty cycle value (0-65535)
    ///   - Higher values create faster rotation
    ///   - Recommended: Use 50% of forward speed for controlled turns
    ///
    /// # Examples
    ///
    /// ```ignore
    /// // Turn right at 25% speed for precise control
    /// motors.right(16384);
    /// ```
    pub fn right(&mut self, speed: u16) {
        self.pwm_left.set_duty_cycle(speed).ok();
        self.pwm_right.set_duty_cycle(speed).ok();
        self.left_fwd.set_high();
        self.left_back.set_low();
        self.right_fwd.set_low();
        self.right_back.set_high();
    }
}
