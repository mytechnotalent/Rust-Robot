// Copyright (c) 2025 Kevin Thomas
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

//! RP2350A Robot Controller with IR Remote
//!
//! This firmware provides async robot control for Waveshare Pico2Go (RP2350-Plus)
//! using Embassy async runtime and NEC IR protocol for remote control.
//!
//! # Hardware Configuration
//! - **Microcontroller**: RP2350A (ARM Cortex-M33)
//! - **Motors**: Dual DC motors with H-bridge driver
//!   - Left motor: PWM on GPIO 16 (PWMA), direction on GPIO 17 (AIN2) & GPIO 18 (AIN1)
//!   - Right motor: PWM on GPIO 21 (PWMB), direction on GPIO 19 (BIN1) & GPIO 20 (BIN2)
//! - **IR Receiver**: NEC protocol decoder on GPIO 5
//! - **Status LED**: GPIO 25
//!
//! # Features
//! - Full directional control (forward, backward, left, right, stop)
//! - Variable speed control via IR remote (10% increments)
//! - NEC IR protocol decoder with 32-bit data frame support
//! - Automatic safety stop after 800ms of no signal
//! - Real-time defmt logging for debugging
//!
//! # IR Remote Commands
//! - `0x18`: Move forward at current speed
//! - `0x52`: Move backward at current speed
//! - `0x08`: Turn left (50% speed for precision)
//! - `0x5A`: Turn right (50% speed for precision)
//! - `0x1C`: Stop all motors
//! - `0x09`: Reset speed to 50%
//! - `0x15`: Increase speed by ~10%
//! - `0x07`: Decrease speed by ~10%
//!
//! # Build
//! ```bash
//! cargo build --release --target thumbv8m.main-none-eabihf
//! ```
//!
//! # Flash
//! ```bash
//! ./quick-flash.sh
//! ```

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::pwm::{Config as PwmConfig, Pwm, SetDutyCycle};
use embassy_time::{Duration, Instant, Timer};
use {defmt_rtt as _, panic_probe as _};

/// Program metadata for picotool info command
///
/// This information is embedded in the binary and can be read by picotool
/// to display program information when querying the firmware.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Robot Controller"),
    embassy_rp::binary_info::rp_program_description!(c"RP2350A Robot with IR Remote Control"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

/// Represents a dual DC motor controller with H-bridge driver
///
/// Manages PWM-based speed control and directional control for two DC motors
/// using separate direction pins for forward/backward movement.
struct MotorController {
    pwm_left: Pwm<'static>,
    pwm_right: Pwm<'static>,
    left_fwd: Output<'static>,
    left_back: Output<'static>,
    right_fwd: Output<'static>,
    right_back: Output<'static>,
}

impl MotorController {
    /// Stops both motors immediately
    ///
    /// Sets PWM duty cycle to zero and disables all direction pins.
    ///
    /// # Example
    /// ```ignore
    /// motors.stop();
    /// ```
    fn stop(&mut self) {
        self.pwm_left.set_duty_cycle(0).ok();
        self.pwm_right.set_duty_cycle(0).ok();
        self.left_fwd.set_low();
        self.left_back.set_low();
        self.right_fwd.set_low();
        self.right_back.set_low();
    }

    /// Moves the robot forward at the specified speed
    ///
    /// Sets both motors to forward direction with matching PWM duty cycle.
    ///
    /// # Arguments
    /// * `speed` - PWM duty cycle (0-65535)
    ///
    /// # Example
    /// ```ignore
    /// motors.forward(32768);
    /// ```
    fn forward(&mut self, speed: u16) {
        self.pwm_left.set_duty_cycle(speed).ok();
        self.pwm_right.set_duty_cycle(speed).ok();
        self.left_fwd.set_high();
        self.left_back.set_low();
        self.right_fwd.set_high();
        self.right_back.set_low();
    }

    /// Moves the robot backward at the specified speed
    ///
    /// Sets both motors to reverse direction with matching PWM duty cycle.
    ///
    /// # Arguments
    /// * `speed` - PWM duty cycle (0-65535)
    ///
    /// # Example
    /// ```ignore
    /// motors.backward(32768);
    /// ```
    fn backward(&mut self, speed: u16) {
        self.pwm_left.set_duty_cycle(speed).ok();
        self.pwm_right.set_duty_cycle(speed).ok();
        self.left_fwd.set_low();
        self.left_back.set_high();
        self.right_fwd.set_low();
        self.right_back.set_high();
    }

    /// Turns the robot left (counter-clockwise) at the specified speed
    ///
    /// Left motor reverses, right motor moves forward for in-place rotation.
    ///
    /// # Arguments
    /// * `speed` - PWM duty cycle (0-65535)
    ///
    /// # Example
    /// ```ignore
    /// motors.left(10000);
    /// ```
    fn left(&mut self, speed: u16) {
        self.pwm_left.set_duty_cycle(speed).ok();
        self.pwm_right.set_duty_cycle(speed).ok();
        self.left_fwd.set_low();
        self.left_back.set_high();
        self.right_fwd.set_high();
        self.right_back.set_low();
    }

    /// Turns the robot right (clockwise) at the specified speed
    ///
    /// Left motor moves forward, right motor reverses for in-place rotation.
    ///
    /// # Arguments
    /// * `speed` - PWM duty cycle (0-65535)
    ///
    /// # Example
    /// ```ignore
    /// motors.right(10000);
    /// ```
    fn right(&mut self, speed: u16) {
        self.pwm_left.set_duty_cycle(speed).ok();
        self.pwm_right.set_duty_cycle(speed).ok();
        self.left_fwd.set_high();
        self.left_back.set_low();
        self.right_fwd.set_low();
        self.right_back.set_high();
    }
}

/// Waits for IR pin to reach specified logic level with timeout
///
/// Polls the IR receiver pin until it matches the target level or timeout expires.
/// Used for measuring pulse widths in NEC protocol decoding.
///
/// # Arguments
/// * `pin` - Input GPIO pin connected to IR receiver
/// * `level` - Target logic level to wait for (High or Low)
/// * `timeout_us` - Maximum time to wait in microseconds
///
/// # Returns
/// * `Some(duration)` - Time elapsed in microseconds when level reached
/// * `None` - Timeout occurred before level was reached
fn wait_for_level(pin: &Input, level: Level, timeout_us: u64) -> Option<u64> {
    let start = Instant::now();
    let timeout = Duration::from_micros(timeout_us);

    loop {
        // Check current pin state
        let current_level = if pin.is_high() {
            Level::High
        } else {
            Level::Low
        };
        if current_level == level {
            return Some(start.elapsed().as_micros());
        }
        // Check for timeout
        if start.elapsed() > timeout {
            return None;
        }
    }
}

/// Decodes NEC IR protocol command from IR receiver
///
/// Implements the complete NEC IR protocol state machine to decode 32-bit
/// data frames. Validates timing of lead pulse, space, and data bits.
///
/// # Arguments
/// * `ir_pin` - Input GPIO pin connected to IR receiver (active low)
///
/// # Returns
/// * `Some(command)` - Successfully decoded command byte
/// * `None` - Invalid frame or timeout
fn ir_getkey(ir_pin: &Input) -> Option<u8> {
    // Wait for leading pulse (9ms low)
    wait_for_level(ir_pin, Level::Low, 150000)?;
    let t = wait_for_level(ir_pin, Level::High, 12000)?;
    if t < 8000 || t > 10000 {
        return None; // Not a valid lead pulse
    }

    // Wait for space (4.5ms high)
    let t = wait_for_level(ir_pin, Level::Low, 7000)?;
    if t < 3500 || t > 5000 {
        return None;
    }

    let mut data = [0u8; 4];
    for i in 0..32 {
        // Wait 0.56ms low
        wait_for_level(ir_pin, Level::High, 1000)?;
        // Measure next high (bit time)
        let t = wait_for_level(ir_pin, Level::Low, 2500)?;
        if t < 200 {
            return None;
        }
        let idx = i / 8;
        let bit = i % 8;
        if t > 1200 {
            // 1.69ms for '1'
            data[idx] |= 1 << bit;
        }
    }

    // Check address/inverse and data/inverse
    if data[0].wrapping_add(data[1]) == 0xFF && data[2].wrapping_add(data[3]) == 0xFF {
        Some(data[2])
    } else {
        None
    }
}

/// Main robot controller task
///
/// Initializes all hardware peripherals and enters the main control loop
/// which continuously polls the IR receiver for commands and controls
/// the motors accordingly.
///
/// # Initialization Sequence
/// 1. Configure GPIO pins for motors, LED, and IR receiver
/// 2. Initialize PWM controllers for motor speed control
/// 3. Enter main control loop with IR command processing
///
/// # Safety
/// Never returns. Runs indefinitely until power loss or reset.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("RP2350A Robot Controller Starting!");
    let p = embassy_rp::init(Default::default());

    // Initialize status LED on GPIO 25
    let mut led = Output::new(p.PIN_25, Level::Low);

    // Initialize motor direction control pins
    let left_fwd = Output::new(p.PIN_18, Level::Low); // AIN1
    let left_back = Output::new(p.PIN_17, Level::Low); // AIN2
    let right_fwd = Output::new(p.PIN_19, Level::Low); // BIN1
    let right_back = Output::new(p.PIN_20, Level::Low); // BIN2

    // Configure PWM for motor speed control
    let mut pwm_config = PwmConfig::default();
    pwm_config.top = 65535; // 16-bit resolution
    pwm_config.compare_a = 0;
    pwm_config.compare_b = 0;

    // Left motor PWM on GPIO 16 (PWMA)
    let pwm_left = Pwm::new_output_a(p.PWM_SLICE0, p.PIN_16, pwm_config.clone());

    // Right motor PWM on GPIO 21 (PWMB)
    let pwm_right = Pwm::new_output_b(p.PWM_SLICE2, p.PIN_21, pwm_config.clone());

    let mut motors = MotorController {
        pwm_left,
        pwm_right,
        left_fwd,
        left_back,
        right_fwd,
        right_back,
    };

    // Initialize IR receiver input on GPIO 5 with pull-up
    let ir_pin = Input::new(p.PIN_5, Pull::Up);

    // Initialize control variables
    let mut speed: u16 = 32768; // 50% duty cycle (default speed)
    let mut n = 0u32; // Safety timeout counter

    info!("Robot ready! Waiting for IR commands...");
    led.set_high(); // Set LED high to indicate ready state

    // Main control loop
    loop {
        if let Some(key) = ir_getkey(&ir_pin) {
            // Reset safety counter on valid command
            n = 0;
            // Toggle LED to indicate command reception
            led.toggle();

            match key {
                0x18 => {
                    // Forward
                    motors.forward(speed);
                    info!("Forward");
                }
                0x08 => {
                    // Left
                    motors.left(speed / 2);
                    info!("Left");
                }
                0x1C => {
                    // Stop
                    motors.stop();
                    info!("Stop");
                }
                0x5A => {
                    // Right
                    motors.right(speed / 2);
                    info!("Right");
                }
                0x52 => {
                    // Backward
                    motors.backward(speed);
                    info!("Backward");
                }
                0x09 => {
                    // Speed reset
                    speed = 32768;
                    info!("Speed reset: {}", speed);
                }
                0x15 => {
                    // Speed up
                    if speed <= 65535 - 6553 {
                        speed += 6553;
                    }
                    info!("Speed up: {}", speed);
                }
                0x07 => {
                    // Speed down
                    if speed >= 6553 {
                        speed -= 6553;
                    }
                    info!("Speed down: {}", speed);
                }
                _ => {
                    info!("Unknown key: 0x{:02X}", key);
                }
            }
        } else {
            // Increment timeout counter when no command received
            n += 1;
            // Safety stop after 800ms of no signal
            if n > 800 {
                n = 0;
                motors.stop();
            }
            Timer::after_millis(1).await;
        }
    }
}
