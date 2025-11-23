// Copyright (c) 2025 Kevin Thomas
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

//! RP2350A Robot Controller with IR Remote Control
//!
//! High-level firmware for Waveshare Pico2Go (RP2350-Plus) mobile robot platform
//! using Embassy async runtime and NEC infrared protocol for wireless control.
//!
//! # Overview
//!
//! This firmware provides a complete robot control system featuring:
//! - **Dual DC motor control** with PWM speed regulation
//! - **NEC IR protocol decoder** for wireless remote control
//! - **Automatic safety features** including timeout-based motor stop
//! - **Variable speed control** with 10% increment adjustments
//! - **Real-time logging** via defmt for debugging
//!
//! # Hardware Platform
//!
//! - **Microcontroller**: RP2350A (ARM Cortex-M33, dual-core)
//! - **Board**: Waveshare Pico2Go Robot Platform
//! - **Motors**: Dual DC motors with H-bridge driver
//! - **Remote**: NEC protocol IR remote (38kHz carrier)
//!
//! # Architecture
//!
//! The firmware is organized into modular components:
//!
//! ```text
//! ┌──────────────────────────────────────────┐
//! │            main.rs (this file)           │
//! │  - Hardware initialization               │
//! │  - Main control loop                     │
//! │  - Task spawning                         │
//! └──────────────────────────────────────────┘
//!                    │
//!         ┌──────────┴──────────┐
//!         ▼                     ▼
//! ┌───────────────┐    ┌───────────────┐
//! │  ir_receiver  │    │  controller   │
//! │  - NEC decode │    │  - State mgmt │
//! │  - Timing     │    │  - Commands   │
//! └───────────────┘    └───────────────┘
//!                              │
//!                              ▼
//!                      ┌───────────────┐
//!                      │     motor     │
//!                      │  - PWM ctrl   │
//!                      │  - Direction  │
//!                      └───────────────┘
//! ```
//!
//! # Module Organization
//!
//! - [`config`] - Hardware pin mappings and constants
//! - [`motor`] - Motor control with H-bridge driver
//! - [`ir_receiver`] - NEC IR protocol decoder
//! - [`command`] - Command definitions and IR code mappings
//! - [`controller`] - Robot state machine and command execution
//!
//! # Control Flow
//!
//! 1. **Initialization**: Configure GPIO, PWM, and peripherals
//! 2. **Main Loop**:
//!    - Poll IR receiver for commands
//!    - Decode NEC protocol frames
//!    - Execute motor commands
//!    - Monitor safety timeout
//!    - Update status LED
//!
//! # Safety Features
//!
//! - **Automatic Stop**: Motors stop after 800ms without valid commands
//! - **Speed Limits**: Constrained to 10-100% PWM range
//! - **Turn Speed Reduction**: Turning uses 50% of current speed
//!
//! # Build & Flash
//!
//! ```bash
//! cargo run --release
//! ```
//!
//! # Dependencies
//!
//! - `embassy-rp`: RP2350 HAL and async runtime
//! - `embassy-executor`: Async task executor
//! - `embassy-time`: Async timers and delays
//! - `defmt`: Efficient logging framework
//! - `panic-probe`: Debug panic handler
//!
//! # License
//!
//! MIT License - See LICENSE file for details

#![no_std]
#![no_main]

mod command;
mod config;
mod controller;
mod ir_receiver;
mod motor;

use command::Command;
use controller::RobotController;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_time::Timer;
use ir_receiver::IrReceiver;
use motor::MotorController;
use {defmt_rtt as _, panic_probe as _};

/// Program metadata for picotool information display.
///
/// This data is embedded in the binary and can be queried using the
/// picotool utility to display program name, version, and build info.
///
/// # Examples
///
/// ```bash
/// picotool info robot-embassy.uf2
/// ```
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Robot Controller"),
    embassy_rp::binary_info::rp_program_description!(c"RP2350A Robot with IR Remote Control"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

/// Main firmware entry point and control loop.
///
/// Initializes all hardware peripherals and runs the main robot control loop
/// which continuously monitors the IR receiver and executes commands.
///
/// # Initialization Sequence
///
/// 1. Initialize Embassy runtime and RP2350 peripherals
/// 2. Configure status LED (GPIO 25)
/// 3. Configure motor H-bridge direction pins
/// 4. Initialize PWM controllers for motor speed
/// 5. Configure IR receiver input with pull-up
/// 6. Create controller and motor instances
/// 7. Enter main control loop
///
/// # Main Loop Operation
///
/// The control loop operates at approximately 1kHz (1ms per iteration):
///
/// 1. **Check IR Receiver**: Attempt to decode NEC command
/// 2. **On Valid Command**:
///    - Reset safety timeout
///    - Toggle status LED
///    - Execute command via controller
/// 3. **On No Command**:
///    - Increment safety timeout
///    - Stop motors if timeout exceeded
///    - Delay 1ms before next iteration
///
/// # Task Lifecycle
///
/// This task never returns and runs indefinitely until power loss or reset.
///
/// # Arguments
///
/// * `_spawner` - Embassy task spawner (unused in current implementation)
///
/// # Panics
///
/// Will panic (via `panic_probe`) if:
/// - Hardware initialization fails
/// - Critical runtime error occurs
///
/// # Examples
///
/// This function is the entry point and is automatically called by Embassy:
///
/// ```ignore
/// #[embassy_executor::main]
/// async fn main(_spawner: Spawner) {
///     // Firmware runs here
/// }
/// ```
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("RP2350A Robot Controller Starting!");

    // Initialize RP2350 peripherals with default configuration
    let p = embassy_rp::init(Default::default());

    // Configure status LED (onboard LED on GPIO 25)
    let mut led = Output::new(p.PIN_25, Level::Low);

    // Configure motor H-bridge direction control pins
    let left_fwd = Output::new(p.PIN_18, Level::Low); // AIN1 - Left forward
    let left_back = Output::new(p.PIN_17, Level::Low); // AIN2 - Left backward
    let right_fwd = Output::new(p.PIN_19, Level::Low); // BIN1 - Right forward
    let right_back = Output::new(p.PIN_20, Level::Low); // BIN2 - Right backward

    // Configure PWM for motor speed control (16-bit resolution)
    let mut pwm_config = PwmConfig::default();
    pwm_config.top = config::PWM_TOP;
    pwm_config.compare_a = config::PWM_INITIAL_COMPARE_A;
    pwm_config.compare_b = config::PWM_INITIAL_COMPARE_B;

    // Initialize left motor PWM (GPIO 16, PWM_SLICE0 Channel A)
    let pwm_left = Pwm::new_output_a(p.PWM_SLICE0, p.PIN_16, pwm_config.clone());

    // Initialize right motor PWM (GPIO 21, PWM_SLICE2 Channel B)
    let pwm_right = Pwm::new_output_b(p.PWM_SLICE2, p.PIN_21, pwm_config);

    // Create motor controller instance
    let mut motors = MotorController::new(
        pwm_left,
        pwm_right,
        left_fwd,
        left_back,
        right_fwd,
        right_back,
    );

    // Configure IR receiver input (GPIO 5 with pull-up)
    let ir_pin = Input::new(p.PIN_5, Pull::Up);
    let ir = IrReceiver::new(&ir_pin);

    // Create robot controller with default settings
    let mut controller = RobotController::new();

    // Signal ready state
    info!("Robot ready! Waiting for IR commands...");
    led.set_high();

    // Main control loop - runs indefinitely
    loop {
        // Attempt to read and decode IR command
        if let Some(ir_code) = ir.read_command() {
            // Valid command received - reset safety timeout
            controller.reset_timeout();

            // Toggle LED to provide visual feedback
            led.toggle();

            // Convert IR code to command and execute
            let command = Command::from_ir_code(ir_code);
            controller.execute_command(command, &mut motors);
        } else {
            // No valid command - increment timeout counter
            if controller.increment_timeout() {
                // Safety timeout exceeded - stop all motors
                motors.stop();
            }

            // Wait 1ms before next iteration
            Timer::after_millis(1).await;
        }
    }
}
