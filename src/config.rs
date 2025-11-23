// Copyright (c) 2025 Kevin Thomas
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

//! Hardware configuration and pin mappings for RP2350A robot.
//!
//! This module defines all hardware-specific configurations including:
//! - GPIO pin assignments
//! - PWM configuration parameters
//! - Hardware feature constants
//!
//! # Pin Mapping Summary
//!
//! ## Motors
//! - **Left Motor PWM**: GPIO 16 (PWM_SLICE0 Channel A)
//! - **Left Forward**: GPIO 18 (AIN1)
//! - **Left Backward**: GPIO 17 (AIN2)
//! - **Right Motor PWM**: GPIO 21 (PWM_SLICE2 Channel B)
//! - **Right Forward**: GPIO 19 (BIN1)
//! - **Right Backward**: GPIO 20 (BIN2)
//!
//! ## Sensors
//! - **IR Receiver**: GPIO 5 (with pull-up)
//!
//! ## Indicators
//! - **Status LED**: GPIO 25 (onboard LED)
//!
//! # PWM Configuration
//!
//! PWM frequency and resolution:
//! - **Resolution**: 16-bit (0-65535)
//! - **Frequency**: System clock / 65536
//! - **Initial duty cycle**: 0 (stopped)

/// GPIO pin number for status LED (onboard LED on RP2350)
#[allow(dead_code)]
pub const LED_PIN: u8 = 25;

/// GPIO pin number for IR receiver data output
#[allow(dead_code)]
pub const IR_RECEIVER_PIN: u8 = 5;

/// GPIO pin number for left motor PWM (PWMA)
#[allow(dead_code)]
pub const LEFT_MOTOR_PWM_PIN: u8 = 16;

/// GPIO pin number for left motor forward direction (AIN1)
#[allow(dead_code)]
pub const LEFT_MOTOR_FORWARD_PIN: u8 = 18;

/// GPIO pin number for left motor backward direction (AIN2)
#[allow(dead_code)]
pub const LEFT_MOTOR_BACKWARD_PIN: u8 = 17;

/// GPIO pin number for right motor PWM (PWMB)
#[allow(dead_code)]
pub const RIGHT_MOTOR_PWM_PIN: u8 = 21;

/// GPIO pin number for right motor forward direction (BIN1)
#[allow(dead_code)]
pub const RIGHT_MOTOR_FORWARD_PIN: u8 = 19;

/// GPIO pin number for right motor backward direction (BIN2)
#[allow(dead_code)]
pub const RIGHT_MOTOR_BACKWARD_PIN: u8 = 20;

/// PWM top value for 16-bit resolution (maximum duty cycle)
pub const PWM_TOP: u16 = 65535;

/// Initial PWM compare value for channel A (starts at 0)
pub const PWM_INITIAL_COMPARE_A: u16 = 0;

/// Initial PWM compare value for channel B (starts at 0)
pub const PWM_INITIAL_COMPARE_B: u16 = 0;
