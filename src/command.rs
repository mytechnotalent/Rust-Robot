// Copyright (c) 2025 Kevin Thomas
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

//! Robot command definitions and IR remote control mappings.
//!
//! This module defines the command set for robot control and provides
//! mapping from IR remote button codes to robot actions.
//!
//! # IR Remote Button Layout
//!
//! The robot is designed to work with standard NEC protocol IR remotes.
//! Default button mappings:
//!
//! ```text
//!        [  ^  ]  0x18 - Forward
//!   [ < ][ ■ ][ > ]  0x08 - Left, 0x1C - Stop, 0x5A - Right
//!        [  v  ]  0x52 - Backward
//!
//!   Speed Control:
//!   [ + ]  0x15 - Increase speed (~10%)
//!   [ - ]  0x07 - Decrease speed (~10%)
//!   [ R ]  0x09 - Reset speed to 50%
//! ```
//!
//! # Command Codes
//!
//! All command codes are 8-bit values transmitted via NEC IR protocol:
//!
//! - **Motion Commands**: 0x18, 0x52, 0x08, 0x5A, 0x1C
//! - **Speed Commands**: 0x09, 0x15, 0x07

use defmt::Format;

/// Robot motion and control commands.
///
/// Represents all possible actions the robot can perform, including
/// directional movement and speed adjustments.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum Command {
    /// Move forward at current speed
    Forward,
    /// Move backward at current speed
    Backward,
    /// Turn left (counter-clockwise) at reduced speed
    Left,
    /// Turn right (clockwise) at reduced speed
    Right,
    /// Stop all motor movement
    Stop,
    /// Reset speed to default (50%)
    SpeedReset,
    /// Increase speed by approximately 10%
    SpeedUp,
    /// Decrease speed by approximately 10%
    SpeedDown,
    /// Unknown or unsupported command code
    Unknown,
}

impl Command {
    /// Converts an IR remote button code to a robot command.
    ///
    /// Maps NEC protocol command bytes to robot actions. Unknown codes
    /// are mapped to `Command::Unknown`.
    ///
    /// # Arguments
    ///
    /// * `code` - 8-bit NEC IR protocol command byte
    ///
    /// # Returns
    ///
    /// The corresponding `Command` enum variant.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let cmd = Command::from_ir_code(0x18);
    /// assert_eq!(cmd, Command::Forward);
    ///
    /// let unknown = Command::from_ir_code(0xFF);
    /// assert_eq!(unknown, Command::Unknown);
    /// ```
    pub fn from_ir_code(code: u8) -> Self {
        match code {
            0x18 => Command::Forward,
            0x52 => Command::Backward,
            0x08 => Command::Left,
            0x5A => Command::Right,
            0x1C => Command::Stop,
            0x09 => Command::SpeedReset,
            0x15 => Command::SpeedUp,
            0x07 => Command::SpeedDown,
            _ => Command::Unknown,
        }
    }

    /// Returns the IR code for this command.
    ///
    /// Provides reverse mapping from command to IR button code.
    /// Returns `None` for `Command::Unknown`.
    ///
    /// # Returns
    ///
    /// * `Some(code)` - The 8-bit IR command code
    /// * `None` - For unknown commands
    ///
    /// # Examples
    ///
    /// ```ignore
    /// assert_eq!(Command::Forward.to_ir_code(), Some(0x18));
    /// assert_eq!(Command::Unknown.to_ir_code(), None);
    /// ```
    #[allow(dead_code)]
    pub fn to_ir_code(&self) -> Option<u8> {
        match self {
            Command::Forward => Some(0x18),
            Command::Backward => Some(0x52),
            Command::Left => Some(0x08),
            Command::Right => Some(0x5A),
            Command::Stop => Some(0x1C),
            Command::SpeedReset => Some(0x09),
            Command::SpeedUp => Some(0x15),
            Command::SpeedDown => Some(0x07),
            Command::Unknown => None,
        }
    }
}
