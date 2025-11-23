// Copyright (c) 2025 Kevin Thomas
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

//! NEC IR protocol decoder for infrared remote control.
//!
//! This module implements a complete NEC infrared protocol decoder that can
//! receive and validate 32-bit data frames from standard IR remote controls.
//!
//! # NEC Protocol Specification
//!
//! The NEC protocol uses pulse distance encoding with the following timing:
//!
//! - **Lead Pulse**: 9ms LOW
//! - **Space**: 4.5ms HIGH
//! - **Logical '0'**: 560µs LOW + 560µs HIGH
//! - **Logical '1'**: 560µs LOW + 1.69ms HIGH
//! - **Frame Format**: 8-bit address, 8-bit inverse address, 8-bit command, 8-bit inverse command
//!
//! # Data Frame Structure
//!
//! ```text
//! [Lead Pulse][Space][Address][~Address][Command][~Command]
//!    9ms LOW   4.5ms   8 bits    8 bits    8 bits   8 bits
//! ```
//!
//! # Validation
//!
//! The decoder validates:
//! - Lead pulse timing (8-10ms)
//! - Space timing (3.5-5ms)
//! - Bit timing (200-2500µs)
//! - Address/inverse checksum
//! - Command/inverse checksum
//!
//! # Examples
//!
//! ```ignore
//! use ir_receiver::IrReceiver;
//!
//! let ir = IrReceiver::new(ir_pin);
//!
//! if let Some(command) = ir.read_command() {
//!     match command {
//!         0x18 => println!("Forward"),
//!         0x52 => println!("Backward"),
//!         _ => println!("Unknown command"),
//!     }
//! }
//! ```

use embassy_rp::gpio::{Input, Level};
use embassy_time::{Duration, Instant};

/// NEC IR protocol receiver and decoder.
///
/// Implements state machine for decoding NEC infrared protocol commands
/// from a standard IR receiver module (active LOW output).
///
/// # Hardware Requirements
///
/// - IR receiver module (38kHz carrier frequency)
/// - Output connected to RP2350 GPIO with pull-up resistor
/// - Receiver output is active LOW (pulls LOW when IR detected)
pub struct IrReceiver<'a> {
    /// GPIO input pin connected to IR receiver module
    pin: &'a Input<'a>,
}

impl<'a> IrReceiver<'a> {
    /// Creates a new IR receiver instance.
    ///
    /// # Arguments
    ///
    /// * `pin` - GPIO input configured with pull-up resistor
    ///
    /// # Returns
    ///
    /// A new `IrReceiver` instance ready to decode commands.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let ir_pin = Input::new(p.PIN_5, Pull::Up);
    /// let ir = IrReceiver::new(&ir_pin);
    /// ```
    pub fn new(pin: &'a Input<'a>) -> Self {
        Self { pin }
    }

    /// Reads and decodes a single NEC IR command.
    ///
    /// Attempts to decode one complete NEC protocol frame from the IR receiver.
    /// This is a blocking operation that waits for IR activity.
    ///
    /// # Returns
    ///
    /// * `Some(command)` - Successfully decoded 8-bit command byte
    /// * `None` - Invalid frame, timeout, or checksum mismatch
    ///
    /// # Timing
    ///
    /// A complete NEC frame takes approximately 67.5ms to transmit.
    /// This method will block for up to ~150ms waiting for a valid frame.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// if let Some(cmd) = ir.read_command() {
    ///     println!("Received command: 0x{:02X}", cmd);
    /// }
    /// ```
    pub fn read_command(&self) -> Option<u8> {
        // Wait for leading pulse (9ms low)
        self.wait_for_level(Level::Low, 150000)?;
        let lead_pulse_time = self.wait_for_level(Level::High, 12000)?;

        // Validate lead pulse timing (8-10ms)
        if lead_pulse_time < 8000 || lead_pulse_time > 10000 {
            return None;
        }

        // Wait for space (4.5ms high)
        let space_time = self.wait_for_level(Level::Low, 7000)?;

        // Validate space timing (3.5-5ms)
        if space_time < 3500 || space_time > 5000 {
            return None;
        }

        // Decode 32 data bits
        let mut data = [0u8; 4];
        for i in 0..32 {
            // Wait for 560µs low period
            self.wait_for_level(Level::High, 1000)?;

            // Measure high period to determine bit value
            let high_time = self.wait_for_level(Level::Low, 2500)?;

            // Validate minimum timing
            if high_time < 200 {
                return None;
            }

            let byte_idx = i / 8;
            let bit_pos = i % 8;

            // High > 1.2ms indicates logical '1'
            if high_time > 1200 {
                data[byte_idx] |= 1 << bit_pos;
            }
            // Otherwise it's logical '0' (already 0 by default)
        }

        // Validate checksums
        // Address + ~Address should equal 0xFF
        // Command + ~Command should equal 0xFF
        if data[0].wrapping_add(data[1]) == 0xFF && data[2].wrapping_add(data[3]) == 0xFF {
            Some(data[2]) // Return command byte
        } else {
            None // Checksum validation failed
        }
    }

    /// Waits for IR pin to reach specified logic level with timeout.
    ///
    /// Polls the IR receiver pin until it matches the target level or the
    /// timeout expires. Used internally for measuring pulse widths.
    ///
    /// # Arguments
    ///
    /// * `level` - Target logic level to wait for (High or Low)
    /// * `timeout_us` - Maximum wait time in microseconds
    ///
    /// # Returns
    ///
    /// * `Some(duration)` - Time elapsed in microseconds when level was reached
    /// * `None` - Timeout occurred before target level was reached
    ///
    /// # Performance
    ///
    /// This method busy-waits (polls) the GPIO pin, which consumes CPU cycles
    /// but provides accurate timing measurements necessary for IR decoding.
    fn wait_for_level(&self, level: Level, timeout_us: u64) -> Option<u64> {
        let start = Instant::now();
        let timeout = Duration::from_micros(timeout_us);

        loop {
            // Read current pin state
            let current_level = if self.pin.is_high() {
                Level::High
            } else {
                Level::Low
            };

            // Check if target level reached
            if current_level == level {
                return Some(start.elapsed().as_micros());
            }

            // Check for timeout
            if start.elapsed() > timeout {
                return None;
            }
        }
    }
}
