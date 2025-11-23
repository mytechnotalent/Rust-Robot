// Copyright (c) 2025 Kevin Thomas
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

//! Build script for the robot-embassy embedded application
//!
//! This build script configures the linker for the RP2350 microcontroller by:
//! - Copying the memory layout definition to the build output directory
//! - Setting up linker search paths for cortex-m-rt
//! - Configuring rebuild triggers for memory layout changes
//!
//! # Memory Layout
//! The `memory.x` file defines the RP2350's memory regions:
//! - **FLASH**: 4MB starting at 0x10000000 (program code storage)
//! - **RAM**: 512KB starting at 0x20000000 (runtime data)
//! - **SRAM8**: 4KB starting at 0x20080000 (dedicated memory bank 8)
//! - **SRAM9**: 4KB starting at 0x20081000 (dedicated memory bank 9)
//!
//! # Linker Integration
//! This script integrates with `cortex-m-rt` by providing the memory layout
//! through the `-Tlink.x` linker argument specified in `.cargo/config.toml`.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

/// Main build script entry point
///
/// Configures the linker environment for embedded RP2350 firmware builds.
///
/// # Process Flow
/// 1. Retrieves the cargo output directory from the `OUT_DIR` environment variable
/// 2. Copies `memory.x` to the output directory so the linker can find it
/// 3. Adds the output directory to the linker search path
/// 4. Registers `memory.x` as a build dependency for rebuild detection
///
/// # Panics
/// Panics if:
/// - The `OUT_DIR` environment variable is not set
/// - Unable to create or write to `memory.x` in the output directory
fn main() {
    // Get the output directory where cargo places build artifacts
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    
    // Copy memory.x to the output directory for the linker to find
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    
    // Tell cargo to add the output directory to the linker search path
    println!("cargo:rustc-link-search={}", out.display());
    
    // Rebuild if memory.x changes
    println!("cargo:rerun-if-changed=memory.x");
}
