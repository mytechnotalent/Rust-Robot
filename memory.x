/*
 * Copyright (c) 2025 Kevin Thomas
 * Licensed under the MIT License. See LICENSE file in the project root for full license information.
 *
 * RP2350 Linker Script
 * ====================
 * 
 * This linker script defines the memory layout and section placement for the
 * RP2350 microcontroller. It orchestrates how code and data are organized
 * in flash and RAM at link time.
 *
 * Hardware: RP2350 (ARM Cortex-M33)
 * - Flash: 4MB starting at 0x10000000
 * - RAM: 512KB starting at 0x20000000 (SRAM0-SRAM7, striped mapping)
 * - SRAM8: 4KB starting at 0x20080000 (direct mapping)
 * - SRAM9: 4KB starting at 0x20081000 (direct mapping)
 *
 * Memory Regions:
 * - FLASH: Non-volatile program storage starting at 0x10000000
 * - RAM: General purpose SRAM with striped mapping for performance
 * - SRAM8/SRAM9: Dedicated memory banks with direct mapping
 */

/* 
 * Memory Region Definitions
 * -------------------------
 * Defines the physical memory available to the application
 */
MEMORY
{
    /* Flash memory: Executable code and read-only data */
    FLASH : ORIGIN = 0x10000000, LENGTH = 4096K
    
    /* RAM: Striped mapping across SRAM0-SRAM7 for optimal performance */
    RAM : ORIGIN = 0x20000000, LENGTH = 512K
    
    /* SRAM8: Direct-mapped memory bank 8 for dedicated use */
    SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K
    
    /* SRAM9: Direct-mapped memory bank 9 for dedicated use */
    SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K
}

SECTIONS {
    /*
     * Boot ROM Info Section
     * ---------------------
     * Goes after .vector_table, to keep it in the first 4K of flash
     * where the Boot ROM (and picotool) can find it
     */
    .start_block : ALIGN(4)
    {
        __start_block_addr = .;
        KEEP(*(.start_block));
        KEEP(*(.boot_info));
    } > FLASH

} INSERT AFTER .vector_table;

/* Move .text to start after the boot info */
_stext = ADDR(.start_block) + SIZEOF(.start_block);

SECTIONS {
    /*
     * Picotool Binary Info Entries
     * ----------------------------
     * Picotool looks through this block (as we have pointers to it in our
     * header) to find interesting information
     */
    .bi_entries : ALIGN(4)
    {
        /* We put this in the header */
        __bi_entries_start = .;
        /* Here are the entries */
        KEEP(*(.bi_entries));
        /* Keep this block a nice round size */
        . = ALIGN(4);
        /* We put this in the header */
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;

SECTIONS {
    /*
     * Boot ROM Extra Info
     * -------------------
     * Goes after everything in our program, so it can contain a signature
     */
    .end_block : ALIGN(4)
    {
        __end_block_addr = .;
        KEEP(*(.end_block));
    } > FLASH

} INSERT AFTER .uninit;

/* Provide symbols for boot block calculations */
PROVIDE(start_to_end = __end_block_addr - __start_block_addr);
PROVIDE(end_to_start = __start_block_addr - __end_block_addr);