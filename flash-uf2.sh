#!/bin/bash
# Custom cargo runner for RP2350 UF2 flashing via BOOTSEL mode

BINARY="$1"
UF2="${BINARY}.uf2"

# Picotool needs .elf extension, so create a symlink
ELF_LINK="${BINARY}.elf"
ln -sf "$(basename "$BINARY")" "$ELF_LINK"

# Convert ELF to UF2 using picotool
echo "📦 Converting to UF2..."
picotool uf2 convert "$ELF_LINK" "$UF2" --family rp2350-arm-s || exit 1

# Clean up symlink
rm -f "$ELF_LINK"

# Try to find and flash to mounted RP2350 drive
for vol in "/Volumes/RP2350" "/Volumes/RPI-RP2"; do
    if [ -d "$vol" ]; then
        echo "🚀 Flashing to $vol..."
        # Use -X to avoid extended attributes issues on FAT32
        cp -X "$UF2" "$vol/" && echo "✅ Flashed! Device will reset automatically." && exit 0
    fi
done

# No drive found
echo "⚠️  No RP2350 drive found. Created UF2 at: $UF2"
echo "   Manually copy it to your RP2350 drive"
exit 1
