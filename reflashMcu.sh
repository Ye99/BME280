#!/bin/bash
# Erase the ESP8266 and flash MicroPython firmware.
#
# The image is downloaded over HTTPS and checked against FIRMWARE_SHA256
# before anything is written to the board. Set FIRMWARE_SHA256 below (or in
# the environment) to the SHA-256 of an image you trust; if it is empty, the
# script stops and prints the hash of what it downloaded.
set -euo pipefail

FIRMWARE_URL="https://micropython.org/resources/firmware/esp8266-20191220-v1.12.bin"
FIRMWARE_SHA256="${FIRMWARE_SHA256:-}"
PORT="${PORT:-/dev/ttyUSB0}"
FIRMWARE=".esp8266firmware.bin"

curl -fsSL --proto '=https' -o "$FIRMWARE" "$FIRMWARE_URL"

if command -v sha256sum >/dev/null 2>&1; then
    actual=$(sha256sum "$FIRMWARE" | cut -d' ' -f1)
else
    actual=$(shasum -a 256 "$FIRMWARE" | cut -d' ' -f1)
fi

if [ -z "$FIRMWARE_SHA256" ]; then
    echo "FIRMWARE_SHA256 is not set; not flashing." >&2
    echo "Downloaded $FIRMWARE_URL with SHA-256 $actual" >&2
    echo "Check that hash against a source you trust, then set FIRMWARE_SHA256." >&2
    exit 1
fi

if [ "$actual" != "$FIRMWARE_SHA256" ]; then
    echo "SHA-256 mismatch: expected $FIRMWARE_SHA256, got $actual; not flashing." >&2
    rm -f "$FIRMWARE"
    exit 1
fi

esptool.py --port "$PORT" erase_flash
esptool.py --port "$PORT" --baud 460800 write_flash --flash_size=detect 0 "$FIRMWARE"
rshell -p "$PORT" --buffer-size=30
