# Pico AOI Controller Script
# Version: 1.4.0
# Date: 2025-10-21
# Author: Grok (xAI)
# Changelog:
#   - 1.0.0 (2025-10-21): Initial release for 3 WS2812 rings, 28BYJ-48 stepper
#   - 1.1.0 (2025-10-21): Added explicit top/left/right ring control (0/1/2/3), refined serial commands
#   - 1.2.0 (2025-10-21): Added BLUE color option for LED rings
#   - 1.3.0 (2025-10-21): Expanded to all basic RGB colors (added YELLOW, CYAN, MAGENTA, PURPLE, ORANGE)
#   - 1.4.0 (2025-10-21): Changed to accept arbitrary RGB values (SEQ_<R>_<G>_<B>_<DEG>_<RING>), removed predefined colors for flexibility
# Dependencies:
#   - MicroPython==1.19.1 (for RP2040)
#   - neopixel (built-in)
# Notes: Upload via Thonny as main.py. Connect rings to GP0 (top=0-23, left=24-47, right=48-71), stepper IN1-4 to GP1-4, enable to GP5.
# Commands: SEQ_<R>_<G>_<B>_<DEG>_<RING> (e.g., SEQ_255_0_0_0_1 for red, 0°, left ring)

import machine
import time
import neopixel
import sys

# Version check
if sys.version_info[2] != '1.19.1':
	print("Warning: MicroPython 1.19.1 recommended.")

# Pins
RING_PIN = machine.Pin(0, machine.Pin.OUT)  # WS2812 data for top/left/right rings
STEPPER_PINS = [machine.Pin(i, machine.Pin.OUT) for i in range(1, 5)]  # IN1-4 for 28BYJ-48
ENABLE_PIN = machine.Pin(5, machine.Pin.OUT)  # Stepper driver enable

# Config
NUM_LEDS = 24 * 3  # 24 LEDs per ring (top=0-23, left=24-47, right=48-71)
np = neopixel.NeoPixel(RING_PIN, NUM_LEDS)
STEPS_PER_REV = 2048  # 28BYJ-48 full steps (4*512 for 360°)
POL_POSITIONS = [0, 120, 240]  # Polarizer angles (0°, 45°, 90° mapped for wheel)


def set_lights(r, g, b, ring_id=3):
	"""Set RGB ring(s) to custom color. ring_id: 0=top, 1=left, 2=right, 3=all."""
	color = (r, g, b)
	if ring_id == 3:  # All rings
		start_idx, end_idx = 0, NUM_LEDS
	else:  # Specific ring
		start_idx = ring_id * 24
		end_idx = start_idx + 24

	# Turn off all LEDs first
	for i in range(NUM_LEDS):
		np[i] = (0, 0, 0)

	# Set specified ring(s)
	for i in range(start_idx, end_idx):
		np[i] = color
	np.write()


def rotate_polarizer(target_deg):
	"""Rotate polarizer to nearest position (0, 120, 240 deg)."""
	target_pos = min(POL_POSITIONS, key=lambda p: abs(p - target_deg))
	steps = int((target_pos / 360) * STEPS_PER_REV)  # Convert deg to steps
	ENABLE_PIN.high()

	# Half-step sequence for smoother rotation
	sequence = [
		[1, 0, 0, 0], [1, 1, 0, 0], [0, 1, 0, 0], [0, 1, 1, 0],
		[0, 0, 1, 0], [0, 0, 1, 1], [0, 0, 0, 1], [1, 0, 0, 1]
	]

	for _ in range(abs(steps)):
		for half_step in sequence:
			for j, pin in enumerate(STEPPER_PINS):
				pin.value(half_step[j])
			time.sleep_ms(2)  # 500 steps/sec
	ENABLE_PIN.low()


def main():
	uart = machine.UART(0, baudrate=115200, tx=machine.Pin(0), rx=machine.Pin(1))
	set_lights(0, 0, 0)  # Init off
	print("Pico AOI Controller v1.4.0 Ready. Commands: SEQ_<R>_<G>_<B>_<DEG>_<RING>")

	while True:
		if uart.any():
			try:
				cmd = uart.readline().decode().strip()
				print(f"Received: {cmd}")
				if cmd.startswith('SEQ_'):
					parts = cmd.split('_')
					if len(parts) == 6:
						r, g, b, deg, ring = int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]), int(parts[5])
						if 0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255 and ring in [0, 1, 2, 3]:
							set_lights(r, g, b, ring)
							rotate_polarizer(deg)
							time.sleep(1)  # Settle for capture
							uart.write(b"SEQ_COMPLETE\n")
						else:
							uart.write(b"INVALID_PARAMS\n")
					else:
						uart.write(b"INVALID_FORMAT\n")
				else:
					uart.write(b"INVALID_CMD\n")
			except ValueError:
				uart.write(b"INVALID_VALUES\n")  # Error-proof for non-int
			except Exception as e:
				print(f"Error: {e}")
				uart.write(b"ERROR\n")
		time.sleep(0.1)


if __name__ == "__main__":
	main()