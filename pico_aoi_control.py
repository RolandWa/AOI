# Pico AOI Controller Script - FIXED
# Version: 1.4.1
# Date: 2025-10-22
# Critical Fixes:
#   - Changed UART to pins 16/17 to avoid conflict with WS2812 on GP0
#   - Fixed version check logic
#   - Added error recovery and connection status feedback

import machine
import time
import neopixel
import sys

# Version check - fixed to check major.minor properly
version_parts = sys.version.split('.')
if len(version_parts) >= 2:
	major, minor = int(version_parts[0]), int(version_parts[1])
	if major < 1 or (major == 1 and minor < 19):
		print("Warning: MicroPython 1.19+ recommended.")

# Pins - FIXED: UART moved to avoid GP0 conflict
RING_PIN = machine.Pin(0, machine.Pin.OUT)  # WS2812 data for all 3 rings
STEPPER_PINS = [machine.Pin(i, machine.Pin.OUT) for i in range(1, 5)]  # IN1-4
ENABLE_PIN = machine.Pin(5, machine.Pin.OUT)  # Stepper enable

# UART now on GP16 (TX) and GP17 (RX) - no conflict with GP0
uart = machine.UART(0, baudrate=115200, tx=machine.Pin(16), rx=machine.Pin(17))

# Config
NUM_LEDS = 24 * 3  # 72 total LEDs (3 rings × 24 LEDs)
np = neopixel.NeoPixel(RING_PIN, NUM_LEDS)
STEPS_PER_REV = 2048  # 28BYJ-48 in half-step mode
POL_POSITIONS = [0, 120, 240]  # Polarizer angles

# Stepper sequence for smooth rotation
HALF_STEP_SEQ = [
	[1, 0, 0, 0], [1, 1, 0, 0], [0, 1, 0, 0], [0, 1, 1, 0],
	[0, 0, 1, 0], [0, 0, 1, 1], [0, 0, 0, 1], [1, 0, 0, 1]
]


def set_lights(r, g, b, ring_id=3):
	"""Set RGB ring(s) to custom color. ring_id: 0=top, 1=left, 2=right, 3=all."""
	# Validate inputs
	r = max(0, min(255, r))
	g = max(0, min(255, g))
	b = max(0, min(255, b))

	color = (r, g, b)

	if ring_id == 3:  # All rings
		start_idx, end_idx = 0, NUM_LEDS
	elif 0 <= ring_id <= 2:  # Specific ring
		start_idx = ring_id * 24
		end_idx = start_idx + 24
	else:
		return  # Invalid ring_id

	# Turn off all LEDs first for clean transitions
	for i in range(NUM_LEDS):
		np[i] = (0, 0, 0)

	# Set specified ring(s)
	for i in range(start_idx, end_idx):
		np[i] = color
	np.write()


def rotate_polarizer(target_deg):
	"""Rotate polarizer to nearest position (0, 120, 240 deg)."""
	# Clamp and find nearest position
	target_deg = target_deg % 360
	target_pos = min(POL_POSITIONS,
	                 key=lambda p: min(abs(p - target_deg), abs(p - target_deg + 360), abs(p - target_deg - 360)))

	steps = int((target_pos / 360) * STEPS_PER_REV)
	ENABLE_PIN.high()

	try:
		for _ in range(abs(steps)):
			for half_step in HALF_STEP_SEQ:
				for j, pin in enumerate(STEPPER_PINS):
					pin.value(half_step[j])
				time.sleep_ms(2)  # 500 steps/sec
	finally:
		# Always disable stepper to save power and reduce heat
		ENABLE_PIN.low()


def main():
	# Initialize - all LEDs off
	set_lights(0, 0, 0)

	# Send ready signal
	print("Pico AOI Controller v1.4.1 Ready")
	uart.write(b"PICO_READY\n")

	last_heartbeat = time.ticks_ms()

	while True:
		# Heartbeat every 5 seconds
		if time.ticks_diff(time.ticks_ms(), last_heartbeat) > 5000:
			uart.write(b"HEARTBEAT\n")
			last_heartbeat = time.ticks_ms()

		if uart.any():
			try:
				cmd = uart.readline().decode().strip()
				print(f"Received: {cmd}")

				if cmd.startswith('SEQ_'):
					parts = cmd.split('_')
					if len(parts) == 6:
						r, g, b, deg, ring = int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]), int(parts[5])

						# Validate parameters
						if 0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255 and ring in [0, 1, 2, 3]:
							set_lights(r, g, b, ring)
							rotate_polarizer(deg)
							time.sleep(0.5)  # Settle time for capture
							uart.write(b"SEQ_COMPLETE\n")
						else:
							uart.write(b"INVALID_PARAMS\n")
					else:
						uart.write(b"INVALID_FORMAT\n")

				elif cmd == 'PING':
					uart.write(b"PONG\n")

				elif cmd == 'STATUS':
					uart.write(b"OK\n")

				elif cmd == 'RESET':
					set_lights(0, 0, 0)
					uart.write(b"RESET_COMPLETE\n")

				else:
					uart.write(b"INVALID_CMD\n")

			except ValueError as e:
				print(f"Value error: {e}")
				uart.write(b"INVALID_VALUES\n")
			except Exception as e:
				print(f"Error: {e}")
				uart.write(b"ERROR\n")

		time.sleep_ms(50)  # Reduced polling interval


if __name__ == "__main__":
	main()