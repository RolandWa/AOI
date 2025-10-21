# DIY AOI PCB Inspection System

## Overview

This project is a low-cost, DIY Automated Optical Inspection (AOI) system for PCBs, focused on detecting soldering errors, missing/mismatched components (e.g., 0402 passives, 0.5mm pitch ICs), and generating root-cause reports for process tuning (stencil printing, component placement, reflow soldering). It uses computer vision with OpenCV, adaptive lighting for contrast optimization, and Raspberry Pi Pico for hardware control. The system is designed for small batches (up to 100 boards) and supports various solder mask colors via adaptive RGB tuning.

Key features:
- Defect detection with operator feedback GUI.
- SN/QR recognition and reporting.
- Gerber/BOM/placement parsing for design-aware inspection.
- Adaptive RGB lighting and polarizer for glare reduction and contrast (fine-tuned to solder mask).
- Root-cause analysis for manufacturing improvements.

## Requirements

### Hardware
- **Camera**: OBSBOT Meet 2 4K USB webcam (45x36x22mm, ~40g, f/1.8 lens). Mount on a tripod 50-100mm above the PCB stage for macro view.
- **Lighting**: 3x WS2812 RGB LED rings (66mm outer diameter, 24 LEDs each, 5V, ~1.4A max). Configuration:
  - Top ring: Centered around camera lens for uniform illumination.
  - Left ring: 30mm above PCB edge, tilted 45° inward, ~100mm from center.
  - Right ring: Mirror of left, for cross-lighting.
  - Dimensions: Each ring 66mm dia., 2.75mm LED spacing; chain on single data pin.
  - Power: External 5V supply; connect GND/5V/data (GP0 on Pico).

- **Motorized Polarizer**: 28BYJ-48 stepper motor (ULN2003 driver) with 60mm diameter wheel holding 50x50mm linear polarizer sheet (slots for 0°/45°/90°). Dimensions: Stepper 28mm dia., 19mm height; wheel 60mm dia., 120° steps.
  - Wiring: IN1-4 to GP1-4 (Pico), enable to GP5, 5V/GND to driver.

- **Microcontroller**: Raspberry Pi Pico (~$5).
- **PC**: Windows laptop for running Python scripts, connected via USB to Pico and camera.
- **Other**: Tripod/jig for stationary positioning (3D-printable frame ~200x200mm base), USB-C cable, 5V power supply.

Overall setup with PC:




Light configuration with dimensions:





### Software
- **Python**: 3.8.x (download from [python.org](https://www.python.org/downloads/release/python-380/)).
  - Installation: Run the installer, check "Add Python 3.8 to PATH", install for all users.

- **Dependencies**: Install via `pip install -r requirements.txt` (see file in repo).
- **MicroPython**: v1.19.1 for Pico (download UF2 from [micropython.org](https://micropython.org/download/rp2-pico/)).
- **IDE**: Thonny for Pico programming ([thonny.org](https://thonny.org/)).

## Installation
1. **Clone Repo**: `git clone <repo_url>` or download ZIP.
2. **Install Python 3.8**: Run installer, add to PATH.
3. **Install Deps**: `cd aoi_project`, `pip install -r requirements.txt`.
4. **Setup Pico**: Flash MicroPython 1.19.1 UF2 (hold BOOTSEL, copy UF2 to RPI-RP2 drive).
5. **Program Pico**: Open Thonny, connect Pico, open `pico_aoi_control.py`, save as `main.py` on Pico.


## Hardware Setup
1. **Camera**: Mount OBSBOT Meet 2 on tripod, 50-100mm above PCB. Connect to PC USB.
2. **Lights**: Chain 3 WS2812 rings (top around camera, left/right tilted 45°). Connect data to GP0, GND/5V external.
3. **Polar Filter**: Attach stepper to 60mm wheel with polarizer sheet. Mount on camera lens barrel. Connect to ULN2003, then to Pico GP1-5.
4. **PCB Stage**: Flat surface/jig for PCBs; align fiducials under camera.
5. **PC Connection**: Pico USB to PC (serial COM port), camera USB.

## Running the System
1. Update paths in `aoi_inspector.py` (golden images, Gerber, BOM, pos).
2. Run `python aoi_inspector.py` for inspection (calibrates on golden, inspects boards).
3. Use `python root_cause_analyzer.py` for reports.

## Troubleshooting
- Pico not responding? Check COM port, 5V supply.
- Low contrast? Re-run calibration with different golden.
- Dependencies issues? Use virtualenv: `python -m venv env`, `env\Scripts\activate`, `pip install -r requirements.txt`. 

For questions, open an issue on GitHub.