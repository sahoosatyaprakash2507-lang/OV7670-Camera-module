# Image Capture Using ESP32 and OV7670

A camera image-acquisition project that captures pixel data from an
**OV7670 camera module** using an **ESP32**, transfers the captured
frame over USB serial, and reconstructs the image on a computer using
**Python, NumPy, and OpenCV**.

> **Architecture note:** In this implementation, an external FPGA
> supplies the OV7670 clock (XCLK) and configures the camera registers
> over SCCB. The ESP32 handles pixel-data acquisition and serial frame
> transfer. This is not a standalone ESP32 camera-initialization
> project.

## Table of Contents

-   [Overview](#overview)
-   [System Architecture](#system-architecture)
-   [Features](#features)
-   [Hardware Requirements](#hardware-requirements)
-   [Wiring](#wiring)
-   [Frame Format](#frame-format)
-   [Software Requirements](#software-requirements)
-   [Getting Started](#getting-started)
-   [Serial Commands](#serial-commands)
-   [Python Image Conversion](#python-image-conversion)
-   [Troubleshooting](#troubleshooting)
-   [Limitations](#limitations)
-   [Possible Improvements](#possible-improvements)
-   [Project Files](#project-files)
-   [Contributing](#contributing)
-   [License](#license)

## Overview

The OV7670 produces a parallel pixel-data stream accompanied by
synchronization and pixel-clock signals. The ESP32 samples this stream
and stores a reduced image in an RGB565 frame buffer.

The camera's configured sensor raster is **320 × 240 (QVGA)**. The
firmware stores every second source row and every second source column,
producing a **160 × 120** image. Each stored pixel uses 16-bit RGB565
encoding.

The ESP32 prints the frame as hexadecimal text over serial. A Python
utility parses the dump, decodes RGB565 pixels, and saves a standard
image file such as PNG.

## System Architecture

``` text
                 SCCB configuration + XCLK
             +-------------------------------+
             |                               v
        +---------+                     +---------+
        |   FPGA  |-------------------->| OV7670  |
        +---------+                     +---------+
                                             |
                              D0-D7, PCLK, HREF, VSYNC
                                             |
                                             v
                                        +---------+
                                        |  ESP32  |
                                        | Capture |
                                        +---------+
                                             |
                                      USB Serial
                                  RGB565 hex frame dump
                                             |
                                             v
                                    +----------------+
                                    | Python utility |
                                    | NumPy + OpenCV |
                                    +----------------+
                                             |
                                             v
                                         PNG image
```

### Main processing stages

1.  **Camera setup:** The FPGA generates XCLK and configures the OV7670
    through SCCB.
2.  **Frame synchronization:** The ESP32 uses VSYNC to identify a frame
    and HREF to identify active lines.
3.  **Pixel sampling:** The ESP32 reads the eight camera data bits in
    synchronization with PCLK.
4.  **Frame storage:** The firmware combines two bytes into each RGB565
    pixel and stores every second row and column.
5.  **Serial transfer:** The ESP32 sends frame metadata and hexadecimal
    pixel values.
6.  **Image reconstruction:** Python parses the dump, converts RGB565 to
    color pixels, and writes an image file.

## Features

-   OV7670 parallel camera data acquisition.
-   ESP32 GPIO sampling using VSYNC, HREF, and PCLK.
-   FPGA-based camera clock and SCCB configuration.
-   QVGA sensor input with a 160 × 120 stored frame.
-   16-bit RGB565 pixel format.
-   Serial frame dump at 115200 baud.
-   Python conversion from a serial dump or saved text file to an image.
-   Optional byte-alignment, duplicate-row, blur, and rotation
    processing in the supplied Python utilities.
-   Firmware diagnostics and serial commands for capture and frame
    inspection.

## Hardware Requirements

-   ESP32 development board compatible with the supplied Arduino sketch.
-   OV7670 camera module.
-   FPGA board/design that provides the camera XCLK and SCCB
    configuration.
-   USB cable for ESP32 programming and serial communication.
-   Jumper wires.
-   Computer with Python installed.

### Power and electrical note

The supplied wiring notes specify **3.3 V** for the camera and warn
against applying 5 V. Check the specifications of your exact camera
breakout and development boards before wiring. Connect the grounds of
the ESP32, FPGA, and camera system together.

## Wiring

The following mapping is taken from the supplied ESP32 firmware.

  OV7670 signal     ESP32 pin Description
  --------------- ----------- ------------------------
  D0                   GPIO 4 Pixel data bit 0
  D1                   GPIO 5 Pixel data bit 1
  D2                  GPIO 18 Pixel data bit 2
  D3                  GPIO 19 Pixel data bit 3
  D4                  GPIO 36 Pixel data bit 4
  D5                  GPIO 39 Pixel data bit 5
  D6                  GPIO 34 Pixel data bit 6
  D7                  GPIO 35 Pixel data bit 7
  PCLK                GPIO 25 Pixel clock
  VSYNC               GPIO 27 Frame synchronization
  HREF                GPIO 14 Active-line indication
  GND                     GND Common ground
  3V3                   3.3 V Camera power

Additional connections described by the firmware:

  ------------------------------------------------------------------------
  Signal                  Connected to            Description
  ----------------------- ----------------------- ------------------------
  SIOC / SIOD             FPGA                    SCCB camera
                                                  configuration

  XCLK                    FPGA                    Camera master clock

  PWDN / RESET            FPGA                    Camera control, as
                                                  implemented in the FPGA
                                                  design

  `cfg_done` (optional)   ESP32 GPIO 23           FPGA
                                                  configuration-complete
                                                  status
  ------------------------------------------------------------------------

**Important:** GPIO 34, 35, 36, and 39 are input-only pins on the
classic ESP32. The firmware uses them as camera data inputs.

The ESP32 does not drive the OV7670 SIOC, SIOD, or XCLK signals in this
design. Make sure the FPGA configuration and wiring match the firmware's
expected camera mode and timing.

## Frame Format

  Parameter                                 Value
  ----------------------------------------- --------------
  Camera raster described by the firmware   320 × 240
  Stored frame                              160 × 120
  Pixel format                              RGB565
  Bits per pixel                            16
  Stored pixel count                        19,200
  Frame buffer size                         38,400 bytes
  Serial baud rate                          115200

Each RGB565 pixel is represented by two bytes:

-   5 bits for red
-   6 bits for green
-   5 bits for blue

The firmware prints each pixel as four hexadecimal characters, with the
high byte first. Each image row therefore contains 160 × 4 = **640
hexadecimal characters**, excluding line endings.

A frame dump follows this general structure:

``` text
---BEGIN FRAME---
WIDTH=160
HEIGHT=120
FORMAT=RGB565
<hexadecimal pixel data, one row per line>
...
---END FRAME---
```

The dump ends with:

``` text
---END FRAME---
```

## Software Requirements

### ESP32 firmware

-   Arduino IDE or another Arduino-compatible build environment.
-   ESP32 board support package compatible with the supplied sketch.
-   The supplied firmware file: `ov7670_esp32_565.ino`.

The firmware source targets the **Arduino-ESP32 Core 3.x** API.

### Python tools

-   Python 3
-   NumPy
-   OpenCV
-   pySerial (needed for direct serial capture)

Install the dependencies:

``` bash
python -m pip install numpy opencv-python pyserial
```

For conversion of an already-saved dump file, pySerial is not required.

## Getting Started

### 1. Connect the hardware

Wire the OV7670 data and synchronization signals to the ESP32 using the
table above. Connect the FPGA signals required for camera configuration
and XCLK. Verify the voltage requirements and common ground before
powering the system.

### 2. Configure the FPGA

Program the FPGA with the design used by your hardware setup. It must
provide the OV7670 clock and configure the camera registers over SCCB.

The FPGA design/source is a separate part of the system; it is not
included in the ESP32 and Python source files described here. Camera
register settings and exact XCLK frequency therefore depend on the FPGA
implementation.

### 3. Upload the ESP32 firmware

Open `ov7670_esp32_565.ino` in the Arduino environment, select the
correct ESP32 board and serial port, and upload the sketch.

The firmware starts serial communication at **115200 baud**. It waits
for the FPGA configuration-complete signal when that option is enabled;
otherwise it uses its configured startup delay.

### 4. Capture a frame

Open a serial terminal at 115200 baud. Use the firmware commands
described below. If automatic capture on boot is enabled in the sketch,
the ESP32 captures a frame during startup.

### 5. Convert the frame to an image

For direct serial capture, close any serial monitor using the same port,
then run the relevant Python script. For a saved frame dump, use the
file-conversion option supported by the selected script.

## Serial Commands

The ESP32 firmware provides these commands:

  -----------------------------------------------------------------------
  Command                             Action
  ----------------------------------- -----------------------------------
  `c`                                 Capture a frame and print frame
                                      statistics.

  `p`                                 Print the first 20 pixels with RGB
                                      channel information.

  `d`                                 Dump the complete stored frame as
                                      hexadecimal RGB565 rows.

  `w`                                 Wait for FPGA configuration
                                      completion again, or use the
                                      configured delay fallback.

  `h`                                 Display command help.
  -----------------------------------------------------------------------

Send the command through a serial terminal configured for 115200 baud.

## Python Image Conversion

The repository includes Python utilities for reading the ESP32 frame
dump and reconstructing an image. The supplied variants have different
defaults, so check the selected script's command-line help and options
before running it.

### Convert a saved dump

The converter supports a file-based workflow. The general form is:

``` bash
python dump_to_image.py --file frame_dump.txt output.png
```

Here, `frame_dump.txt` is a text file containing the complete frame
markers, metadata, and hexadecimal pixel rows.

### Capture from the ESP32 serial port

The live-capture workflow opens the serial port, requests a capture,
asks the firmware to dump the frame, and collects the marker-delimited
output.

A typical invocation is:

``` bash
python dump_to_image.py COM3
```

Or specify an output filename:

``` bash
python dump_to_image.py COM3 captured.png
```

Replace `COM3` with the port assigned to the ESP32. On Linux, a port may
look like `/dev/ttyUSB0` or `/dev/ttyACM0`; on macOS, it may look like
`/dev/cu.usbserial-XXXX`.

Close the Arduino Serial Monitor or any other application using the same
serial port before starting live capture.

### RGB565 decoding

For each 16-bit pixel value, the decoder extracts the channels:

``` python
red   = (pixel >> 11) & 0x1F
green = (pixel >> 5)  & 0x3F
blue  =  pixel        & 0x1F
```

The channel values are expanded to 8-bit color values for image output.
The Python implementation accounts for OpenCV's BGR channel ordering
when constructing the image.

### Image correction options

Depending on the selected utility and its parameters, the scripts
include processing options such as:

-   **Byte alignment:** `none`, `shift1`, or automatic per-row
    alignment.
-   **Duplicate-row detection:** identifies possible repeated vertical
    scan-line patterns.
-   **Duplicate-row removal:** optionally removes detected repeated rows
    and resizes the result.
-   **Gaussian blur:** applies a 5 × 5 blur in the supplied
    implementation.
-   **180-degree rotation:** changes image orientation when required.
-   **Output resizing:** the supplied script variants use different
    default output dimensions.

These are post-processing measures for inspecting or mitigating capture
artefacts. They do not correct the underlying camera timing or wiring.

## Troubleshooting

  -----------------------------------------------------------------------
  Symptom                             Checks
  ----------------------------------- -----------------------------------
  VSYNC timeout                       Check GPIO 27 wiring, FPGA
                                      configuration, camera power, and
                                      synchronization output.

  HREF timeout                        Check GPIO 14 wiring and the
                                      camera's configured output mode.

  PCLK timeout                        Check GPIO 25, FPGA XCLK
                                      generation, camera clock
                                      configuration, and signal
                                      integrity.

  Image is mostly black               Check the data bus connections,
                                      common ground, camera
                                      configuration, and pixel sampling.

  Image is mostly white               Check for floating or incorrectly
                                      connected data pins.

  All pixels have the same value      Check PCLK synchronization and
                                      whether the ESP32 is sampling valid
                                      camera data.

  Colors look incorrect               Verify RGB565 byte order and
                                      RGB/BGR channel conversion.

  Image has shifted rows or columns   Inspect byte alignment and
                                      synchronization timing.

  Repeated horizontal bands           Inspect HREF handling and line
                                      capture timing.

  Python cannot open the serial port  Close other serial applications and
                                      verify the port name and
                                      permissions.

  Incomplete frame dump               Check the serial connection,
                                      capture/dump timeouts, and the
                                      beginning/end frame markers.
  -----------------------------------------------------------------------

## Limitations

-   An external FPGA is required for the camera clock and SCCB
    configuration in this implementation.
-   The stored image is 160 × 120, not the full 320 × 240 sensor raster.
-   Pixel acquisition uses ESP32 GPIO polling and is sensitive to PCLK
    and synchronization timing.
-   Interrupt masking is used during time-critical portions of each
    line; timing disturbances may still affect pixels.
-   Serial hexadecimal output is larger than packed binary pixel data
    and takes additional transfer time.
-   Python correction routines cannot restore pixel information that was
    never captured correctly.
-   The supplied source set does not include the FPGA HDL/configuration,
    a complete board-specific bill of materials, or measured
    image-quality and frame-rate results.

## Possible Improvements

-   Investigate hardware-assisted parallel capture or DMA for more
    deterministic pixel sampling.
-   Evaluate packed binary serial transfer to reduce transmission
    overhead.
-   Add frame length checks and checksums to detect corrupted or
    incomplete transfers.
-   Improve synchronization and pixel-clock timing validation.
-   Add configurable image dimensions and pixel formats, ensuring the
    FPGA camera configuration and ESP32 capture logic remain consistent.
-   Use suitable external memory, such as PSRAM on a compatible board,
    if a larger frame buffer is required.
-   Add sample output images and a verified wiring diagram to the
    repository.

## Project Files

The source set used for this documentation contains the following files:

  -----------------------------------------------------------------------
  File                                Purpose
  ----------------------------------- -----------------------------------
  `ov7670_esp32_565.ino`              ESP32 camera pixel acquisition,
                                      frame buffer, diagnostics, serial
                                      commands, and RGB565 dump.

  `dump_to_image.py`                  Python frame-dump conversion
                                      utility with live serial and
                                      saved-file workflows.

  `dump_to_image_160.py`              Converter variant with native 160 ×
                                      120 output default.

  `dump_to_image_320 - Copy.py`       Converter variant with 320 × 240
                                      output default.
  -----------------------------------------------------------------------

If you rename the Python files for a public repository, update the
example commands above to match the names you commit.

## Contributing

Contributions, bug reports, wiring corrections, and improvements to
capture reliability are welcome. When submitting a change, include:

-   ESP32 board model and Arduino-ESP32 core version.
-   FPGA board and camera clock/configuration details.
-   The wiring or pin changes.
-   A sample frame dump and resulting image, where possible.
-   Steps to reproduce any issue.

