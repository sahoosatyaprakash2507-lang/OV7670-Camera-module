// ============================================================
//  OV7670 (no FIFO) + ESP32  --  RGB565 diagnostic sketch
//  Arduino-ESP32 Core 3.x
// ============================================================
//
//  WHAT CHANGED vs. the original sketch
//  ------------------------------------
//  1. ov7670Read() no longer uses a repeated START. SCCB is not
//     I2C: the OV7670 does not support repeated START, so a
//     register read must be two separate transactions with a
//     full STOP in between. This alone was enough to wedge the
//     bus and make every later write return error=4.
//  2. PWDN and RESET are now driven explicitly with a correct
//     power-up sequence. Floating PWDN (pulled high) puts the
//     camera to sleep and kills SCCB entirely.
//  3. XCLK is started BEFORE any SCCB traffic. The OV7670's
//     SCCB block is clocked from XCLK; with no XCLK it ignores
//     you completely.
//  4. Added an I2C bus-recovery routine (9 clock pulses + STOP)
//     that frees a slave holding SDA low, plus an address
//     scanner that reports Arduino's endTransmission() codes.
//  5. Fixed the scaling config. The original set QVGA divider
//     values (DCWCTR=0x11, PCLK_DIV=0xF1) while sizing the
//     buffer for QQVGA, and never wrote COM3 (0x0C) or COM14
//     (0x3E) at all -- so scaling was disabled outright.
//  6. Dropped the writes to 0x67/0x68 (MANU/MANV). Those set a
//     fixed U/V value in YUV mode and do nothing useful in
//     RGB565. Replaced with a real colour matrix.
//  7. Capture now uses direct GPIO register reads instead of
//     digitalRead(), disables interrupts per line, and the
//     pixel clock is slowed way down so bit-banging is
//     actually feasible. See the CLKRC note below.
//  8. Added a serial command menu, a register dump, frame
//     statistics, a hex frame dump, and the built-in colour-bar
//     test pattern.
//  9. Added a verilogRegs table (COM6, MVFP, CHLF, COM12, GFIX,
//     REG74, and the B0-B3 black-level block) ported from a
//     known-good full-VGA Verilog config ROM. The ROM's clock,
//     scaler-disable, and VGA-window registers were deliberately
//     NOT ported -- they target full-speed PCLK / no scaling,
//     which is incompatible with this sketch's QQVGA + slow-
//     CLKRC bit-banging strategy. See the comment above
//     verilogRegs[] for the full reasoning.
//
//  READ THIS BEFORE YOU EXPECT PRETTY PICTURES
//  -------------------------------------------
//  Bit-banging an OV7670 from an ESP32 is a diagnostic tool,
//  not a camera. Even with everything below tuned, you get
//  well under 1 fps and occasional torn lines. If you want a
//  working camera, use the esp32-camera driver, which drives
//  the parallel bus over I2S with DMA and supports the OV7670.
//  Use this sketch to prove your wiring and SCCB are sane,
//  then switch.
//
//  WIRING
//  ------
//    OV7670        ESP32       Note
//    ------        -----       ----
//    3V3           3V3         3.3V ONLY. Never 5V.
//    GND           GND         Common ground is mandatory.
//    SIOC (SCL)    GPIO22      4.7k pull-up to 3V3
//    SIOD (SDA)    GPIO21      4.7k pull-up to 3V3
//    XCLK          GPIO32
//    PCLK          GPIO25
//    VSYNC         GPIO27
//    HREF          GPIO14
//    PWDN          GPIO26      or tie to GND
//    RESET         GPIO33      or tie to 3V3
//    D0            GPIO4
//    D1            GPIO5
//    D2            GPIO18
//    D3            GPIO19
//    D4            GPIO36      input-only pin, fine for data
//    D5            GPIO39      input-only
//    D6            GPIO34      input-only
//    D7            GPIO35      input-only
//
//  Many bare OV7670 breakouts have NO pull-ups on SIOC/SIOD.
//  The ESP32's internal pull-ups are ~45k and are marginal at
//  best. Add real 4.7k resistors before blaming the software.
//
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include "soc/gpio_struct.h"

// ------------------------------------------------------------
// Control pins
// ------------------------------------------------------------
#define SIOD_PIN    21
#define SIOC_PIN    22

#define XCLK_PIN    32
#define PCLK_PIN    25
#define VSYNC_PIN   27
#define HREF_PIN    14

// Set either of these to -1 if you hard-wired the pin
// (PWDN to GND, RESET to 3V3) instead of driving it.
#define PWDN_PIN    26
#define RESET_PIN   33

// ------------------------------------------------------------
// Data pins  D0..D7
// ------------------------------------------------------------
#define D0_PIN       4
#define D1_PIN       5
#define D2_PIN      18
#define D3_PIN      19
#define D4_PIN      36
#define D5_PIN      39
#define D6_PIN      34
#define D7_PIN      35

// ------------------------------------------------------------
// SCCB
// ------------------------------------------------------------
#define OV7670_ADDR 0x21          // 7-bit. 0x42 write / 0x43 read.
#define SCCB_FREQ   100000

// ------------------------------------------------------------
// Frame size (QQVGA)
// ------------------------------------------------------------
#define FRAME_WIDTH   160
#define FRAME_HEIGHT  120

// ------------------------------------------------------------
// XCLK frequency
//
// The datasheet says 10-48 MHz. In practice most OV7670s run
// well below that. We keep XCLK at a legal 10 MHz and instead
// slow the INTERNAL clock down with CLKRC, which is what
// actually sets PCLK.
// ------------------------------------------------------------
#define XCLK_FREQ   10000000

// ------------------------------------------------------------
// CLKRC (0x11) -- internal clock prescaler.
//
//   bit[6] = 1 : use XCLK directly, no prescale
//   bit[5:0]   : F_internal = F_XCLK / (value + 1)
//
// IMPORTANT: OmniVision's own QQVGA scaler table (Implementation
// Guide, Table 2-2) is calibrated around CLKRC = 0x01, not a
// heavily-divided clock. Earlier revisions of this sketch used
// 0x0F (divide by 16) on the theory that a slower PCLK is
// always easier to bit-bang -- that turned out to be wrong.
// Underclocking the sensor's INTERNAL clock this far can stop
// its DSP/scaler stage from generating PCLK correctly at all,
// even though VSYNC/HREF (coarser, more robust signals) keep
// toggling normally. If you see HREF/VSYNC detected fine but
// PCLK simply never appears, an over-slow CLKRC is a likely
// cause -- try a FASTER value, not a slower one.
//
//   CLKRC  internal(10MHz XCLK)  PCLK (x/4 scaler)   half-period
//   0x00   10.0 MHz              2.5  MHz             200 ns
//   0x01    5.0 MHz  (datasheet default for this table)
//                                 1.25 MHz             400 ns
//   0x03    2.5 MHz              625  kHz              800 ns
//   0x07    1.25 MHz             312  kHz              1.6 us
//
// Start at 0x01. With direct-register GPIO reads (not
// digitalRead) on a 240 MHz ESP32 this is comfortably fast
// enough to sample. If captures come through torn/glitchy
// rather than timing out completely, that's the point to raise
// this toward 0x03 for more margin -- but don't chase it past
// about 0x07; you're back in "may break the scaler" territory.
// ------------------------------------------------------------
#define CLKRC_VALUE  0x01

// ------------------------------------------------------------
// Options
// ------------------------------------------------------------
#define AUTO_CAPTURE_ON_BOOT  true

// ------------------------------------------------------------
// Fast GPIO masks (all of these are < GPIO32, so GPIO.in)
// ------------------------------------------------------------
#define PCLK_MASK   (1UL << PCLK_PIN)
#define VSYNC_MASK  (1UL << VSYNC_PIN)
#define HREF_MASK   (1UL << HREF_PIN)

// ------------------------------------------------------------
// Frame buffer: 160 * 120 * 2 = 38400 bytes
// ------------------------------------------------------------
uint16_t frameBuffer[FRAME_WIDTH * FRAME_HEIGHT];

bool testPatternOn = false;

// ------------------------------------------------------------
// Forward declarations. The Arduino IDE generates these for
// you; PlatformIO and plain .cpp builds do not.
// ------------------------------------------------------------
const char *i2cErrorName(uint8_t code);
bool ov7670Write(uint8_t reg, uint8_t value);
bool ov7670Read(uint8_t reg, uint8_t &value);

// ============================================================
//  REGISTER TABLES
// ============================================================

struct RegVal {
    uint8_t reg;
    uint8_t val;
};

#define REG_TABLE_END 0xFF

// ------------------------------------------------------------
// Core configuration: output format, window, scaling, clock.
// Everything here is load-bearing. Change it and the capture
// geometry changes with it.
// ------------------------------------------------------------
static const RegVal coreRegs[] = {
    // --- clock ---
    { 0x11, CLKRC_VALUE },   // CLKRC

    // --- output format ---
    { 0x3A, 0x04 },          // TSLB
    { 0x12, 0x04 },          // COM7  : RGB output, VGA base
    { 0x04, 0x00 },          // COM1  : disable CCIR656 embedded sync codes
    { 0x8C, 0x00 },          // RGB444: disabled
    { 0x40, 0xD0 },          // COM15 : RGB565, full 00-FF range

    // --- sync behaviour ---
    // COM10 bit[5] = 1 -> PCLK does NOT toggle during the
    // horizontal blanking interval. Essential for bit-banging:
    // without it you have to count blanking clocks yourself.
    { 0x15, 0x20 },          // COM10

    // --- windowing (VGA reference window) ---
    { 0x17, 0x13 },          // HSTART
    { 0x18, 0x01 },          // HSTOP
    { 0x32, 0xB6 },          // HREF
    { 0x19, 0x02 },          // VSTART
    { 0x1A, 0x7A },          // VSTOP
    { 0x03, 0x0A },          // VREF

    // --- scaling to QQVGA (VGA / 4) ---
    // COM3 bit[2] enables the scaler. Some modules also want
    // bit[3] (DCW enable) set, i.e. 0x0C -- if your image comes
    // out as a cropped VGA corner rather than a downscaled
    // full frame, try 0x0C here.
    { 0x0C, 0x04 },          // COM3  : enable scaling
    { 0x3E, 0x1A },          // COM14 : manual scaling, PCLK /4
    { 0x70, 0x3A },          // SCALING_XSC
    { 0x71, 0x35 },          // SCALING_YSC
    { 0x72, 0x22 },          // SCALING_DCWCTR   : /4 H, /4 V
    { 0x73, 0xF2 },          // SCALING_PCLK_DIV : /4
    { 0xA2, 0x02 },          // SCALING_PCLK_DELAY

    { REG_TABLE_END, REG_TABLE_END }
};

// ------------------------------------------------------------
// Image quality: AEC/AGC, AWB, gamma, colour matrix.
// None of this affects whether capture works -- only how the
// picture looks. Safe to trim if you want a minimal sketch.
// ------------------------------------------------------------
static const RegVal qualityRegs[] = {
    // --- freeze the auto algorithms while we configure ---
    { 0x13, 0xE0 },          // COM8: AGC/AWB/AEC off
    { 0x00, 0x00 },          // GAIN
    { 0x10, 0x00 },          // AECH
    { 0x0D, 0x40 },          // COM4
    { 0x14, 0x18 },          // COM9: 4x gain ceiling

    // --- AEC/AGC control window ---
    { 0xA5, 0x05 }, { 0xAB, 0x07 }, { 0x24, 0x95 }, { 0x25, 0x33 },
    { 0x26, 0xE3 }, { 0x9F, 0x78 }, { 0xA0, 0x68 }, { 0xA1, 0x03 },
    { 0xA6, 0xD8 }, { 0xA7, 0xD8 }, { 0xA8, 0xF0 }, { 0xA9, 0x90 },
    { 0xAA, 0x94 },
    { 0x13, 0xE5 },          // COM8: AGC + AEC back on

    // --- gamma curve ---
    { 0x7A, 0x20 }, { 0x7B, 0x10 }, { 0x7C, 0x1E }, { 0x7D, 0x35 },
    { 0x7E, 0x5A }, { 0x7F, 0x69 }, { 0x80, 0x76 }, { 0x81, 0x80 },
    { 0x82, 0x88 }, { 0x83, 0x8F }, { 0x84, 0x96 }, { 0x85, 0xA3 },
    { 0x86, 0xAF }, { 0x87, 0xC4 }, { 0x88, 0xD7 }, { 0x89, 0xE8 },

    // --- auto white balance ---
    { 0x43, 0x0A }, { 0x44, 0xF0 }, { 0x45, 0x34 }, { 0x46, 0x58 },
    { 0x47, 0x28 }, { 0x48, 0x3A }, { 0x59, 0x88 }, { 0x5A, 0x88 },
    { 0x5B, 0x44 }, { 0x5C, 0x67 }, { 0x5D, 0x49 }, { 0x5E, 0x0E },
    { 0x6C, 0x0A }, { 0x6D, 0x55 }, { 0x6E, 0x11 }, { 0x6F, 0x9F },
    { 0x6A, 0x40 },
    { 0x01, 0x40 },          // blue gain
    { 0x02, 0x60 },          // red gain
    { 0x13, 0xE7 },          // COM8: AGC + AWB + AEC all on

    // --- colour matrix (this is what 0x67/0x68 should have been) ---
    { 0x4F, 0xB3 },          // MTX1
    { 0x50, 0xB3 },          // MTX2
    { 0x51, 0x00 },          // MTX3
    { 0x52, 0x3D },          // MTX4
    { 0x53, 0xA7 },          // MTX5
    { 0x54, 0xE4 },          // MTX6
    { 0x58, 0x9E },          // MTXS

    // --- denoise / edge / misc ---
    { 0x41, 0x08 },          // COM16: AWB gain enable
    { 0x3F, 0x00 },          // EDGE
    { 0x75, 0x05 }, { 0x76, 0xE1 }, { 0x4C, 0x00 }, { 0x77, 0x01 },
    { 0x3D, 0xC0 },          // COM13: gamma enable, UV auto
    { 0x4B, 0x09 }, { 0xC9, 0x60 }, { 0x56, 0x40 },
    { 0x34, 0x11 },
    { 0x3B, 0x02 },          // COM11: banding filter
    { 0xA4, 0x89 },
    { 0x96, 0x00 }, { 0x97, 0x30 }, { 0x98, 0x20 }, { 0x99, 0x30 },
    { 0x9A, 0x84 }, { 0x9B, 0x29 }, { 0x9C, 0x03 }, { 0x9D, 0x4C },
    { 0x9E, 0x3F }, { 0x78, 0x04 },

    { REG_TABLE_END, REG_TABLE_END }
};

// ------------------------------------------------------------
// Registers ported from a known-good OV7670 Verilog config ROM
// (a full-VGA, no-scaling FPGA reference design). Most of that
// ROM either duplicates what's already in coreRegs/qualityRegs
// above or sets a VGA geometry and full-speed CLKRC that is
// incompatible with QQVGA + slow-clock bit-banging -- those
// parts (CLKRC=0x80, COM3=0x00, COM14=0x00, the VGA HSTART/
// HSTOP/HREF/VSTART/VSTOP window, and the passthrough scaling
// registers 0x70-0x73/0xA2) were deliberately NOT carried over,
// since they'd break the geometry this sketch already relies
// on. What's here is genuinely new: timing/black-level/mirror
// registers the existing tables never touched.
//
// COM12 (no HREF during VBLANK) is worth watching in particular
// -- a spurious HREF pulse while VSYNC is low is a plausible
// explanation for a capture landing on half the real lines
// (each buffer row ending up a near-duplicate of another).
// ------------------------------------------------------------
static const RegVal verilogRegs[] = {
    { 0x0F, 0x41 },          // COM6   : reset timing generator
    { 0x1E, 0x00 },          // MVFP   : mirror/flip both off
    { 0x33, 0x0B },          // CHLF
    { 0x3C, 0x78 },          // COM12  : no HREF when VSYNC is low
    { 0x69, 0x00 },          // GFIX   : fix gain control
    { 0x74, 0x00 },          // REG74  : digital gain control
    { 0xB0, 0x84 },          // RSVD   : required for correct color (per source ROM)
    { 0xB1, 0x0C },          // ABLC1
    { 0xB2, 0x0E },          // RSVD
    { 0xB3, 0x80 },          // THL_ST

    { REG_TABLE_END, REG_TABLE_END }
};

// ============================================================
//  I2C BUS RECOVERY
//
//  If a slave was interrupted mid-byte it can sit there holding
//  SDA low forever, which makes the ESP32 unable to issue a
//  START -- that is exactly what endTransmission() error 4
//  looks like. Clocking SCL up to 9 times lets the slave finish
//  its byte and release the line.
// ============================================================
bool recoverI2CBus()
{
    Wire.end();

    pinMode(SIOD_PIN, INPUT_PULLUP);
    pinMode(SIOC_PIN, INPUT_PULLUP);
    delayMicroseconds(10);

    if (digitalRead(SIOD_PIN) == HIGH) {
        Serial.println("Bus check: SDA is free.");
        return true;
    }

    Serial.println("Bus check: SDA stuck LOW -- clocking it out...");

    pinMode(SIOC_PIN, OUTPUT_OPEN_DRAIN);
    digitalWrite(SIOC_PIN, HIGH);

    for (int i = 0; i < 9; i++) {
        digitalWrite(SIOC_PIN, LOW);
        delayMicroseconds(5);
        digitalWrite(SIOC_PIN, HIGH);
        delayMicroseconds(5);

        if (digitalRead(SIOD_PIN) == HIGH) {
            break;
        }
    }

    // Manual STOP: SDA low -> high while SCL is high.
    pinMode(SIOD_PIN, OUTPUT_OPEN_DRAIN);
    digitalWrite(SIOD_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SIOC_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SIOD_PIN, HIGH);
    delayMicroseconds(5);

    pinMode(SIOD_PIN, INPUT_PULLUP);
    pinMode(SIOC_PIN, INPUT_PULLUP);
    delayMicroseconds(10);

    bool freed = (digitalRead(SIOD_PIN) == HIGH);

    Serial.println(freed
        ? "Bus recovered."
        : "Bus STILL stuck. This is a hardware problem: check "
          "pull-ups, check for a short to GND, check that SIOC "
          "and SIOD are not swapped.");

    return freed;
}

void startSCCB()
{
    Wire.begin(SIOD_PIN, SIOC_PIN, SCCB_FREQ);
    Wire.setTimeOut(50);
    delay(10);
}

// ============================================================
//  SCCB WRITE
// ============================================================
bool ov7670Write(uint8_t reg, uint8_t value)
{
    for (int attempt = 0; attempt < 3; attempt++) {

        Wire.beginTransmission(OV7670_ADDR);
        Wire.write(reg);
        Wire.write(value);

        uint8_t error = Wire.endTransmission(true);

        if (error == 0) {
            // The OV7670 needs settling time after a register
            // write. Skimping here causes intermittent,
            // maddening failures.
            delayMicroseconds(200);
            return true;
        }

        if (attempt == 2) {
            Serial.print("SCCB WRITE FAIL  reg 0x");
            if (reg < 0x10) Serial.print("0");
            Serial.print(reg, HEX);
            Serial.print("  val 0x");
            if (value < 0x10) Serial.print("0");
            Serial.print(value, HEX);
            Serial.print("  err=");
            Serial.print(error);
            Serial.print("  (");
            Serial.print(i2cErrorName(error));
            Serial.println(")");
            return false;
        }

        delay(2);
    }

    return false;
}

// ============================================================
//  SCCB READ
//
//  Two separate transactions. NO repeated start -- the OV7670
//  does not implement it. This was the original bug.
// ============================================================
bool ov7670Read(uint8_t reg, uint8_t &value)
{
    // Phase 1: write the register address, full STOP.
    Wire.beginTransmission(OV7670_ADDR);
    Wire.write(reg);

    if (Wire.endTransmission(true) != 0) {
        return false;
    }

    delayMicroseconds(200);

    // Phase 2: separate read transaction.
    if (Wire.requestFrom((int)OV7670_ADDR, (int)1) != 1) {
        return false;
    }

    value = Wire.read();
    return true;
}

// ============================================================
//  Human-readable endTransmission() codes
// ============================================================
const char *i2cErrorName(uint8_t code)
{
    switch (code) {
        case 0: return "ok";
        case 1: return "data too long for buffer";
        case 2: return "NACK on address - nothing is answering";
        case 3: return "NACK on data";
        case 4: return "bus error - line stuck or START failed";
        case 5: return "timeout";
        default: return "unknown";
    }
}

// ============================================================
//  I2C SCANNER
// ============================================================
void scanI2C()
{
    Serial.println();
    Serial.println("--- I2C scan ---");

    int found = 0;
    int busErrors = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {

        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission(true);

        if (err == 0) {
            Serial.print("  device at 0x");
            if (addr < 0x10) Serial.print("0");
            Serial.print(addr, HEX);
            if (addr == OV7670_ADDR) {
                Serial.print("   <-- OV7670");
            }
            Serial.println();
            found++;
        }
        else if (err == 4) {
            busErrors++;
        }

        delay(2);
    }

    Serial.print("Scan complete: ");
    Serial.print(found);
    Serial.print(" device(s), ");
    Serial.print(busErrors);
    Serial.println(" bus error(s).");

    if (found == 0 && busErrors > 100) {
        Serial.println();
        Serial.println("Every address returned a bus error.");
        Serial.println("The bus itself is broken, not the camera:");
        Serial.println("  - missing or wrong-value pull-ups");
        Serial.println("  - SDA or SCL shorted to GND");
        Serial.println("  - SIOC and SIOD swapped");
    }
    else if (found == 0) {
        Serial.println();
        Serial.println("Bus is electrically fine, nothing answered.");
        Serial.println("The camera is not responding:");
        Serial.println("  - PWDN high (camera asleep)");
        Serial.println("  - RESET held low");
        Serial.println("  - no XCLK reaching the sensor");
        Serial.println("  - no 3.3V, or no common ground");
    }

    Serial.println();
}

// ============================================================
//  XCLK
// ============================================================
bool startXCLK()
{
    Serial.println("Starting XCLK...");

    // Detach first in case this is a re-init (e.g. the 'R'
    // command) and XCLK is already attached to this pin from a
    // previous call. Core 3.x refuses to ledcAttach() a pin
    // that's already attached, so this makes the function safe
    // to call more than once. If nothing is attached yet this
    // is a harmless no-op.
    ledcDetach(XCLK_PIN);

    // Core 3.x: ledcAttach(pin, freq, resolution_bits)
    // At 1-bit resolution the duty range is 0..2, so duty=1
    // gives a 50% square wave.
    bool ok = ledcAttach(XCLK_PIN, XCLK_FREQ, 1);

    if (!ok) {
        Serial.println("ERROR: ledcAttach failed. The requested");
        Serial.println("frequency may be too high for the chosen");
        Serial.println("resolution.");
        return false;
    }

    ledcWrite(XCLK_PIN, 1);
    delay(50);

    Serial.print("XCLK = ");
    Serial.print(XCLK_FREQ / 1000000.0, 2);
    Serial.println(" MHz");

    return true;
}

// ============================================================
//  POWER / RESET SEQUENCE
//
//  Order matters. PWDN must be released before RESET, and XCLK
//  must already be running when RESET is released, or the
//  sensor's internal state machine never starts.
// ============================================================
void powerUpCamera()
{
    // Every step below prints AND flushes before the next GPIO
    // operation. If the board stalls or browns out partway
    // through, whatever printed last tells us exactly which
    // line was the trigger -- no guessing.
    Serial.println("Power-up sequence...");
    Serial.flush();

#if PWDN_PIN >= 0
    Serial.println("  [1] pinMode(PWDN, OUTPUT)...");
    Serial.flush();
    pinMode(PWDN_PIN, OUTPUT);
    Serial.println("  [2] digitalWrite(PWDN, HIGH)...");
    Serial.flush();
    digitalWrite(PWDN_PIN, HIGH);        // powered down
    Serial.println("  [2] ok, PWDN held HIGH (powered down)");
    Serial.flush();
#endif

#if RESET_PIN >= 0
    Serial.println("  [3] pinMode(RESET, OUTPUT)...");
    Serial.flush();
    pinMode(RESET_PIN, OUTPUT);
    Serial.println("  [4] digitalWrite(RESET, LOW)...");
    Serial.flush();
    digitalWrite(RESET_PIN, LOW);        // held in reset
    Serial.println("  [4] ok, RESET held LOW");
    Serial.flush();
#endif

    delay(10);
    Serial.println("  [5] past first delay(10)");
    Serial.flush();

#if PWDN_PIN >= 0
    digitalWrite(PWDN_PIN, LOW);         // wake up
    Serial.println("  PWDN released");
    Serial.flush();
    delay(10);
#else
    Serial.println("  PWDN not driven - make sure it is tied LOW");
#endif

#if RESET_PIN >= 0
    delay(5);
    digitalWrite(RESET_PIN, HIGH);       // release reset
    Serial.println("  RESET released");
    Serial.flush();
#else
    Serial.println("  RESET not driven - make sure it is tied HIGH");
#endif

    // The sensor needs time to boot before it will ACK.
    Serial.println("  [6] waiting 200ms for sensor boot...");
    Serial.flush();
    delay(200);
    Serial.println("  [7] powerUpCamera() done");
    Serial.flush();
}

// ============================================================
//  APPLY A REGISTER TABLE
// ============================================================
int applyRegs(const RegVal *table, const char *label)
{
    int failures = 0;
    int count = 0;

    for (int i = 0; table[i].reg != REG_TABLE_END; i++) {
        if (!ov7670Write(table[i].reg, table[i].val)) {
            failures++;
        }
        count++;
    }

    Serial.print("  ");
    Serial.print(label);
    Serial.print(": ");
    Serial.print(count - failures);
    Serial.print("/");
    Serial.print(count);
    Serial.println(" registers written");

    return failures;
}

// ============================================================
//  TEST PATTERN
//
//  On the OV7670 the test pattern lives in the top bit of the
//  two scaling registers:
//    XSC[7]=0 YSC[7]=0 -> off
//    XSC[7]=1 YSC[7]=0 -> 8-bar colour bar
//    XSC[7]=1 YSC[7]=1 -> fade-to-grey colour bar
//
//  This is the single most useful debugging tool you have: it
//  produces known pixel values with no lens, no light and no
//  exposure involved. If the bars come out clean, your data
//  bus and capture timing are correct and any remaining
//  problem is optical or exposure-related.
// ============================================================
void setTestPattern(bool on)
{
    testPatternOn = on;

    if (on) {
        ov7670Write(0x70, 0x3A | 0x80);   // XSC bit7 = 1
        ov7670Write(0x71, 0x35 & ~0x80);  // YSC bit7 = 0
        Serial.println("Test pattern: 8-bar colour bar ON");
    } else {
        ov7670Write(0x70, 0x3A);
        ov7670Write(0x71, 0x35);
        Serial.println("Test pattern: OFF (normal image)");
    }
}

// ============================================================
//  CONFIGURE OV7670
// ============================================================
bool configureOV7670()
{
    Serial.println();
    Serial.println("Configuring OV7670...");

    // Software reset. COM7 bit[7] returns every register to
    // its default, so this must come first and nothing before
    // it will survive.
    if (!ov7670Write(0x12, 0x80)) {
        Serial.println("Reset write failed - aborting config.");
        return false;
    }

    delay(150);

    int failures = 0;
    failures += applyRegs(coreRegs,    "core   ");
    failures += applyRegs(qualityRegs, "quality");
    failures += applyRegs(verilogRegs, "verilog");

    delay(100);

    if (failures > 0) {
        Serial.print("Configuration completed with ");
        Serial.print(failures);
        Serial.println(" failed write(s).");
        return false;
    }

    Serial.println("Configuration complete.");
    return true;
}

// ============================================================
//  DATA PINS
// ============================================================
void setupDataPins()
{
    pinMode(D0_PIN, INPUT);
    pinMode(D1_PIN, INPUT);
    pinMode(D2_PIN, INPUT);
    pinMode(D3_PIN, INPUT);
    pinMode(D4_PIN, INPUT);
    pinMode(D5_PIN, INPUT);
    pinMode(D6_PIN, INPUT);
    pinMode(D7_PIN, INPUT);

    pinMode(PCLK_PIN,  INPUT);
    pinMode(VSYNC_PIN, INPUT);
    pinMode(HREF_PIN,  INPUT);
}

// ============================================================
//  FAST BYTE READ
//
//  Eight digitalRead() calls cost several microseconds. Two
//  direct register reads cost tens of nanoseconds. The data
//  pins straddle the 32-bit boundary, so we need both GPIO.in
//  (GPIO0-31) and GPIO.in1 (GPIO32-39).
//
//    D0 = GPIO4   -> GPIO.in  bit 4
//    D1 = GPIO5   -> GPIO.in  bit 5
//    D2 = GPIO18  -> GPIO.in  bit 18
//    D3 = GPIO19  -> GPIO.in  bit 19
//    D4 = GPIO36  -> GPIO.in1 bit 4
//    D5 = GPIO39  -> GPIO.in1 bit 7
//    D6 = GPIO34  -> GPIO.in1 bit 2
//    D7 = GPIO35  -> GPIO.in1 bit 3
//
//  If you change the data pin assignment you MUST update this
//  function to match.
// ============================================================
static inline uint8_t IRAM_ATTR readCameraByte()
{
    uint32_t lo = GPIO.in;
    uint32_t hi = GPIO.in1.val;

    uint8_t v = 0;
    v |=  ((lo >>  4) & 0x01);
    v |= (((lo >>  5) & 0x01) << 1);
    v |= (((lo >> 18) & 0x01) << 2);
    v |= (((lo >> 19) & 0x01) << 3);
    v |= (((hi >>  4) & 0x01) << 4);
    v |= (((hi >>  7) & 0x01) << 5);
    v |= (((hi >>  2) & 0x01) << 6);
    v |= (((hi >>  3) & 0x01) << 7);

    return v;
}

// ============================================================
//  CAPTURE ONE FRAME
//
//  Interrupts are disabled per LINE rather than per frame. A
//  whole frame at these clock rates is a few hundred ms, which
//  would trip the interrupt watchdog; a single line is ~2 ms,
//  which is safe. The cost is that an interrupt can land in
//  the gap between lines and cost you a pixel or two at the
//  left edge. That is an acceptable trade for a test sketch.
// ============================================================
bool captureFrame()
{
    Serial.println();
    Serial.println("Waiting for VSYNC...");

    uint32_t t0 = millis();

    // Wait for the VSYNC pulse to go high (start of vertical blank)
    while ((GPIO.in & VSYNC_MASK) == 0) {
        if (millis() - t0 > 2000) {
            Serial.println("ERROR: VSYNC never went HIGH.");
            Serial.println("  No frames are being produced. Check");
            Serial.println("  XCLK, and check VSYNC is on GPIO27.");
            return false;
        }
    }

    // Wait for it to fall -- pixel data begins right after.
    while ((GPIO.in & VSYNC_MASK) != 0) {
        if (millis() - t0 > 2000) {
            Serial.println("ERROR: VSYNC stuck HIGH.");
            return false;
        }
    }

    Serial.println("VSYNC found. Capturing...");

    uint32_t captureStart = millis();
    int badLines = 0;
    int firstBadLine = -1;

    for (int y = 0; y < FRAME_HEIGHT; y++) {

        // ---- wait for HREF to rise (start of an active line) ----
        uint32_t hrefStart = micros();
        while ((GPIO.in & HREF_MASK) == 0) {
            if (micros() - hrefStart > 100000) {
                Serial.print("ERROR: HREF timeout at line ");
                Serial.println(y);
                Serial.println("  Got some lines but not all --");
                Serial.println("  likely the scaler config or the");
                Serial.println("  window registers do not match");
                Serial.println("  FRAME_HEIGHT.");
                return false;
            }
        }

        uint16_t *row = &frameBuffer[y * FRAME_WIDTH];
        bool lineOk = true;

        // ---- time-critical region ----
        portDISABLE_INTERRUPTS();

        for (int x = 0; x < FRAME_WIDTH; x++) {

            uint32_t guard;
            uint8_t hi, lo;

            // --- first byte (high byte of RGB565) ---
            guard = 0;
            while ((GPIO.in & PCLK_MASK) == 0) {
                if (++guard > 200000) { lineOk = false; break; }
            }
            if (!lineOk) break;

            hi = readCameraByte();

            guard = 0;
            while ((GPIO.in & PCLK_MASK) != 0) {
                if (++guard > 200000) { lineOk = false; break; }
            }
            if (!lineOk) break;

            // --- second byte (low byte) ---
            guard = 0;
            while ((GPIO.in & PCLK_MASK) == 0) {
                if (++guard > 200000) { lineOk = false; break; }
            }
            if (!lineOk) break;

            lo = readCameraByte();

            guard = 0;
            while ((GPIO.in & PCLK_MASK) != 0) {
                if (++guard > 200000) { lineOk = false; break; }
            }
            if (!lineOk) break;

            row[x] = ((uint16_t)hi << 8) | lo;
        }

        portENABLE_INTERRUPTS();
        // ---- end time-critical region ----

        if (!lineOk) {
            if (firstBadLine < 0) firstBadLine = y;
            badLines++;
            if (badLines > 5) {
                Serial.print("ERROR: PCLK timeout, too many bad ");
                Serial.print("lines (first bad line was ");
                Serial.print(firstBadLine);
                Serial.println(").");
                Serial.println("  PCLK is not toggling within the");
                Serial.println("  per-byte timeout. This can mean");
                Serial.println("  EITHER PCLK is too fast to sample");
                Serial.println("  (try a SMALLER CLKRC_VALUE) OR, if");
                Serial.println("  CLKRC_VALUE is already low (>=0x08),");
                Serial.println("  the internal clock is so slow the");
                Serial.println("  sensor's scaler has stopped driving");
                Serial.println("  PCLK correctly (try a LARGER value,");
                Serial.println("  back toward the datasheet default");
                Serial.println("  of 0x01). Also check PCLK wiring and");
                Serial.println("  try the 't' test pattern to rule out");
                Serial.println("  AEC/exposure timing as a factor.");
                return false;
            }
        }

        // ---- wait for HREF to fall (end of line) ----
        hrefStart = micros();
        while ((GPIO.in & HREF_MASK) != 0) {
            if (micros() - hrefStart > 100000) {
                Serial.print("ERROR: HREF stuck HIGH at line ");
                Serial.println(y);
                return false;
            }
        }
    }

    uint32_t elapsed = millis() - captureStart;

    Serial.print("Frame captured in ");
    Serial.print(elapsed);
    Serial.print(" ms");
    if (badLines) {
        Serial.print("  (");
        Serial.print(badLines);
        Serial.print(" degraded line(s))");
    }
    Serial.println();

    return true;
}

// ============================================================
//  FRAME STATISTICS
//
//  Tells you at a glance whether you captured an image or just
//  captured the wiring.
// ============================================================
void frameStats()
{
    uint16_t minV = 0xFFFF;
    uint16_t maxV = 0x0000;
    uint32_t zeros = 0;
    uint32_t ones  = 0;
    uint32_t total = FRAME_WIDTH * FRAME_HEIGHT;
    uint64_t sum   = 0;

    for (uint32_t i = 0; i < total; i++) {
        uint16_t p = frameBuffer[i];
        if (p < minV) minV = p;
        if (p > maxV) maxV = p;
        if (p == 0x0000) zeros++;
        if (p == 0xFFFF) ones++;
        sum += p;
    }

    Serial.println();
    Serial.println("--- frame statistics ---");
    Serial.print("  pixels      : "); Serial.println(total);
    Serial.print("  min / max   : 0x");
    Serial.print(minV, HEX); Serial.print(" / 0x");
    Serial.println(maxV, HEX);
    Serial.print("  mean        : ");
    Serial.println((uint32_t)(sum / total));
    Serial.print("  all-zero    : ");
    Serial.print(zeros);
    Serial.print(" ("); Serial.print(zeros * 100.0 / total, 1);
    Serial.println("%)");
    Serial.print("  all-ones    : ");
    Serial.print(ones);
    Serial.print(" ("); Serial.print(ones * 100.0 / total, 1);
    Serial.println("%)");

    Serial.println();
    if (zeros > total * 0.95) {
        Serial.println("  VERDICT: everything is zero. The data");
        Serial.println("  lines are not reaching the ESP32, or the");
        Serial.println("  sensor is outputting nothing. Check D0-D7");
        Serial.println("  wiring and try the test pattern ('t').");
    }
    else if (ones > total * 0.95) {
        Serial.println("  VERDICT: everything is 0xFFFF. Data pins");
        Serial.println("  are floating high or disconnected.");
    }
    else if (minV == maxV) {
        Serial.println("  VERDICT: every pixel identical. Sampling");
        Serial.println("  is happening but not in sync with PCLK.");
    }
    else {
        Serial.println("  VERDICT: varied data - this looks like a");
        Serial.println("  real capture. Dump it with 'd' and view");
        Serial.println("  it on your PC to confirm.");
    }
    Serial.println();
}

// ============================================================
//  PRINT FIRST PIXELS
// ============================================================
void printPixels(int count)
{
    Serial.println();
    Serial.print("First ");
    Serial.print(count);
    Serial.println(" RGB565 pixels (with R/G/B split):");

    for (int i = 0; i < count; i++) {
        uint16_t p = frameBuffer[i];

        uint8_t r = (p >> 11) & 0x1F;
        uint8_t g = (p >>  5) & 0x3F;
        uint8_t b =  p        & 0x1F;

        Serial.print("  [");
        if (i < 10) Serial.print(" ");
        Serial.print(i);
        Serial.print("] 0x");

        if (p < 0x1000) Serial.print("0");
        if (p < 0x0100) Serial.print("0");
        if (p < 0x0010) Serial.print("0");
        Serial.print(p, HEX);

        Serial.print("   R="); Serial.print(r);
        Serial.print(" G=");   Serial.print(g);
        Serial.print(" B=");   Serial.println(b);
    }
}

// ============================================================
//  DUMP THE WHOLE FRAME AS HEX
//
//  Capture the serial output to a file, strip the header and
//  footer lines, and convert it on your PC. See the note at
//  the bottom of this file.
// ============================================================
void dumpFrame()
{
    Serial.println();
    Serial.println("---BEGIN FRAME---");
    Serial.print("WIDTH=");  Serial.println(FRAME_WIDTH);
    Serial.print("HEIGHT="); Serial.println(FRAME_HEIGHT);
    Serial.println("FORMAT=RGB565");

    char buf[8];

    for (int y = 0; y < FRAME_HEIGHT; y++) {
        for (int x = 0; x < FRAME_WIDTH; x++) {
            sprintf(buf, "%04X", frameBuffer[y * FRAME_WIDTH + x]);
            Serial.print(buf);
        }
        Serial.println();
    }

    Serial.println("---END FRAME---");
    Serial.println();
}

// ============================================================
//  REGISTER DUMP
// ============================================================
void dumpRegisters()
{
    struct { uint8_t reg; const char *name; } regs[] = {
        { 0x0A, "PID    " }, { 0x0B, "VER    " },
        { 0x11, "CLKRC  " }, { 0x12, "COM7   " },
        { 0x0C, "COM3   " }, { 0x0D, "COM4   " },
        { 0x13, "COM8   " }, { 0x15, "COM10  " },
        { 0x3A, "TSLB   " }, { 0x3E, "COM14  " },
        { 0x40, "COM15  " }, { 0x8C, "RGB444 " },
        { 0x17, "HSTART " }, { 0x18, "HSTOP  " },
        { 0x32, "HREF   " }, { 0x19, "VSTART " },
        { 0x1A, "VSTOP  " }, { 0x03, "VREF   " },
        { 0x70, "SCL_XSC" }, { 0x71, "SCL_YSC" },
        { 0x72, "DCWCTR " }, { 0x73, "PCLK_DV" },
        { 0xA2, "PCLK_DL" },
        { 0x0F, "COM6   " }, { 0x1E, "MVFP   " },
        { 0x33, "CHLF   " }, { 0x3C, "COM12  " },
        { 0x69, "GFIX   " }, { 0x74, "REG74  " },
        { 0xB0, "RSVD_B0" }, { 0xB1, "ABLC1  " },
        { 0xB2, "RSVD_B2" }, { 0xB3, "THL_ST " },
    };

    Serial.println();
    Serial.println("--- register dump ---");

    for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
        uint8_t v;
        Serial.print("  ");
        Serial.print(regs[i].name);
        Serial.print(" (0x");
        if (regs[i].reg < 0x10) Serial.print("0");
        Serial.print(regs[i].reg, HEX);
        Serial.print(") = ");

        if (ov7670Read(regs[i].reg, v)) {
            Serial.print("0x");
            if (v < 0x10) Serial.print("0");
            Serial.println(v, HEX);
        } else {
            Serial.println("READ FAILED");
        }
    }
    Serial.println();
}

// ============================================================
//  IDENTIFY THE SENSOR
// ============================================================
bool identifySensor()
{
    Serial.println();
    Serial.println("Reading sensor ID...");

    uint8_t pid = 0, ver = 0;
    bool pidOk = ov7670Read(0x0A, pid);
    bool verOk = ov7670Read(0x0B, ver);

    if (!pidOk || !verOk) {
        Serial.println("  ID read FAILED.");
        return false;
    }

    Serial.print("  PID = 0x");
    if (pid < 0x10) Serial.print("0");
    Serial.print(pid, HEX);
    Serial.print("   VER = 0x");
    if (ver < 0x10) Serial.print("0");
    Serial.println(ver, HEX);

    if (pid == 0x76 && (ver == 0x73 || ver == 0x71)) {
        Serial.println("  Confirmed: OV7670.");
        return true;
    }

    Serial.println("  Unexpected ID. Expected PID=0x76, VER=0x73.");
    Serial.println("  The camera answers but may not be an OV7670,");
    Serial.println("  or the SCCB read path is marginal.");
    return false;
}

// ============================================================
//  HELP
// ============================================================
void printHelp()
{
    Serial.println();
    Serial.println("--- commands ---");
    Serial.println("  c  capture a frame + statistics");
    Serial.println("  p  print the first 20 pixels");
    Serial.println("  d  dump the whole frame as hex");
    Serial.println("  s  scan the I2C bus");
    Serial.println("  r  dump camera registers");
    Serial.println("  t  toggle the colour-bar test pattern");
    Serial.println("  i  read sensor ID");
    Serial.println("  R  full re-init (power cycle + configure)");
    Serial.println("  h  this help");
    Serial.println();
}

// ============================================================
//  FULL INITIALISATION
// ============================================================
bool initCamera()
{
    // 1. Make sure the bus is usable before we touch it.
    recoverI2CBus();
    startSCCB();

    // 2. XCLK must be running before the sensor leaves reset.
    if (!startXCLK()) {
        return false;
    }

    // 3. Power and reset sequence.
    powerUpCamera();

    // 4. Data pins.
    setupDataPins();

    // 5. Who is on the bus?
    scanI2C();

    // 6. Identify.
    if (!identifySensor()) {
        Serial.println();
        Serial.println("Cannot talk to the sensor. Stopping here --");
        Serial.println("there is no point configuring a camera that");
        Serial.println("is not answering. Work through the checklist");
        Serial.println("printed above, then send 'R' to retry.");
        return false;
    }

    // 7. Configure.
    if (!configureOV7670()) {
        return false;
    }

    // Give AEC/AGC a moment to settle before the first capture.
    delay(500);

    return true;
}

// ============================================================
//  SETUP
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(2000);

    setCpuFrequencyMhz(240);

    Serial.println();
    Serial.println("================================================");
    Serial.println("  ESP32 + OV7670 (no FIFO) -- RGB565 diagnostic");
    Serial.println("================================================");

#ifdef ESP_ARDUINO_VERSION_STR
    Serial.print("Arduino-ESP32 core : ");
    Serial.println(ESP_ARDUINO_VERSION_STR);
#endif
    Serial.print("CPU frequency      : ");
    Serial.print(getCpuFrequencyMhz());
    Serial.println(" MHz");
    Serial.print("Frame buffer       : ");
    Serial.print(sizeof(frameBuffer));
    Serial.println(" bytes");
    Serial.print("Free heap          : ");
    Serial.println(ESP.getFreeHeap());
    Serial.println();

    bool ok = initCamera();

    if (ok) {
        Serial.println();
        Serial.println("Camera ready.");

        if (AUTO_CAPTURE_ON_BOOT) {
            if (captureFrame()) {
                frameStats();
                printPixels(20);
            }
        }
    }

    printHelp();
}

// ============================================================
//  LOOP
// ============================================================
void loop()
{
    if (!Serial.available()) {
        delay(20);
        return;
    }

    char cmd = Serial.read();

    switch (cmd) {

        case 'c':
            if (captureFrame()) {
                frameStats();
            }
            break;

        case 'p':
            printPixels(20);
            break;

        case 'd':
            dumpFrame();
            break;

        case 's':
            scanI2C();
            break;

        case 'r':
            dumpRegisters();
            break;

        case 't':
            setTestPattern(!testPatternOn);
            delay(200);
            break;

        case 'i':
            identifySensor();
            break;

        case 'R':
            Serial.println();
            Serial.println("Re-initialising...");
            if (initCamera()) {
                Serial.println("Camera ready.");
            }
            break;

        case 'h':
            printHelp();
            break;

        case '\r':
        case '\n':
            break;

        default:
            Serial.print("Unknown command: '");
            Serial.print(cmd);
            Serial.println("'  (send 'h' for help)");
            break;
    }
}

// ============================================================
//  APPENDIX: turning a hex dump into a PNG
//
//  Capture the serial output of the 'd' command to a file, then:
//
//    from PIL import Image
//
//    lines = [l.strip() for l in open("dump.txt")]
//    start = lines.index("---BEGIN FRAME---")
//    end   = lines.index("---END FRAME---")
//    rows  = [l for l in lines[start+1:end] if len(l) == 640]
//
//    img = Image.new("RGB", (160, len(rows)))
//    for y, line in enumerate(rows):
//        for x in range(160):
//            p = int(line[x*4:(x+1)*4], 16)
//            r = ((p >> 11) & 0x1F) << 3
//            g = ((p >>  5) & 0x3F) << 2
//            b = ( p        & 0x1F) << 3
//            img.putpixel((x, y), (r, g, b))
//
//    img.save("frame.png")
//
//  If the colours look swapped, your byte order is reversed --
//  swap hi and lo in captureFrame(), or flip TSLB (0x3A) bit 3.
// ============================================================
