"""
Convert an ESP32/OV7670 serial hex frame dump (RGB565, big-endian)
into a viewable image using numpy + OpenCV.

Expected input: a text log containing a block like

    ---BEGIN FRAME---
    WIDTH=160
    HEIGHT=120
    FORMAT=RGB565
    <hex row 0>
    <hex row 1>
    ...
    ---END FRAME---

where each hex row is WIDTH*4 hex characters (2 bytes / pixel, one
row of pixels per line, most-significant byte first).

This capture pipeline has a known bit-banging bug: the ESP32 capture
loop misses every other HREF pulse, so the reported HEIGHT rows are
actually only HEIGHT/2 unique scan lines, each one duplicated once
(row y and row y + HEIGHT/2 are near-identical). detect_duplicate_period()
confirms that automatically instead of assuming it, then only the
unique half is kept and stretched back to the original aspect ratio.
"""

"""
Convert an ESP32/OV7670 serial hex frame dump (RGB565, big-endian)
into a viewable image using numpy + OpenCV -- either from a saved
log file, or captured live over the serial port (e.g. COM3).

Expected frame format, whether read from a file or straight off
the serial port:

    ---BEGIN FRAME---
    WIDTH=160
    HEIGHT=120
    FORMAT=RGB565
    <hex row 0>
    <hex row 1>
    ...
    ---END FRAME---

where each hex row is WIDTH*4 hex characters (2 bytes / pixel, one
row of pixels per line, most-significant byte first).

This capture pipeline has a known bit-banging bug: the ESP32 capture
loop misses every other HREF pulse, so the reported HEIGHT rows are
actually only HEIGHT/2 unique scan lines, each one duplicated once
(row y and row y + HEIGHT/2 are near-identical). detect_duplicate_period()
confirms that automatically instead of assuming it, then only the
unique half is kept and stretched back to the original aspect ratio.

Requires: numpy, opencv-python, and (for live capture) pyserial.
    pip install numpy opencv-python pyserial
"""

import re
import sys
import time
import numpy as np
import cv2

try:
    import serial  # pyserial; only needed for live capture
except ImportError:
    serial = None


# ------------------------------------------------------------------
# Serial capture
# ------------------------------------------------------------------

def capture_frame_over_serial(port: str, baud: int = 115200,
                               timeout: float = 10.0) -> str:
    """
    Open the ESP32's serial port, send 'c' (capture) then 'd' (dump),
    and return the raw text between (and including) the
    ---BEGIN FRAME--- / ---END FRAME--- markers.

    port: e.g. "COM3" on Windows, "/dev/ttyUSB0" on Linux, "/dev/cu.SLAB_USBtoUART" on macOS.
    """
    if serial is None:
        raise RuntimeError("pyserial is required for live capture: pip install pyserial")

    with serial.Serial(port, baud, timeout=1) as ser:
        time.sleep(2)                # let the board finish its post-reset boot banner
        ser.reset_input_buffer()

        ser.write(b'c')              # trigger a capture
        time.sleep(1.0)              # captureFrame() takes real time (sensor readout)

        ser.write(b'd')              # ask for the hex dump

        lines = []
        started = False
        deadline = time.time() + timeout
        while time.time() < deadline:
            raw_line = ser.readline()
            if not raw_line:
                continue
            line = raw_line.decode("utf-8", errors="replace").rstrip("\r\n")
            if "BEGIN FRAME" in line:
                started = True
            if started:
                lines.append(line)
            if "END FRAME" in line:
                break
        else:
            raise TimeoutError(f"Did not see ---END FRAME--- within {timeout}s "
                                f"(got {len(lines)} lines)")

    return "\n".join(lines)


# ------------------------------------------------------------------
# Frame parsing / decoding (shared by file and serial sources)
# ------------------------------------------------------------------

def parse_frame_text(text: str):
    """Same job as load_frame_rows(), but works on an in-memory string."""
    lines = text.splitlines()
    start = next(i for i, l in enumerate(lines) if "BEGIN FRAME" in l)
    end = next(i for i, l in enumerate(lines) if "END FRAME" in l)

    width = height = None
    for l in lines[start:end]:
        if l.startswith("WIDTH="):
            width = int(l.split("=", 1)[1])
        elif l.startswith("HEIGHT="):
            height = int(l.split("=", 1)[1])

    hex_pattern = re.compile(r"^[0-9A-Fa-f]+$")
    rows = [l.strip() for l in lines[start + 1:end] if hex_pattern.fullmatch(l.strip())]

    if width is None:
        width = len(rows[0]) // 4
    if height is None:
        height = len(rows)

    return rows, width, height


def load_frame_rows(path: str):
    """Pull the hex rows + declared WIDTH/HEIGHT out of a saved dump log."""
    with open(path, "rb") as f:
        raw = f.read().decode("utf-8", errors="replace")
    return parse_frame_text(raw)


def rows_to_rgb565(rows, width, align: str = "none"):
    """
    Hex strings -> (H, W) uint16 array of raw big-endian RGB565 words.

    The OV7670 puts out 8 bits per PCLK; a full RGB565 pixel takes TWO
    consecutive PCLK edges (high byte, then low byte). If the capture
    loop ever samples one PCLK edge late -- easy to happen with a
    bit-banged digitalRead() loop -- every byte pair downstream of that
    point is shifted by one byte for the rest of the line: pixel N's
    high byte ends up paired with pixel N+1's low byte, etc. The
    result isn't noise, it's a smooth, detail-free blend of neighboring
    pixels, which is a very different failure from the line-doubling
    bug found earlier.

    HREF resets the byte-phase at the start of every line, so a
    mis-sample doesn't propagate frame-to-frame -- but it can differ
    row to row. align="auto" tries both phases (drop 0 or 1 leading
    byte) independently for each row and keeps whichever decodes to
    a sharper, more structured line (more high-frequency content =
    real edges rather than blended color). This is a best-effort
    software salvage, not a real fix -- the actual fix is making the
    firmware's capture loop sample every PCLK edge reliably (an
    interrupt-driven or hardware-timed read instead of polling with
    digitalRead()).

    align: "none" (assume byte-phase is always correct), "shift1"
    (always drop the first byte of every row), or "auto" (per-row,
    pick whichever phase looks sharper).
    """
    height = len(rows)
    buf = np.zeros((height, width), dtype=np.uint16)

    for y, row in enumerate(rows):
        raw = bytes.fromhex(row.ljust(width * 4 + 2, "0")[: width * 4 + 2])

        if align == "none":
            chosen = raw[: width * 2]
        elif align == "shift1":
            chosen = raw[1: width * 2 + 1]
        else:  # "auto"
            cand0 = raw[: width * 2]
            cand1 = raw[1: width * 2 + 1]
            cand1 = cand1.ljust(width * 2, b"\x00")
            chosen = cand0 if _line_sharpness(cand0) >= _line_sharpness(cand1) else cand1

        words = np.frombuffer(chosen.ljust(width * 2, b"\x00")[: width * 2], dtype=">u2")
        buf[y, :] = words

    return buf


def _line_sharpness(raw_bytes: bytes) -> float:
    """
    Rough per-line detail metric used to pick the better byte-phase
    alignment: mean absolute difference between adjacent decoded
    pixels. A byte-phase-shifted line blends neighbors together and
    scores lower; a correctly-phased line carrying real scene edges
    scores higher.
    """
    words = np.frombuffer(raw_bytes.ljust(len(raw_bytes) + (len(raw_bytes) % 2), b"\x00"),
                           dtype=">u2")
    if len(words) < 2:
        return 0.0
    diffs = np.abs(words[1:].astype(np.int32) - words[:-1].astype(np.int32))
    return float(diffs.mean())


def rgb565_to_bgr888(words: np.ndarray) -> np.ndarray:
    """Vectorized RGB565 -> 8-bit BGR (OpenCV's native channel order)."""
    r5 = (words >> 11) & 0x1F
    g6 = (words >> 5) & 0x3F
    b5 = words & 0x1F

    r8 = (r5 * 255 // 31).astype(np.uint8)
    g8 = (g6 * 255 // 63).astype(np.uint8)
    b8 = (b5 * 255 // 31).astype(np.uint8)

    return cv2.merge([b8, g8, r8])   # OpenCV wants B, G, R


def detect_duplicate_period(words: np.ndarray) -> int:
    """
    Find the row offset k that minimizes mean |row[y] - row[y+k]|,
    i.e. the vertical period the capture loop is duplicating at.
    Returns 0 if no strong duplication is found.
    """
    h = words.shape[0]
    signed = words.astype(np.int32)
    best_k, best_diff = 0, None
    for k in range(1, h // 2 + 1):
        diff = np.abs(signed[:-k] - signed[k:]).mean()
        if best_diff is None or diff < best_diff:
            best_diff, best_k = diff, k
    # Only treat it as real duplication if it's much tighter than the
    # neighbor-row baseline (k=1) -- otherwise it's just a smooth image.
    baseline = np.abs(signed[:-1] - signed[1:]).mean()
    return best_k if best_diff < 0.5 * baseline else 0


# ------------------------------------------------------------------
# Post-processing
# ------------------------------------------------------------------

def process_image(bgr: np.ndarray, gaussian_ksize: int = 5,
                   gaussian_sigma: float = 1.0, rotate_180: bool = True) -> np.ndarray:
    """Apply a Gaussian blur and (optionally) a 180-degree rotation."""
    out = cv2.GaussianBlur(bgr, (gaussian_ksize, gaussian_ksize), gaussian_sigma)
    if rotate_180:
        out = cv2.rotate(out, cv2.ROTATE_180)
    return out


# ------------------------------------------------------------------
# End-to-end pipelines
# ------------------------------------------------------------------

def frame_to_image(rows, width, height, dedupe: bool = True, align: str = "none") -> np.ndarray:
    """Shared core: hex rows -> full-size BGR image (before blur/rotate)."""
    words = rows_to_rgb565(rows, width, align=align)

    if dedupe:
        period = detect_duplicate_period(words)
        if period:
            words = words[:period]         # keep only the unique scan lines

    bgr = rgb565_to_bgr888(words)

    # Stretch back to the original WIDTH x HEIGHT frame geometry with
    # nearest-neighbor, so duplicated/missing lines don't get smoothed
    # into something that never came off the sensor.
    return cv2.resize(bgr, (width * 4, height * 4), interpolation=cv2.INTER_NEAREST)


def dump_to_image(dump_path: str, out_path: str = "frame_out.png",
                   dedupe: bool = True, align: str = "none", gaussian: bool = True,
                   rotate_180: bool = True) -> np.ndarray:
    """File-based pipeline (unchanged from before, plus the new post-processing)."""
    rows, width, height = load_frame_rows(dump_path)
    out = frame_to_image(rows, width, height, dedupe=dedupe, align=align)
    if gaussian or rotate_180:
        out = process_image(out, rotate_180=rotate_180) if gaussian else \
              (cv2.rotate(out, cv2.ROTATE_180) if rotate_180 else out)
    cv2.imwrite(out_path, out)
    return out


def serial_to_image(port: str, out_path: str = "frame_out.png", baud: int = 115200,
                     dedupe: bool = True, align: str = "none", gaussian: bool = True,
                     rotate_180: bool = True) -> np.ndarray:
    """Live pipeline: grab a frame straight from the ESP32 over serial."""
    text = capture_frame_over_serial(port, baud)
    rows, width, height = parse_frame_text(text)
    out = frame_to_image(rows, width, height, dedupe=dedupe, align=align)
    if gaussian or rotate_180:
        out = process_image(out, rotate_180=rotate_180) if gaussian else \
              (cv2.rotate(out, cv2.ROTATE_180) if rotate_180 else out)
    cv2.imwrite(out_path, out)
    return out


if __name__ == "__main__":
    # Usage:
    #   python3 dump_to_image.py COM3                 -> live capture from COM3
    #   python3 dump_to_image.py COM3 output.png       -> live capture, custom filename
    #   python3 dump_to_image.py --file dump.txt out.png -> process a saved log instead
    args = sys.argv[1:]

    if args and args[0] == "--file":
        src = args[1] if len(args) > 1 else "frame_dump.txt"
        dst = args[2] if len(args) > 2 else "frame_out.png"
        dump_to_image(src, dst)
        print(f"Wrote {dst}")
    else:
        port = args[0] if len(args) > 0 else "COM3"
        dst = args[1] if len(args) > 1 else "frame_out.png"
        serial_to_image(port, dst)
        print(f"Wrote {dst}")
