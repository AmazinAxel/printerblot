#!/usr/bin/env python3
"""Convert a PDF file to G-code for the Blot pen plotter.

Renders page 1 of the PDF to an image, traces dark regions into vector
contour paths, auto-crops whitespace, scales to fit the 125x125mm work
area, and emits G-code that stream.py can send to the Blot.

Usage:
    python3 pdf2gcode.py ~/Downloads/drawing.pdf
    python3 pdf2gcode.py ~/Downloads/drawing.pdf --scale 0.8
    python3 pdf2gcode.py ~/Downloads/drawing.pdf --threshold 128

Output goes to ~/blot-gcode/<filename>.gcode

Requires: poppler (pdftoppm), numpy, pillow, scikit-image
"""
import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path

try:
    import numpy as np
    from PIL import Image
    from skimage import measure
except ImportError as e:
    sys.stderr.write(
        f"Missing dependency: {e}\n"
        f"Make sure you're in the nix devshell: nix develop\n"
    )
    sys.exit(1)


# Blot work area (matches DEFAULT_MAX_X/Y_MM in config.h)
MAX_X_MM = 125.0
MAX_Y_MM = 125.0

# G-code settings
FEED_RATE = 4000   # mm/min — use --speed to override
RAPID_RATE = 16000 # mm/min for pen-up travel

# Path filtering
MIN_SEGMENT_MM = 0.3      # skip paths shorter than this
SIMPLIFY_TOL_MM = 0.05    # Douglas-Peucker tolerance (lower = more detail, more lines)


def pdf_to_image(pdf_path, dpi=200):
    """Render page 1 of a PDF to a grayscale PIL Image using pdftoppm."""
    if not os.path.isfile(pdf_path):
        sys.stderr.write(f"File not found: {pdf_path}\n")
        sys.exit(1)

    with tempfile.TemporaryDirectory() as tmpdir:
        cmd = [
            "pdftoppm", "-gray", "-r", str(dpi),
            "-f", "1", "-l", "1",  # page 1 only
            pdf_path, os.path.join(tmpdir, "page"),
        ]

        try:
            subprocess.run(cmd, check=True, capture_output=True)
        except FileNotFoundError:
            sys.stderr.write(
                "pdftoppm not found. Make sure poppler is installed.\n"
                "In the nix devshell this should be available automatically.\n"
            )
            sys.exit(1)
        except subprocess.CalledProcessError as e:
            sys.stderr.write(f"pdftoppm failed: {e.stderr.decode()}\n")
            sys.exit(1)

        out_files = sorted(Path(tmpdir).glob("page-*.*"))
        if not out_files:
            sys.stderr.write("pdftoppm produced no output files.\n")
            sys.exit(1)

        return Image.open(str(out_files[0])).convert("L")


def auto_crop(img, threshold=240, padding=10):
    """Crop whitespace borders from a grayscale image.
    Returns the cropped image."""
    arr = np.array(img)
    # Find rows and columns that contain dark pixels
    dark = arr < threshold
    rows = np.any(dark, axis=1)
    cols = np.any(dark, axis=0)

    if not np.any(rows) or not np.any(cols):
        return img  # nothing to crop

    row_min, row_max = np.where(rows)[0][[0, -1]]
    col_min, col_max = np.where(cols)[0][[0, -1]]

    # Add padding
    row_min = max(0, row_min - padding)
    row_max = min(arr.shape[0] - 1, row_max + padding)
    col_min = max(0, col_min - padding)
    col_max = min(arr.shape[1] - 1, col_max + padding)

    return img.crop((col_min, row_min, col_max + 1, row_max + 1))


def image_to_contours(img, threshold=128):
    """Extract contour paths from a grayscale image.
    Returns list of Nx2 arrays of (x, y) in pixel coordinates."""
    arr = np.array(img)
    binary = (arr < threshold).astype(np.uint8)
    contours = measure.find_contours(binary, 0.5)

    result = []
    for c in contours:
        # find_contours returns (row, col) — swap to (x, y)
        xy = np.column_stack([c[:, 1], c[:, 0]])
        result.append(xy)
    return result


def simplify_path(points, tolerance):
    """Douglas-Peucker path simplification."""
    if len(points) <= 2:
        return points

    start = points[0]
    end = points[-1]
    line_vec = end - start
    line_len = np.linalg.norm(line_vec)

    if line_len < 1e-10:
        dists = np.linalg.norm(points - start, axis=1)
    else:
        line_unit = line_vec / line_len
        vecs = points - start
        proj = np.outer(vecs @ line_unit, line_unit)
        dists = np.linalg.norm(vecs - proj, axis=1)

    max_idx = np.argmax(dists)
    if dists[max_idx] <= tolerance:
        return np.array([start, end])

    left = simplify_path(points[:max_idx + 1], tolerance)
    right = simplify_path(points[max_idx:], tolerance)
    return np.vstack([left[:-1], right])


def optimize_path_order(contours):
    """Nearest-neighbor sort to minimize pen-up travel."""
    if len(contours) <= 1:
        return contours

    remaining = list(range(len(contours)))
    ordered = [remaining.pop(0)]
    current_end = contours[ordered[0]][-1]

    while remaining:
        best_idx = None
        best_dist = float("inf")
        best_reversed = False

        for i, ri in enumerate(remaining):
            c = contours[ri]
            d_start = np.linalg.norm(current_end - c[0])
            d_end = np.linalg.norm(current_end - c[-1])

            if d_start < best_dist:
                best_dist = d_start
                best_idx = i
                best_reversed = False
            if d_end < best_dist:
                best_dist = d_end
                best_idx = i
                best_reversed = True

        ri = remaining.pop(best_idx)
        if best_reversed:
            contours[ri] = contours[ri][::-1]
        ordered.append(ri)
        current_end = contours[ri][-1]

    return [contours[i] for i in ordered]


def contours_to_gcode(contours, img_width, img_height,
                      margin_mm=5.0, scale_factor=1.0, feed_rate=None):
    """Convert pixel contours to G-code, scaled to fit the work area.

    Origin (0,0) is the starting position of the carriage (bottom-left of
    the work area).  The drawing is centered within the work area using
    only positive coordinates so nothing goes out of bounds.
    scale_factor further shrinks the drawing (0.5 = half size)."""
    if feed_rate is None:
        feed_rate = FEED_RATE

    # Usable area (work area minus margin on each side)
    usable_x = (MAX_X_MM - 2.0 * margin_mm) * scale_factor
    usable_y = (MAX_Y_MM - 2.0 * margin_mm) * scale_factor

    # Scale image to fit, preserving aspect ratio
    scale = min(usable_x / img_width, usable_y / img_height)

    # Actual drawing dimensions in mm
    drawing_w = img_width * scale
    drawing_h = img_height * scale

    # Offset to center the drawing within the work area
    offset_x = margin_mm + (usable_x / scale_factor - drawing_w) / 2.0
    offset_y = margin_mm + (usable_y / scale_factor - drawing_h) / 2.0

    def to_mm(px, py):
        # Flip Y: image Y=0 is top, plotter Y+ is up
        mx = px * scale + offset_x
        my = (img_height - py) * scale + offset_y
        return mx, my

    # Verify bounds
    x_min_mm, y_min_mm = to_mm(0, img_height)
    x_max_mm, y_max_mm = to_mm(img_width, 0)

    lines = []
    lines.append("; Generated by pdf2gcode.py for Blot pen plotter")
    lines.append(f"; Drawing area: {drawing_w:.1f}x{drawing_h:.1f}mm "
                 f"(scale {scale:.4f} px->mm)")
    lines.append(f"; Origin at bottom-left. Bounds: X[{x_min_mm:.1f}, {x_max_mm:.1f}] "
                 f"Y[{y_min_mm:.1f}, {y_max_mm:.1f}]")
    lines.append("G21")
    lines.append("G90")
    lines.append("G92 X0 Y0")
    lines.append("M5")

    total_paths = 0
    total_points = 0

    for contour in contours:
        if len(contour) < 2:
            continue

        mm_points = np.array([to_mm(p[0], p[1]) for p in contour])

        # Simplify to reduce point count
        mm_points = simplify_path(mm_points, SIMPLIFY_TOL_MM)
        if len(mm_points) < 2:
            continue

        # Compute path length, skip tiny paths
        diffs = np.diff(mm_points, axis=0)
        path_len = np.sum(np.linalg.norm(diffs, axis=1))
        if path_len < MIN_SEGMENT_MM:
            continue

        # Clip to work area bounds
        mm_points[:, 0] = np.clip(mm_points[:, 0], margin_mm, MAX_X_MM - margin_mm)
        mm_points[:, 1] = np.clip(mm_points[:, 1], margin_mm, MAX_Y_MM - margin_mm)

        # Pen up, rapid travel to start
        sx, sy = mm_points[0]
        lines.append(f"G1 X{sx:.3f} Y{sy:.3f} F{RAPID_RATE}")
        lines.append("M3")

        # Draw the path
        for px, py in mm_points[1:]:
            lines.append(f"G1 X{px:.3f} Y{py:.3f} F{feed_rate}")

        # Pen up
        lines.append("M5")

        total_paths += 1
        total_points += len(mm_points)

    # Park at origin and disable
    lines.append(f"G1 X0 Y0 F{RAPID_RATE}")
    lines.append("M18")

    print(f"  {total_paths} paths, {total_points} total points")
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser(
        description="Convert a PDF (page 1) to G-code for the Blot pen plotter."
    )
    ap.add_argument("pdf", help="Path to the PDF file")
    ap.add_argument("--threshold", type=int, default=128,
                    help="Darkness threshold 0-255 (default: 128, "
                         "lower = only darkest features)")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="Scale factor 0-1 (default: 1.0 = fill work area)")
    ap.add_argument("--dpi", type=int, default=300,
                    help="PDF render resolution (default: 300, higher = finer detail)")
    ap.add_argument("--speed", type=int, default=None,
                    help="Drawing speed in mm/min (default: 2000, lower = sharper)")
    ap.add_argument("--margin", type=float, default=2.0,
                    help="Margin in mm around the drawing (default: 2)")
    ap.add_argument("--no-crop", action="store_true",
                    help="Don't auto-crop whitespace from the PDF")
    ap.add_argument("--output-dir", default=None,
                    help="Output directory (default: ~/blot-gcode/)")
    args = ap.parse_args()

    pdf_path = os.path.expanduser(args.pdf)
    if not os.path.isfile(pdf_path):
        sys.stderr.write(f"File not found: {pdf_path}\n")
        sys.exit(1)

    # Output directory
    out_dir = args.output_dir or os.path.expanduser("~/blot-gcode")
    os.makedirs(out_dir, exist_ok=True)

    # Output filename
    base = os.path.splitext(os.path.basename(pdf_path))[0]
    out_file = os.path.join(out_dir, f"{base}.gcode")

    print(f"Converting: {pdf_path}")
    print(f"  Rendering page 1 at {args.dpi} DPI...")
    img = pdf_to_image(pdf_path, dpi=args.dpi)
    print(f"  Full image: {img.width}x{img.height}px")

    if not args.no_crop:
        img = auto_crop(img)
        print(f"  After crop: {img.width}x{img.height}px")

    print(f"  Extracting contours (threshold={args.threshold})...")
    contours = image_to_contours(img, threshold=args.threshold)
    print(f"  Found {len(contours)} raw contours")

    if not contours:
        sys.stderr.write("No contours found — try a higher --threshold\n")
        sys.exit(1)

    print(f"  Optimizing path order...")
    contours = optimize_path_order(contours)

    print(f"  Generating G-code...")
    gcode = contours_to_gcode(
        contours, img.width, img.height,
        margin_mm=args.margin, scale_factor=args.scale,
        feed_rate=args.speed,
    )

    with open(out_file, "w") as f:
        f.write(gcode)

    # Count lines for user info
    gcode_lines = [l for l in gcode.strip().split("\n")
                   if l.strip() and not l.startswith(";")]
    print(f"\nSaved: {out_file} ({len(gcode_lines)} G-code lines)")
    print(f"\nTo print:")
    print(f"  python3 tools/stream.py '{out_file}' --port /dev/ttyACM0")


if __name__ == "__main__":
    main()
