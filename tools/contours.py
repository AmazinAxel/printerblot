"""Marching squares contour extraction.

Drop-in replacement for skimage.measure.find_contours for the binary-input,
level=0.5 case that pdf2gcode.py uses. Exists so the Pi image only needs
numpy, not the whole scipy/scikit-image stack (scikit-image has no Buildroot
package).

Returns the same thing skimage does: a list of Nx2 float arrays of (row, col)
coordinates. Closed contours repeat their first point at the end.
"""
import numpy as np


# For each of the 16 corner configurations, which cell edges the contour
# crosses. Corners are numbered:  0=top-left 1=top-right 2=bottom-right
# 3=bottom-left, packed as a 4-bit index. Edges are named by the side of the
# cell they sit on: T(op), R(ight), B(ottom), L(eft).
#
# Each entry lists (entry_edge, exit_edge) pairs, oriented so the "inside"
# (value 1) region stays to the LEFT of travel — this matches skimage's
# orientation, which pdf2gcode's pen-up/pen-down ordering depends on.
_CASES = {
    0:  [],
    1:  [("T", "L")],
    2:  [("R", "T")],
    3:  [("R", "L")],
    4:  [("B", "R")],
    5:  [("T", "L"), ("B", "R")],   # saddle
    6:  [("B", "T")],
    7:  [("B", "L")],
    8:  [("L", "B")],
    9:  [("T", "B")],
    10: [("R", "T"), ("L", "B")],   # saddle
    11: [("R", "B")],
    12: [("L", "R")],
    13: [("T", "R")],
    14: [("L", "T")],
    15: [],
}


def _edge_point(r, c, edge):
    """Midpoint of a cell edge, in (row, col) coords of the source array.

    Cell (r, c) spans rows r..r+1 and cols c..c+1. At level 0.5 on binary
    input every crossing is exactly halfway along the edge, so no
    interpolation is needed.
    """
    if edge == "T":
        return (r, c + 0.5)
    if edge == "B":
        return (r + 1.0, c + 0.5)
    if edge == "L":
        return (r + 0.5, c)
    return (r + 0.5, c + 1.0)  # "R"


# Moving off a cell through one edge enters the neighbour across it, arriving
# on that neighbour's opposite edge.
_STEP = {
    "T": (-1, 0, "B"),
    "B": (1, 0, "T"),
    "L": (0, -1, "R"),
    "R": (0, 1, "L"),
}


def find_contours(array, level=0.5):
    """Extract iso-contours at `level` from a 2D binary array.

    Only the binary/0.5 case is supported — that is all pdf2gcode.py needs.
    """
    arr = np.asarray(array)
    if arr.ndim != 2:
        raise ValueError("find_contours expects a 2D array")

    above = arr > level
    h, w = arr.shape
    if h < 2 or w < 2:
        return []

    # 4-bit corner config per cell, vectorised over the whole grid.
    tl = above[:-1, :-1]
    tr = above[:-1, 1:]
    br = above[1:, 1:]
    bl = above[1:, :-1]
    codes = (tl << 0 | tr << 1 | br << 2 | bl << 3).astype(np.uint8)

    # segments[(r, c)] -> list of (entry_edge, exit_edge)
    segments = {}
    for r, c in zip(*np.nonzero((codes != 0) & (codes != 15))):
        pairs = _CASES[int(codes[r, c])]
        if pairs:
            segments[(r, c)] = list(pairs)

    contours = []
    while segments:
        # Start from an arbitrary remaining segment.
        cell = next(iter(segments))
        entry, exit_ = segments[cell].pop(0)
        if not segments[cell]:
            del segments[cell]

        start_cell, start_entry = cell, entry
        points = [_edge_point(cell[0], cell[1], entry),
                  _edge_point(cell[0], cell[1], exit_)]

        closed = False
        while True:
            dr, dc, arrive = _STEP[exit_]
            nr, nc = cell[0] + dr, cell[1] + dc
            if not (0 <= nr < h - 1 and 0 <= nc < w - 1):
                break  # ran off the edge of the grid: open contour
            if (nr, nc) == start_cell and arrive == start_entry:
                closed = True
                break
            nxt = segments.get((nr, nc))
            if nxt is None:
                break
            # Pick the segment we actually arrived on (matters for saddles).
            for i, (e_in, e_out) in enumerate(nxt):
                if e_in == arrive:
                    nxt.pop(i)
                    break
            else:
                break
            if not nxt:
                del segments[(nr, nc)]
            cell, exit_ = (nr, nc), e_out
            points.append(_edge_point(nr, nc, e_out))

        if closed:
            # The walk already re-emitted the starting edge point as its last
            # step; don't append a second copy.
            if not np.allclose(points[0], points[-1]):
                points.append(points[0])
        elif len(points) >= 2:
            # Open contour: we started mid-path, so also walk backwards from
            # the start so the piece isn't truncated.
            points = _extend_backwards(points, segments, start_cell,
                                       start_entry, h, w)

        if len(points) >= 2:
            contours.append(np.array(points, dtype=float))

    return contours


def _extend_backwards(points, segments, start_cell, start_entry, h, w):
    """Walk backwards from an open contour's start to pick up the earlier half."""
    prefix = []
    cell, entry = start_cell, start_entry
    while True:
        dr, dc, arrive = _STEP[entry]
        nr, nc = cell[0] + dr, cell[1] + dc
        if not (0 <= nr < h - 1 and 0 <= nc < w - 1):
            break
        nxt = segments.get((nr, nc))
        if nxt is None:
            break
        for i, (e_in, e_out) in enumerate(nxt):
            if e_out == arrive:
                nxt.pop(i)
                break
        else:
            break
        if not nxt:
            del segments[(nr, nc)]
        prefix.append(_edge_point(nr, nc, e_in))
        cell, entry = (nr, nc), e_in
    prefix.reverse()
    return prefix + points
