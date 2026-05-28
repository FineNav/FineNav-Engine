#!/usr/bin/env python3
"""Generate a test PGM for flat_small_scale_maze.

Default output:
  ../config/flat_small_scale_maze_2d.pgm

Black pixels = occupied, white pixels = free.
"""

#  Copyright (c) 2026.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


Wall = Tuple[float, float, float, float, float]  # cx, cy, yaw, length, thickness

LONG_LEN = 8.0
SHORT_LEN = 4.0
THICK = 0.4
PI_2 = 1.5708


def long_wall(x: float, y: float, yaw: float) -> Wall:
    return (x, y, yaw, LONG_LEN, THICK)


def short_wall(x: float, y: float, yaw: float) -> Wall:
    return (x, y, yaw, SHORT_LEN, THICK)


def build_walls() -> List[Wall]:
    # Copied from finenav_simulator/resource/model/smaze2d/model.sdf
    return [
        long_wall(0, 4, PI_2), long_wall(0, 12, PI_2), long_wall(0, 20, PI_2), short_wall(0, 26, PI_2),
        long_wall(24, 4, PI_2), long_wall(24, 12, PI_2), long_wall(24, 20, PI_2), short_wall(24, 26, PI_2),
        long_wall(4, 0, 0), long_wall(12, 0, 0), long_wall(20, 0, 0),
        long_wall(4, 28, 0), long_wall(12, 28, 0), long_wall(20, 28, 0),
        short_wall(10, 24, 0), short_wall(10, 12, 0), long_wall(8, 8, 0),
        short_wall(5, 4, 0), short_wall(17, 8, 0), short_wall(20, 12, 0),
        long_wall(16, 24, PI_2),
        short_wall(20, 22, PI_2), short_wall(16, 14, PI_2), short_wall(16, 6, PI_2), short_wall(20, 2, PI_2),
        short_wall(12, 2, PI_2), short_wall(7, 6, PI_2), short_wall(4, 14, PI_2), short_wall(8, 18, PI_2),
        short_wall(4, 22, PI_2), short_wall(12, 14, PI_2),
        short_wall(2, 12, 0), short_wall(2, 20, 0), short_wall(6, 16, 0),
        short_wall(10, 20, 0), short_wall(18, 16, 0), short_wall(18, 20, 0),
    ]


def rasterize(width: int, height: int, resolution: float, origin_x: float, origin_y: float) -> bytearray:
    # 255 = free, 0 = occupied
    image = bytearray([255] * width * height)

    def set_occ(gx: int, gy: int) -> None:
        if 0 <= gx < width and 0 <= gy < height:
            image[(height - 1 - gy) * width + gx] = 0  # flip y for PGM image coordinates

    for cx, cy, yaw, length, thick in build_walls():
        c = math.cos(yaw)
        s = math.sin(yaw)
        hx = 0.5 * length
        hy = 0.5 * thick

        ex = abs(c * hx) + abs(s * hy)
        ey = abs(s * hx) + abs(c * hy)
        min_x, max_x = cx - ex, cx + ex
        min_y, max_y = cy - ey, cy + ey

        gx0 = max(0, int(math.floor((min_x - origin_x) / resolution)) - 1)
        gx1 = min(width - 1, int(math.floor((max_x - origin_x) / resolution)) + 1)
        gy0 = max(0, int(math.floor((min_y - origin_y) / resolution)) - 1)
        gy1 = min(height - 1, int(math.floor((max_y - origin_y) / resolution)) + 1)

        for gy in range(gy0, gy1 + 1):
            wy = origin_y + (gy + 0.5) * resolution
            for gx in range(gx0, gx1 + 1):
                wx = origin_x + (gx + 0.5) * resolution
                dx = wx - cx
                dy = wy - cy
                lx = c * dx + s * dy
                ly = -s * dx + c * dy
                if abs(lx) <= hx and abs(ly) <= hy:
                    set_occ(gx, gy)

    return image


def write_pgm(path: Path, width: int, height: int, pixels: Sequence[int]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        f.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        f.write(bytes(pixels))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "flat_small_scale_maze_2d.pgm",
        help="Output PGM path",
    )
    parser.add_argument("--resolution", type=float, default=0.2)
    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)
    parser.add_argument("--width-m", type=float, default=26.0)
    parser.add_argument("--height-m", type=float, default=30.0)
    args = parser.parse_args()

    width = int(round(args.width_m / args.resolution))
    height = int(round(args.height_m / args.resolution))
    pixels = rasterize(width, height, args.resolution, args.origin_x, args.origin_y)
    write_pgm(args.output, width, height, pixels)
    print(f"Wrote PGM: {args.output} ({width}x{height}, res={args.resolution})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

