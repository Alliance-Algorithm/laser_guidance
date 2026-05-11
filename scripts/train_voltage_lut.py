#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Sample:
    u_norm: float
    v_norm: float
    log_area: float
    vx: float
    vy: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train/export a LUT model from voltage_records.csv",
    )
    parser.add_argument("--input", required=True, help="Path to voltage_records.csv")
    parser.add_argument("--output", required=True, help="Output YAML model path")
    parser.add_argument("--width", type=float, default=1920.0, help="Image width in pixels")
    parser.add_argument("--height", type=float, default=1080.0, help="Image height in pixels")
    parser.add_argument("--u-bins", type=int, default=17, help="Number of bins on normalized u axis")
    parser.add_argument("--v-bins", type=int, default=13, help="Number of bins on normalized v axis")
    parser.add_argument("--area-bins", type=int, default=5, help="Number of bins on log-area axis")
    parser.add_argument(
        "--voltage-limit",
        type=float,
        default=5.0,
        help="Clamp output voltages to +/- this value",
    )
    return parser.parse_args()


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def linspace(lo: float, hi: float, count: int) -> list[float]:
    if count <= 1:
        return [lo]
    step = (hi - lo) / float(count - 1)
    return [lo + i * step for i in range(count)]


def quantile_axis(values: list[float], count: int) -> list[float]:
    if count <= 1:
        return [sum(values) / float(len(values))]
    sorted_values = sorted(values)
    n = len(sorted_values)
    axis: list[float] = []
    for i in range(count):
        q = i / float(count - 1)
        pos = q * float(n - 1)
        lo = int(math.floor(pos))
        hi = int(math.ceil(pos))
        if lo == hi:
            axis.append(sorted_values[lo])
        else:
            t = pos - lo
            axis.append(sorted_values[lo] + (sorted_values[hi] - sorted_values[lo]) * t)
    return axis


def nearest_axis_index(axis: list[float], x: float) -> int:
    best_idx = 0
    best_dist = float("inf")
    for idx, value in enumerate(axis):
        dist = abs(value - x)
        if dist < best_dist:
            best_dist = dist
            best_idx = idx
    return best_idx


def load_samples(csv_path: Path, width: float, height: float, voltage_limit: float) -> list[Sample]:
    if width <= 0 or height <= 0:
        raise ValueError("width/height must be positive")

    samples: list[Sample] = []
    with csv_path.open("r", newline="") as fp:
        reader = csv.DictReader(fp)
        required = {
            "center_x",
            "center_y",
            "bbox_area",
            "manual_vx",
            "manual_vy",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"missing required CSV columns: {sorted(missing)}")

        for row in reader:
            try:
                center_x = float(row["center_x"])
                center_y = float(row["center_y"])
                bbox_area = float(row["bbox_area"])
                vx = clamp(float(row["manual_vx"]), -voltage_limit, voltage_limit)
                vy = clamp(float(row["manual_vy"]), -voltage_limit, voltage_limit)
            except (KeyError, ValueError):
                continue

            if bbox_area <= 0:
                continue

            samples.append(
                Sample(
                    u_norm=clamp(center_x / width, 0.0, 1.0),
                    v_norm=clamp(center_y / height, 0.0, 1.0),
                    log_area=math.log(max(bbox_area, 1.0)),
                    vx=vx,
                    vy=vy,
                )
            )

    if not samples:
        raise ValueError("no valid samples loaded from CSV")
    return samples


def train_lut(
    samples: list[Sample],
    u_bins: int,
    v_bins: int,
    area_bins: int,
) -> tuple[list[float], list[float], list[float], list[list[list[float]]], list[list[list[float]]]]:
    if u_bins <= 0 or v_bins <= 0 or area_bins <= 0:
        raise ValueError("all bin counts must be positive")

    u_axis = linspace(0.0, 1.0, u_bins)
    v_axis = linspace(0.0, 1.0, v_bins)
    area_axis = quantile_axis([s.log_area for s in samples], area_bins)

    sums_vx = [[[0.0 for _ in range(u_bins)] for _ in range(v_bins)] for _ in range(area_bins)]
    sums_vy = [[[0.0 for _ in range(u_bins)] for _ in range(v_bins)] for _ in range(area_bins)]
    counts = [[[0 for _ in range(u_bins)] for _ in range(v_bins)] for _ in range(area_bins)]

    for sample in samples:
        ui = nearest_axis_index(u_axis, sample.u_norm)
        vi = nearest_axis_index(v_axis, sample.v_norm)
        ai = nearest_axis_index(area_axis, sample.log_area)
        sums_vx[ai][vi][ui] += sample.vx
        sums_vy[ai][vi][ui] += sample.vy
        counts[ai][vi][ui] += 1

    populated: list[tuple[int, int, int, float, float]] = []
    for ai in range(area_bins):
        for vi in range(v_bins):
            for ui in range(u_bins):
                count = counts[ai][vi][ui]
                if count > 0:
                    mean_vx = sums_vx[ai][vi][ui] / float(count)
                    mean_vy = sums_vy[ai][vi][ui] / float(count)
                    populated.append((ai, vi, ui, mean_vx, mean_vy))

    if not populated:
        raise ValueError("training produced zero populated LUT cells")

    vx_values = [[[0.0 for _ in range(u_bins)] for _ in range(v_bins)] for _ in range(area_bins)]
    vy_values = [[[0.0 for _ in range(u_bins)] for _ in range(v_bins)] for _ in range(area_bins)]

    def nearest_populated(target_a: int, target_v: int, target_u: int) -> tuple[float, float]:
        best_dist = float("inf")
        best = (0.0, 0.0)
        for a, v, u, mean_vx, mean_vy in populated:
            dist = (a - target_a) ** 2 + (v - target_v) ** 2 + (u - target_u) ** 2
            if dist < best_dist:
                best_dist = dist
                best = (mean_vx, mean_vy)
        return best

    for ai in range(area_bins):
        for vi in range(v_bins):
            for ui in range(u_bins):
                count = counts[ai][vi][ui]
                if count > 0:
                    vx_values[ai][vi][ui] = sums_vx[ai][vi][ui] / float(count)
                    vy_values[ai][vi][ui] = sums_vy[ai][vi][ui] / float(count)
                else:
                    fill_vx, fill_vy = nearest_populated(ai, vi, ui)
                    vx_values[ai][vi][ui] = fill_vx
                    vy_values[ai][vi][ui] = fill_vy

    return u_axis, v_axis, area_axis, vx_values, vy_values


def write_yaml(
    output_path: Path,
    u_axis: list[float],
    v_axis: list[float],
    area_axis: list[float],
    vx_values: list[list[list[float]]],
    vy_values: list[list[list[float]]],
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    def fmt_array(values: list[float]) -> str:
        return "[" + ", ".join(f"{value:.6f}" for value in values) + "]"

    lines: list[str] = []
    lines.append("model:")
    lines.append("  type: lut")
    lines.append(f"  u_axis: {fmt_array(u_axis)}")
    lines.append(f"  v_axis: {fmt_array(v_axis)}")
    lines.append(f"  log_area_axis: {fmt_array(area_axis)}")

    def write_volume(name: str, volume: list[list[list[float]]]) -> None:
        lines.append(f"  {name}:")
        for plane in volume:
            lines.append("    -")
            for row in plane:
                lines.append(f"      - {fmt_array(row)}")

    write_volume("vx_values", vx_values)
    write_volume("vy_values", vy_values)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    input_path = Path(args.input)
    output_path = Path(args.output)

    samples = load_samples(input_path, args.width, args.height, args.voltage_limit)
    u_axis, v_axis, area_axis, vx_values, vy_values = train_lut(
        samples=samples,
        u_bins=args.u_bins,
        v_bins=args.v_bins,
        area_bins=args.area_bins,
    )
    write_yaml(output_path, u_axis, v_axis, area_axis, vx_values, vy_values)

    total_cells = len(u_axis) * len(v_axis) * len(area_axis)
    print(
        f"exported LUT model: {output_path} | samples={len(samples)} | "
        f"u_bins={len(u_axis)} v_bins={len(v_axis)} area_bins={len(area_axis)} cells={total_cells}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
