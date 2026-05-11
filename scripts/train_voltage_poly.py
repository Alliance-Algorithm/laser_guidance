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
        description="Train/export a cubic polynomial model from voltage_records.csv",
    )
    parser.add_argument("--input", required=True, help="Path to voltage_records.csv")
    parser.add_argument("--output", required=True, help="Output YAML model path")
    parser.add_argument("--width", type=float, default=1920.0, help="Image width in pixels")
    parser.add_argument("--height", type=float, default=1080.0, help="Image height in pixels")
    parser.add_argument(
        "--ridge",
        type=float,
        default=1e-3,
        help="Ridge regularization strength for the normal equations",
    )
    parser.add_argument(
        "--voltage-limit",
        type=float,
        default=5.0,
        help="Clamp training labels to +/- this value",
    )
    return parser.parse_args()


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


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


def mean(values: list[float]) -> float:
    return sum(values) / float(len(values))


def stddev(values: list[float], mu: float) -> float:
    variance = sum((value - mu) ** 2 for value in values) / float(len(values))
    sigma = math.sqrt(max(variance, 0.0))
    return sigma if sigma > 1e-6 else 1.0


def cubic_feature_basis(u_norm: float, v_norm: float, log_area_norm: float) -> list[float]:
    x = u_norm * 2.0 - 1.0
    y = v_norm * 2.0 - 1.0
    z = log_area_norm

    basis: list[float] = []
    for total in range(4):
        for i in range(total + 1):
            for j in range(total - i + 1):
                k = total - i - j
                basis.append((x ** i) * (y ** j) * (z ** k))
    return basis


def solve_linear_system(matrix: list[list[float]], vector: list[float]) -> list[float]:
    n = len(vector)
    aug = [row[:] + [vector[i]] for i, row in enumerate(matrix)]

    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(aug[row][col]))
        if abs(aug[pivot][col]) < 1e-9:
            raise ValueError("polynomial fit matrix is singular; collect more diverse samples")
        aug[col], aug[pivot] = aug[pivot], aug[col]

        pivot_value = aug[col][col]
        for j in range(col, n + 1):
            aug[col][j] /= pivot_value

        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col]
            if factor == 0.0:
                continue
            for j in range(col, n + 1):
                aug[row][j] -= factor * aug[col][j]

    return [aug[i][n] for i in range(n)]


def fit_ridge(features: list[list[float]], targets: list[float], ridge: float) -> list[float]:
    n_features = len(features[0])
    xtx = [[0.0 for _ in range(n_features)] for _ in range(n_features)]
    xty = [0.0 for _ in range(n_features)]

    for phi, target in zip(features, targets):
        for i in range(n_features):
            xty[i] += phi[i] * target
            for j in range(n_features):
                xtx[i][j] += phi[i] * phi[j]

    for i in range(n_features):
        xtx[i][i] += ridge

    return solve_linear_system(xtx, xty)


def write_yaml(
    output_path: Path,
    log_area_mean: float,
    log_area_std: float,
    vx_coeffs: list[float],
    vy_coeffs: list[float],
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    def fmt_array(values: list[float]) -> str:
        return "[" + ", ".join(f"{value:.9f}" for value in values) + "]"

    lines = [
        "model:",
        "  type: poly3",
        f"  log_area_mean: {log_area_mean:.9f}",
        f"  log_area_std: {log_area_std:.9f}",
        f"  vx_coeffs: {fmt_array(vx_coeffs)}",
        f"  vy_coeffs: {fmt_array(vy_coeffs)}",
    ]
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    samples = load_samples(Path(args.input), args.width, args.height, args.voltage_limit)

    log_area_values = [sample.log_area for sample in samples]
    log_area_mean = mean(log_area_values)
    log_area_std = stddev(log_area_values, log_area_mean)

    features = [
        cubic_feature_basis(sample.u_norm, sample.v_norm, (sample.log_area - log_area_mean) / log_area_std)
        for sample in samples
    ]
    vx_targets = [sample.vx for sample in samples]
    vy_targets = [sample.vy for sample in samples]

    vx_coeffs = fit_ridge(features, vx_targets, args.ridge)
    vy_coeffs = fit_ridge(features, vy_targets, args.ridge)
    write_yaml(Path(args.output), log_area_mean, log_area_std, vx_coeffs, vy_coeffs)

    print(
        f"exported poly3 model: {args.output} | samples={len(samples)} | "
        f"coeffs={len(vx_coeffs)} ridge={args.ridge}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
