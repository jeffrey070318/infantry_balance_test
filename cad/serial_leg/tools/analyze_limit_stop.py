"""Analyze static STEP evidence for the serial-leg mechanical stop.

Point-cloud distances are only a ranking signal. STEP vertex samples do not
prove contact, collision clearance, or an allowable joint-angle interval.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from extract_step_joint_axes import (
    StepFile,
    apply_point,
    component_axes,
    resolve_component_transform,
    unique_matching_lines,
)


STRUCTURAL_COMPONENTS = [
    "髋关节A",
    "小腿A",
    "连杆1改A",
    "连杆2改A",
    "连杆3A",
    "连杆4改A",
    "连杆5A",
]


def component_points(assembly: Path, component: str) -> np.ndarray:
    transform = resolve_component_transform(assembly, component)
    step_path = assembly.parent / f"{component}.STEP"
    if transform is None or not step_path.exists():
        return np.empty((0, 3))
    step = StepFile(step_path)
    points = [
        apply_point(transform, step.vec3(entity_id))
        for entity_id, value in step.records.items()
        if value.startswith("CARTESIAN_POINT")
    ]
    if not points:
        return np.empty((0, 3))
    return np.unique(np.round(np.asarray(points), decimals=8), axis=0)


def minimum_point_distance(left: np.ndarray, right: np.ndarray) -> float:
    minimum_squared = float("inf")
    for start in range(0, len(left), 256):
        delta = left[start : start + 256, None, :] - right[None, :, :]
        minimum_squared = min(
            minimum_squared, float(np.min(np.einsum("ijk,ijk->ij", delta, delta)))
        )
    return minimum_squared**0.5


def bounding_box(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return np.min(points, axis=0), np.max(points, axis=0)


def axis_gap(left: tuple[np.ndarray, np.ndarray], right: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    left_min, left_max = left
    right_min, right_max = right
    return np.maximum(np.maximum(left_min - right_max, right_min - left_max), 0.0)


def analyze(assembly: Path) -> dict[str, object]:
    limit_name = "腿部限位A"
    limit_transform = resolve_component_transform(assembly, limit_name)
    if limit_transform is None:
        raise RuntimeError("assembly does not contain 腿部限位A")

    limit_path = assembly.parent / f"{limit_name}.STEP"
    limit_axes = component_axes(limit_path, limit_transform)
    limit_points = component_points(assembly, limit_name)
    limit_box = bounding_box(limit_points)

    comparisons = []
    for component in STRUCTURAL_COMPONENTS:
        transform = resolve_component_transform(assembly, component)
        step_path = assembly.parent / f"{component}.STEP"
        if transform is None or not step_path.exists():
            continue
        axes = component_axes(step_path, transform)
        points = component_points(assembly, component)
        box = bounding_box(points)
        gap = axis_gap(limit_box, box)
        comparisons.append(
            {
                "component": component,
                "coincident_axis_count": len(
                    unique_matching_lines(limit_axes, axes)
                ),
                "sample_point_distance_mm": minimum_point_distance(
                    limit_points, points
                ),
                "bounding_box_axis_gap_mm": gap.tolist(),
                "bounding_boxes_overlap": bool(np.all(gap == 0.0)),
            }
        )

    comparisons.sort(key=lambda item: float(item["sample_point_distance_mm"]))
    return {
        "status": "static_geometry_evidence_only",
        "source_assembly": assembly.name,
        "limit_component": limit_name,
        "limit_bounding_box_mm": {
            "minimum": limit_box[0].tolist(),
            "maximum": limit_box[1].tolist(),
        },
        "comparisons": comparisons,
        "warning": (
            "点云最近距离仅用于排序可能的接触构件；不能代替实体碰撞、"
            "SolidWorks配合范围或人工转动装配检查。"
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("assembly", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    result = analyze(args.assembly)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
