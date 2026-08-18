"""Extract coincident cylindrical axes from a SolidWorks AP214 STEP assembly.

The script is intentionally read-only with respect to the exported STEP files.
It uses only the Python standard library so the extraction can be reproduced
without a CAD kernel. Coincident cylinders are joint candidates, not proof of
the mechanism's kinematic topology.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


FLOAT_RE = re.compile(r"[-+]?\d+(?:\.\d*)?(?:[Ee][-+]?\d+)?")
RECORD_RE = re.compile(r"#(\d+)\s*=\s*(.*?);", re.DOTALL)
REF_RE = re.compile(r"#(\d+)")
X2_RE = re.compile(r"\\X2\\([0-9A-Fa-f]+)\\X0\\")


Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]


@dataclass(frozen=True)
class Axis:
    component: str
    point: Vec3
    direction: Vec3
    radius_mm: float


def add(a: Vec3, b: Vec3) -> Vec3:
    return tuple(a[i] + b[i] for i in range(3))  # type: ignore[return-value]


def sub(a: Vec3, b: Vec3) -> Vec3:
    return tuple(a[i] - b[i] for i in range(3))  # type: ignore[return-value]


def scale(a: Vec3, value: float) -> Vec3:
    return tuple(value * a[i] for i in range(3))  # type: ignore[return-value]


def dot(a: Vec3, b: Vec3) -> float:
    return sum(a[i] * b[i] for i in range(3))


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a: Vec3) -> float:
    return math.sqrt(dot(a, a))


def unit(a: Vec3) -> Vec3:
    length = norm(a)
    if length < 1e-12:
        raise ValueError("zero-length direction")
    return scale(a, 1.0 / length)


def mat_vec(matrix: Mat3, vector: Vec3) -> Vec3:
    return tuple(dot(row, vector) for row in matrix)  # type: ignore[return-value]


def transpose(matrix: Mat3) -> Mat3:
    return tuple(tuple(matrix[j][i] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def mat_mul(a: Mat3, b: Mat3) -> Mat3:
    bt = transpose(b)
    return tuple(tuple(dot(row, column) for column in bt) for row in a)  # type: ignore[return-value]


def columns(x: Vec3, y: Vec3, z: Vec3) -> Mat3:
    return tuple(tuple((x, y, z)[j][i] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def decode_step_string(value: str) -> str:
    def decode_match(match: re.Match[str]) -> str:
        return bytes.fromhex(match.group(1)).decode("utf-16-be")

    return X2_RE.sub(decode_match, value).replace("''", "'")


class StepFile:
    def __init__(self, path: Path):
        self.path = path
        text = path.read_text(encoding="latin1")
        self.records = {int(key): value.strip() for key, value in RECORD_RE.findall(text)}

    def refs(self, entity_id: int) -> list[int]:
        return [int(value) for value in REF_RE.findall(self.records[entity_id])]

    def vec3(self, entity_id: int) -> Vec3:
        numbers = [float(value) for value in FLOAT_RE.findall(self.records[entity_id])]
        return tuple(numbers[-3:])  # type: ignore[return-value]

    def name(self, entity_id: int) -> str:
        match = re.search(r"\(\s*'((?:''|[^'])*)'", self.records[entity_id])
        return decode_step_string(match.group(1)) if match else ""

    def placement(self, entity_id: int) -> tuple[Mat3, Vec3]:
        refs = self.refs(entity_id)
        if len(refs) < 3 or not self.records[entity_id].startswith("AXIS2_PLACEMENT_3D"):
            raise ValueError(f"#{entity_id} is not a complete AXIS2_PLACEMENT_3D")
        origin = self.vec3(refs[0])
        z_axis = unit(self.vec3(refs[1]))
        x_hint = self.vec3(refs[2])
        x_axis = unit(sub(x_hint, scale(z_axis, dot(x_hint, z_axis))))
        y_axis = unit(cross(z_axis, x_axis))
        return columns(x_axis, y_axis, z_axis), origin


def rigid_transform(
    destination: tuple[Mat3, Vec3], source: tuple[Mat3, Vec3]
) -> tuple[Mat3, Vec3]:
    destination_rotation, destination_origin = destination
    source_rotation, source_origin = source
    rotation = mat_mul(destination_rotation, transpose(source_rotation))
    translation = sub(destination_origin, mat_vec(rotation, source_origin))
    return rotation, translation


def compose_transforms(
    parent: tuple[Mat3, Vec3], child: tuple[Mat3, Vec3]
) -> tuple[Mat3, Vec3]:
    parent_rotation, parent_translation = parent
    child_rotation, child_translation = child
    return (
        mat_mul(parent_rotation, child_rotation),
        add(mat_vec(parent_rotation, child_translation), parent_translation),
    )


def apply_point(transform: tuple[Mat3, Vec3], point: Vec3) -> Vec3:
    rotation, translation = transform
    return add(mat_vec(rotation, point), translation)


def apply_direction(transform: tuple[Mat3, Vec3], direction: Vec3) -> Vec3:
    return unit(mat_vec(transform[0], direction))


def canonical_line(point: Vec3, direction: Vec3) -> tuple[Vec3, Vec3]:
    direction = unit(direction)
    dominant = max(range(3), key=lambda i: abs(direction[i]))
    if direction[dominant] < 0:
        direction = scale(direction, -1.0)
    nearest_origin = sub(point, scale(direction, dot(point, direction)))
    return nearest_origin, direction


def assembly_transforms(step: StepFile) -> dict[str, tuple[Mat3, Vec3]]:
    representations = {
        entity_id: step.name(entity_id)
        for entity_id, value in step.records.items()
        if value.startswith("SHAPE_REPRESENTATION")
    }
    transforms: dict[str, tuple[Mat3, Vec3]] = {}
    for value in step.records.values():
        if "REPRESENTATION_RELATIONSHIP" not in value or "WITH_TRANSFORMATION" not in value:
            continue
        refs = [int(item) for item in REF_RE.findall(value)]
        if len(refs) < 3 or refs[1] not in representations:
            continue
        transform_record = step.records.get(refs[-1], "")
        if not transform_record.startswith("ITEM_DEFINED_TRANSFORMATION"):
            continue
        placements = step.refs(refs[-1])
        if len(placements) != 2:
            continue
        transforms[representations[refs[1]]] = rigid_transform(
            step.placement(placements[0]), step.placement(placements[1])
        )
    return transforms


def resolve_component_transform(
    assembly_path: Path,
    target: str,
    visited: set[Path] | None = None,
) -> tuple[Mat3, Vec3] | None:
    assembly_path = assembly_path.resolve()
    visited = set() if visited is None else visited
    if assembly_path in visited:
        return None
    visited.add(assembly_path)
    transforms = assembly_transforms(StepFile(assembly_path))
    if target in transforms:
        return transforms[target]

    for child_name, child_transform in transforms.items():
        child_path = assembly_path.parent / f"{child_name}.STEP"
        if not child_path.exists():
            continue
        nested_transform = resolve_component_transform(child_path, target, set(visited))
        if nested_transform is not None:
            return compose_transforms(child_transform, nested_transform)
    return None


def component_axes(path: Path, transform: tuple[Mat3, Vec3]) -> list[Axis]:
    step = StepFile(path)
    axes: list[Axis] = []
    for value in step.records.values():
        if not value.startswith("CYLINDRICAL_SURFACE"):
            continue
        refs = [int(item) for item in REF_RE.findall(value)]
        numbers = [float(item) for item in FLOAT_RE.findall(value)]
        if not refs or not numbers:
            continue
        rotation, origin = step.placement(refs[0])
        local_direction = (rotation[0][2], rotation[1][2], rotation[2][2])
        global_point = apply_point(transform, origin)
        global_direction = apply_direction(transform, local_direction)
        point, direction = canonical_line(global_point, global_direction)
        axes.append(Axis(path.stem, point, direction, numbers[-1]))
    return deduplicate_component_axes(axes)


def line_distance(a: Axis, b: Axis) -> float:
    return norm(cross(sub(b.point, a.point), a.direction))


def deduplicate_component_axes(axes: Iterable[Axis], tolerance_mm: float = 1e-4) -> list[Axis]:
    unique: list[Axis] = []
    for candidate in axes:
        duplicate = any(
            abs(dot(candidate.direction, current.direction)) > 1.0 - 1e-10
            and line_distance(candidate, current) < tolerance_mm
            and abs(candidate.radius_mm - current.radius_mm) < tolerance_mm
            for current in unique
        )
        if not duplicate:
            unique.append(candidate)
    return unique


def cluster_axes(
    axes: list[Axis], distance_tolerance_mm: float, angular_tolerance_deg: float
) -> list[list[Axis]]:
    parent = list(range(len(axes)))

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(left: int, right: int) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    cosine_limit = math.cos(math.radians(angular_tolerance_deg))
    for left in range(len(axes)):
        for right in range(left + 1, len(axes)):
            if axes[left].component == axes[right].component:
                continue
            if abs(dot(axes[left].direction, axes[right].direction)) < cosine_limit:
                continue
            if line_distance(axes[left], axes[right]) <= distance_tolerance_mm:
                union(left, right)

    groups: dict[int, list[Axis]] = {}
    for index, axis in enumerate(axes):
        groups.setdefault(find(index), []).append(axis)
    return [
        group
        for group in groups.values()
        if len({axis.component for axis in group}) >= 2
    ]


def group_axes_by_line(axes: list[Axis], tolerance_mm: float = 1e-3) -> list[list[Axis]]:
    groups: list[list[Axis]] = []
    for axis in axes:
        group = next(
            (
                current
                for current in groups
                if abs(dot(axis.direction, current[0].direction)) > 1.0 - 1e-10
                and line_distance(axis, current[0]) < tolerance_mm
            ),
            None,
        )
        if group is None:
            groups.append([axis])
        else:
            group.append(axis)
    return groups


def extract_transmission_evidence(
    assembly_path: Path, structural_clusters: list[list[Axis]]
) -> list[dict[str, object]]:
    targets = ["主动链轮AA", "主动链轮BA", "大腿从动链轮aA", "小腿从动链轮1aA"]
    evidence: list[dict[str, object]] = []
    references = [cluster[0] for cluster in structural_clusters]
    for target in targets:
        step_path = assembly_path.parent / f"{target}.STEP"
        transform = resolve_component_transform(assembly_path, target)
        if transform is None or not step_path.exists():
            evidence.append({"component": target, "status": "not_found"})
            continue
        line_groups = group_axes_by_line(component_axes(step_path, transform))
        matches: list[tuple[int, list[Axis]]] = []
        for candidate_id, reference in enumerate(references, start=1):
            for group in line_groups:
                if (
                    abs(dot(group[0].direction, reference.direction)) > 1.0 - 1e-10
                    and line_distance(group[0], reference) < 0.05
                ):
                    matches.append((candidate_id, group))
        if matches:
            selected_id, selected_group = max(
                matches, key=lambda item: max(axis.radius_mm for axis in item[1])
            )
            status = "coaxial_with_structural_candidate"
        else:
            selected_id = None
            selected_group = max(
                line_groups, key=lambda group: max(axis.radius_mm for axis in group)
            )
            status = "separate_drive_axis"
        evidence.append(
            {
                "component": target,
                "status": status,
                "structural_candidate_id": selected_id,
                "point_mm": list(selected_group[0].point),
                "direction": list(selected_group[0].direction),
                "observed_cylindrical_radii_mm": sorted(
                    {axis.radius_mm for axis in selected_group}
                ),
            }
        )
    return evidence


def unique_matching_lines(left_axes: list[Axis], right_axes: list[Axis]) -> list[Axis]:
    matches: list[Axis] = []
    for left in left_axes:
        if not any(
            abs(dot(left.direction, right.direction)) > 1.0 - 1e-10
            and line_distance(left, right) < 0.05
            for right in right_axes
        ):
            continue
        if not any(
            abs(dot(left.direction, current.direction)) > 1.0 - 1e-10
            and line_distance(left, current) < 0.05
            for current in matches
        ):
            matches.append(left)
    return matches


def extract_wheel_attachment_evidence(assembly_path: Path) -> dict[str, object]:
    structural_targets = ["小腿A", "连杆1改A", "连杆2改A", "连杆3A", "连杆4改A", "连杆5A"]
    gearbox_path = assembly_path.parent / "轮腿减速箱总装A.STEP"
    gearbox_children = list(assembly_transforms(StepFile(gearbox_path)))
    axis_cache: dict[str, list[Axis]] = {}
    for target in structural_targets + gearbox_children + ["CNC-聚氨酯胶轮A"]:
        step_path = assembly_path.parent / f"{target}.STEP"
        transform = resolve_component_transform(assembly_path, target)
        axis_cache[target] = (
            component_axes(step_path, transform)
            if transform is not None and step_path.exists()
            else []
        )

    attachment_matches: list[dict[str, object]] = []
    for structural in structural_targets:
        for gearbox_child in gearbox_children:
            matches = unique_matching_lines(
                axis_cache[structural], axis_cache[gearbox_child]
            )
            if len(matches) >= 2:
                attachment_matches.append(
                    {
                        "structural_component": structural,
                        "gearbox_component": gearbox_child,
                        "coincident_axis_count": len(matches),
                        "axis_points_mm": [list(axis.point) for axis in matches],
                    }
                )
    attachment_matches.sort(
        key=lambda item: int(item["coincident_axis_count"]), reverse=True
    )

    wheel_groups = group_axes_by_line(axis_cache["CNC-聚氨酯胶轮A"])
    wheel_group = max(
        wheel_groups, key=lambda group: max(axis.radius_mm for axis in group)
    )
    return {
        "wheel_component": "CNC-聚氨酯胶轮A",
        "wheel_axis_point_mm": list(wheel_group[0].point),
        "wheel_axis_direction": list(wheel_group[0].direction),
        "wheel_outer_radius_mm": max(axis.radius_mm for axis in wheel_group),
        "rigid_attachment_matches": attachment_matches,
    }


def estimate_sprocket_teeth(step_path: Path) -> dict[str, object]:
    identity: tuple[Mat3, Vec3] = (
        ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        (0.0, 0.0, 0.0),
    )
    step = StepFile(step_path)
    line_groups = group_axes_by_line(component_axes(step_path, identity))
    center_group = max(
        line_groups, key=lambda group: max(axis.radius_mm for axis in group)
    )
    center = center_group[0].point
    axis = center_group[0].direction
    helper = (1.0, 0.0, 0.0) if abs(axis[0]) < 0.8 else (0.0, 1.0, 0.0)
    basis1 = unit(cross(axis, helper))
    basis2 = cross(axis, basis1)
    polar_points: list[tuple[float, float]] = []
    for entity_id, value in step.records.items():
        if not value.startswith("CARTESIAN_POINT"):
            continue
        relative = sub(step.vec3(entity_id), center)
        x_value = dot(relative, basis1)
        y_value = dot(relative, basis2)
        polar_points.append(
            (math.hypot(x_value, y_value), math.atan2(y_value, x_value) % (2 * math.pi))
        )
    outer_radius = max(radius for radius, _ in polar_points)
    outer_angles = {
        round(angle, 8)
        for radius, angle in polar_points
        if radius > outer_radius * 0.999
    }
    estimated_teeth = len(outer_angles) // 2 if len(outer_angles) % 2 == 0 else None
    return {
        "component": step_path.stem,
        "outer_radius_mm": outer_radius,
        "outer_tip_corner_count": len(outer_angles),
        "estimated_teeth": estimated_teeth,
        "method": "two_outer_tip_corners_per_tooth",
        "status": "cad_geometry_estimate_requires_confirmation",
    }


def extract_sprocket_evidence(assembly_path: Path) -> dict[str, object]:
    names = ["主动链轮AA", "主动链轮BA", "大腿从动链轮aA", "小腿从动链轮1aA"]
    sprockets = [
        estimate_sprocket_teeth(assembly_path.parent / f"{name}.STEP")
        for name in names
    ]
    tooth_counts = [item["estimated_teeth"] for item in sprockets]
    common_count = tooth_counts[0] if len(set(tooth_counts)) == 1 else None
    return {
        "sprockets": sprockets,
        "common_estimated_tooth_count": common_count,
        "active_to_driven_speed_ratio_magnitude": 1.0 if common_count else None,
        "rotation_sign": None,
        "zero_offset_rad": None,
    }


def write_outputs(
    output_dir: Path,
    clusters: list[list[Axis]],
    source: Path,
    skipped: list[str],
    transmission_evidence: list[dict[str, object]],
    wheel_attachment_evidence: dict[str, object],
    sprocket_evidence: dict[str, object],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for cluster_id, cluster in enumerate(clusters, start=1):
        reference = cluster[0]
        components = sorted({axis.component for axis in cluster})
        rows.append(
            {
                "candidate_id": cluster_id,
                "components": components,
                "point_mm": list(reference.point),
                "direction": list(reference.direction),
                "observations": [
                    {"component": axis.component, "radius_mm": axis.radius_mm}
                    for axis in cluster
                ],
            }
        )

    component_candidates: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        for component in row["components"]:
            component_candidates.setdefault(component, []).append(
                {"candidate_id": row["candidate_id"], "point_mm": row["point_mm"]}
            )
    distances = []
    for component, candidates in sorted(component_candidates.items()):
        for left_index, left in enumerate(candidates):
            for right in candidates[left_index + 1 :]:
                delta = sub(tuple(right["point_mm"]), tuple(left["point_mm"]))  # type: ignore[arg-type]
                distances.append(
                    {
                        "component": component,
                        "from_candidate": left["candidate_id"],
                        "to_candidate": right["candidate_id"],
                        "distance_mm": norm(delta),
                    }
                )

    payload = {
        "status": "candidate_only_requires_manual_confirmation",
        "source_assembly": source.name,
        "coordinate_unit": "mm",
        "components_without_root_transform": skipped,
        "candidates": rows,
        "same_component_axis_distances": distances,
        "transmission_axis_evidence": transmission_evidence,
        "wheel_attachment_evidence": wheel_attachment_evidence,
        "sprocket_geometry_evidence": sprocket_evidence,
    }
    (output_dir / "joint_axis_candidates.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    with (output_dir / "joint_axis_candidates.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        writer.writerow(["candidate_id", "components", "point_x_mm", "point_y_mm", "point_z_mm", "dir_x", "dir_y", "dir_z"])
        for row in rows:
            writer.writerow(
                [row["candidate_id"], " + ".join(row["components"]), *row["point_mm"], *row["direction"]]
            )
    with (output_dir / "link_axis_distances.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        writer.writerow(["component", "from_candidate", "to_candidate", "distance_mm"])
        for row in distances:
            writer.writerow(
                [row["component"], row["from_candidate"], row["to_candidate"], row["distance_mm"]]
            )

    report_lines = [
        "# STEP 几何自动提取记录",
        "",
        f"源装配：`{source.name}`。所有坐标和长度单位均为 mm。",
        "",
        "以下结果来自不同零件圆柱面轴线的装配坐标重合，只能作为关节候选；编码器零位、主动轴和机械拓扑仍需结合 SolidWorks 装配确认。",
        "",
        "| 候选轴 | 共同零件 | Y | Z | 方向 |",
        "|---:|---|---:|---:|---|",
    ]
    for row in rows:
        point = row["point_mm"]
        direction = row["direction"]
        report_lines.append(
            f"| {row['candidate_id']} | {' + '.join(row['components'])} | "
            f"{point[1]:.6f} | {point[2]:.6f} | "
            f"[{direction[0]:.6f}, {direction[1]:.6f}, {direction[2]:.6f}] |"
        )
    report_lines.extend(
        [
            "",
            "| 零件 | 候选轴 | 轴距 |",
            "|---|---:|---:|",
        ]
    )
    for row in distances:
        report_lines.append(
            f"| {row['component']} | {row['from_candidate']} -> {row['to_candidate']} | "
            f"{row['distance_mm']:.6f} |"
        )
    if skipped:
        report_lines.extend(
            [
                "",
                "## 未定位零件",
                "",
                "根装配中没有找到以下零件的直接装配变换：" + "、".join(skipped) + "。",
            ]
        )
    report_lines.extend(
        [
            "",
            "## 传动轴证据",
            "",
            "| 零件 | 识别结果 | 共轴候选关节 |",
            "|---|---|---:|",
        ]
    )
    for item in transmission_evidence:
        candidate_id = item.get("structural_candidate_id")
        candidate_text = f"J{candidate_id}" if candidate_id is not None else "-"
        report_lines.append(
            f"| {item['component']} | {item['status']} | {candidate_text} |"
        )
    report_lines.extend(
        [
            "",
            "## 轮轴刚性归属证据",
            "",
            f"轮心轴坐标：`{wheel_attachment_evidence['wheel_axis_point_mm']}` mm，"
            f"轮外半径：`{wheel_attachment_evidence['wheel_outer_radius_mm']:.3f}` mm。",
            "",
            "| 机构构件 | 减速箱构件 | 重合孔轴数 |",
            "|---|---|---:|",
        ]
    )
    for item in wheel_attachment_evidence["rigid_attachment_matches"]:
        report_lines.append(
            f"| {item['structural_component']} | {item['gearbox_component']} | "
            f"{item['coincident_axis_count']} |"
        )
    report_lines.extend(
        [
            "",
            "## 链轮齿形证据",
            "",
            "| 链轮 | 外圆齿尖角点 | 估计齿数 | 状态 |",
            "|---|---:|---:|---|",
        ]
    )
    for item in sprocket_evidence["sprockets"]:
        report_lines.append(
            f"| {item['component']} | {item['outer_tip_corner_count']} | "
            f"{item['estimated_teeth']} | {item['status']} |"
        )
    report_lines.extend(
        [
            "",
            f"链传动比幅值：`{sprocket_evidence['active_to_driven_speed_ratio_magnitude']}`；"
            "旋转符号和零位仍待实机标定。",
        ]
    )
    (output_dir / "EXTRACTION_REPORT.md").write_text(
        "\n".join(report_lines) + "\n", encoding="utf-8"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("assembly", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--distance-mm", type=float, default=0.05)
    parser.add_argument("--angle-deg", type=float, default=0.05)
    args = parser.parse_args()

    required_targets = ["髋关节A", "连杆1改A", "连杆2改A", "连杆3A", "连杆4改A", "连杆5A"]
    optional_targets = ["小腿A"]
    targets = required_targets + optional_targets
    all_axes: list[Axis] = []
    missing: list[str] = []
    for target in targets:
        step_path = args.assembly.parent / f"{target}.STEP"
        transform = resolve_component_transform(args.assembly, target)
        if transform is None or not step_path.exists():
            missing.append(target)
            continue
        all_axes.extend(component_axes(step_path, transform))

    required_missing = [target for target in missing if target in required_targets]
    if required_missing:
        raise SystemExit(f"missing required assembly transforms or files: {', '.join(required_missing)}")
    clusters = cluster_axes(all_axes, args.distance_mm, args.angle_deg)
    clusters.sort(key=lambda group: (-len({axis.component for axis in group}), group[0].point))
    transmission_evidence = extract_transmission_evidence(args.assembly, clusters)
    wheel_attachment_evidence = extract_wheel_attachment_evidence(args.assembly)
    sprocket_evidence = extract_sprocket_evidence(args.assembly)
    write_outputs(
        args.output,
        clusters,
        args.assembly,
        missing,
        transmission_evidence,
        wheel_attachment_evidence,
        sprocket_evidence,
    )
    print(
        f"components={len(targets) - len(missing)} axes={len(all_axes)} "
        f"candidates={len(clusters)} skipped={len(missing)}"
    )


if __name__ == "__main__":
    main()
