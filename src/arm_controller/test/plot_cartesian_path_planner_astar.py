#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np
try:
    import plotly.graph_objects as go
except ImportError:
    go = None


def _load_case(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _xy(points: List[List[float]]) -> Tuple[List[float], List[float]]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return xs, ys


def _orientation_dirs(case: Dict) -> List[np.ndarray]:
    mats = case.get("path_orientations", [])
    dirs: List[np.ndarray] = []
    for flat in mats:
        if not isinstance(flat, list) or len(flat) != 9:
            dirs.append(np.array([1.0, 0.0, 0.0]))
            continue
        R = np.array(flat, dtype=float).reshape(3, 3)
        d = R[:, 0]
        n = np.linalg.norm(d)
        if n < 1e-9:
            d = np.array([1.0, 0.0, 0.0])
        else:
            d = d / n
        dirs.append(d)
    return dirs


def plot_case(case: Dict, out_png: Path) -> None:
    map_min = case["map_min"]
    map_max = case["map_max"]
    start = case["start"]
    goal = case["goal"]
    spheres = case.get("spheres", [])
    boxes = case.get("boxes", [])
    path = case.get("path", [])
    success = bool(case.get("success", False))
    case_name = case.get("case", out_png.stem)

    fig, ax = plt.subplots(figsize=(7, 7))

    map_rect = Rectangle(
        (map_min[0], map_min[1]),
        map_max[0] - map_min[0],
        map_max[1] - map_min[1],
        fill=False,
        edgecolor="black",
        linewidth=1.5,
        linestyle="--",
    )
    ax.add_patch(map_rect)

    for s in spheres:
        c = s["center"]
        r = s["radius"]
        ax.add_patch(Circle((c[0], c[1]), r, facecolor="#fca5a5", edgecolor="#dc2626", alpha=0.5))

    for b in boxes:
        bmin = b["min"]
        bmax = b["max"]
        ax.add_patch(
            Rectangle(
                (bmin[0], bmin[1]),
                bmax[0] - bmin[0],
                bmax[1] - bmin[1],
                facecolor="#93c5fd",
                edgecolor="#1d4ed8",
                alpha=0.5,
            )
        )

    ax.scatter([start[0]], [start[1]], c="#16a34a", s=70, marker="o", label="start")
    ax.scatter([goal[0]], [goal[1]], c="#ea580c", s=90, marker="*", label="goal")

    if path:
        xs, ys = _xy(path)
        ax.plot(xs, ys, color="#111827", linewidth=2.0, label=f"path ({len(path)} pts)")
        ax.scatter(xs, ys, color="#374151", s=12)

    ax.set_xlim(map_min[0] - 0.05, map_max[0] + 0.05)
    ax.set_ylim(map_min[1] - 0.05, map_max[1] + 0.05)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle=":", linewidth=0.7, alpha=0.6)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"{case_name} | {'SUCCESS' if success else 'FAILED'}")
    ax.legend(loc="best")

    fig.tight_layout()
    fig.savefig(out_png, dpi=160)
    plt.close(fig)


def _draw_box_3d(ax, bmin, bmax, color="#1d4ed8", alpha=0.20):
    x0, y0, z0 = bmin
    x1, y1, z1 = bmax
    corners = np.array(
        [
            [x0, y0, z0],
            [x1, y0, z0],
            [x1, y1, z0],
            [x0, y1, z0],
            [x0, y0, z1],
            [x1, y0, z1],
            [x1, y1, z1],
            [x0, y1, z1],
        ]
    )
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    for i, j in edges:
        ax.plot(
            [corners[i, 0], corners[j, 0]],
            [corners[i, 1], corners[j, 1]],
            [corners[i, 2], corners[j, 2]],
            color=color,
            alpha=alpha + 0.35,
            linewidth=1.2,
        )


def _draw_sphere_3d(ax, center, radius, color="#dc2626", alpha=0.18):
    u = np.linspace(0, 2 * np.pi, 28)
    v = np.linspace(0, np.pi, 16)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, color=color, alpha=alpha, linewidth=0, shade=False)


def plot_case_3d(case: Dict, out_png: Path) -> None:
    map_min = case["map_min"]
    map_max = case["map_max"]
    start = case["start"]
    goal = case["goal"]
    spheres = case.get("spheres", [])
    boxes = case.get("boxes", [])
    path = case.get("path", [])
    dirs = _orientation_dirs(case)
    success = bool(case.get("success", False))
    case_name = case.get("case", out_png.stem)

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")

    for s in spheres:
        _draw_sphere_3d(ax, s["center"], s["radius"])
    for b in boxes:
        _draw_box_3d(ax, b["min"], b["max"])

    ax.scatter([start[0]], [start[1]], [start[2]], c="#16a34a", s=60, marker="o", label="start")
    ax.scatter([goal[0]], [goal[1]], [goal[2]], c="#ea580c", s=90, marker="*", label="goal")

    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        zs = [p[2] for p in path]
        ax.plot(xs, ys, zs, color="#111827", linewidth=2.0, label=f"path ({len(path)} pts)")
        ax.scatter(xs, ys, zs, color="#374151", s=12)

        if len(dirs) == len(path):
            stride = max(1, len(path) // 20)
            arrow_len = 0.05
            qx, qy, qz, ux, uy, uz = [], [], [], [], [], []
            for i in range(0, len(path), stride):
                qx.append(path[i][0])
                qy.append(path[i][1])
                qz.append(path[i][2])
                ux.append(float(dirs[i][0]) * arrow_len)
                uy.append(float(dirs[i][1]) * arrow_len)
                uz.append(float(dirs[i][2]) * arrow_len)
            ax.quiver(qx, qy, qz, ux, uy, uz, color="#ef4444", linewidth=1.0, normalize=False)

    ax.set_xlim(map_min[0], map_max[0])
    ax.set_ylim(map_min[1], map_max[1])
    ax.set_zlim(map_min[2], map_max[2])
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(f"{case_name} 3D | {'SUCCESS' if success else 'FAILED'}")
    ax.view_init(elev=22, azim=-55)
    ax.legend(loc="upper left")

    fig.tight_layout()
    fig.savefig(out_png, dpi=170)
    plt.close(fig)


def show_case_3d(case: Dict) -> None:
    map_min = case["map_min"]
    map_max = case["map_max"]
    start = case["start"]
    goal = case["goal"]
    spheres = case.get("spheres", [])
    boxes = case.get("boxes", [])
    path = case.get("path", [])
    dirs = _orientation_dirs(case)
    success = bool(case.get("success", False))
    case_name = case.get("case", "case")

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")

    for s in spheres:
        _draw_sphere_3d(ax, s["center"], s["radius"])
    for b in boxes:
        _draw_box_3d(ax, b["min"], b["max"])

    ax.scatter([start[0]], [start[1]], [start[2]], c="#16a34a", s=60, marker="o", label="start")
    ax.scatter([goal[0]], [goal[1]], [goal[2]], c="#ea580c", s=90, marker="*", label="goal")

    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        zs = [p[2] for p in path]
        ax.plot(xs, ys, zs, color="#111827", linewidth=2.0, label=f"path ({len(path)} pts)")
        ax.scatter(xs, ys, zs, color="#374151", s=12)
        if len(dirs) == len(path):
            stride = max(1, len(path) // 20)
            arrow_len = 0.05
            qx, qy, qz, ux, uy, uz = [], [], [], [], [], []
            for i in range(0, len(path), stride):
                qx.append(path[i][0])
                qy.append(path[i][1])
                qz.append(path[i][2])
                ux.append(float(dirs[i][0]) * arrow_len)
                uy.append(float(dirs[i][1]) * arrow_len)
                uz.append(float(dirs[i][2]) * arrow_len)
            ax.quiver(qx, qy, qz, ux, uy, uz, color="#ef4444", linewidth=1.0, normalize=False)

    ax.set_xlim(map_min[0], map_max[0])
    ax.set_ylim(map_min[1], map_max[1])
    ax.set_zlim(map_min[2], map_max[2])
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(f"{case_name} 3D (interactive window) | {'SUCCESS' if success else 'FAILED'}")
    ax.view_init(elev=22, azim=-55)
    ax.legend(loc="upper left")
    plt.tight_layout()
    plt.show()


def _box_edges(bmin: List[float], bmax: List[float]) -> List[Tuple[List[float], List[float], List[float]]]:
    x0, y0, z0 = bmin
    x1, y1, z1 = bmax
    corners = [
        [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
        [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
    ]
    edge_ids = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    edges = []
    for i, j in edge_ids:
        edges.append(
            (
                [corners[i][0], corners[j][0]],
                [corners[i][1], corners[j][1]],
                [corners[i][2], corners[j][2]],
            )
        )
    return edges


def plot_case_3d_interactive(case: Dict, out_html: Path) -> bool:
    if go is None:
        return False

    map_min = case["map_min"]
    map_max = case["map_max"]
    start = case["start"]
    goal = case["goal"]
    spheres = case.get("spheres", [])
    boxes = case.get("boxes", [])
    path = case.get("path", [])
    dirs = _orientation_dirs(case)
    success = bool(case.get("success", False))
    case_name = case.get("case", out_html.stem)

    fig = go.Figure()

    for s in spheres:
        c = s["center"]
        r = s["radius"]
        u = np.linspace(0, 2 * np.pi, 36)
        v = np.linspace(0, np.pi, 24)
        x = c[0] + r * np.outer(np.cos(u), np.sin(v))
        y = c[1] + r * np.outer(np.sin(u), np.sin(v))
        z = c[2] + r * np.outer(np.ones_like(u), np.cos(v))
        fig.add_trace(
            go.Surface(
                x=x,
                y=y,
                z=z,
                opacity=0.30,
                showscale=False,
                colorscale=[[0.0, "#f87171"], [1.0, "#dc2626"]],
                name="sphere obstacle",
            )
        )

    for b in boxes:
        for ex, ey, ez in _box_edges(b["min"], b["max"]):
            fig.add_trace(
                go.Scatter3d(
                    x=ex,
                    y=ey,
                    z=ez,
                    mode="lines",
                    line={"color": "#2563eb", "width": 5},
                    showlegend=False,
                    hoverinfo="skip",
                )
            )

    fig.add_trace(
        go.Scatter3d(
            x=[start[0]],
            y=[start[1]],
            z=[start[2]],
            mode="markers",
            marker={"size": 6, "color": "#16a34a"},
            name="start",
        )
    )
    fig.add_trace(
        go.Scatter3d(
            x=[goal[0]],
            y=[goal[1]],
            z=[goal[2]],
            mode="markers",
            marker={"size": 7, "color": "#ea580c", "symbol": "diamond"},
            name="goal",
        )
    )

    if path:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        pz = [p[2] for p in path]
        fig.add_trace(
            go.Scatter3d(
                x=px,
                y=py,
                z=pz,
                mode="lines+markers",
                line={"color": "#111827", "width": 7},
                marker={"size": 3, "color": "#374151"},
                name=f"path ({len(path)} pts)",
            )
        )
        if len(dirs) == len(path):
            stride = max(1, len(path) // 20)
            arrow_len = 0.05
            axx, ayy, azz = [], [], []
            for i in range(0, len(path), stride):
                p = path[i]
                d = dirs[i] * arrow_len
                axx.extend([p[0], p[0] + float(d[0]), None])
                ayy.extend([p[1], p[1] + float(d[1]), None])
                azz.extend([p[2], p[2] + float(d[2]), None])
            fig.add_trace(
                go.Scatter3d(
                    x=axx,
                    y=ayy,
                    z=azz,
                    mode="lines",
                    line={"color": "#ef4444", "width": 4},
                    name="orientation x-axis",
                )
            )

    fig.update_layout(
        title=f"{case_name} 3D (interactive) | {'SUCCESS' if success else 'FAILED'}",
        scene={
            "xaxis": {"title": "X (m)", "range": [map_min[0], map_max[0]]},
            "yaxis": {"title": "Y (m)", "range": [map_min[1], map_max[1]]},
            "zaxis": {"title": "Z (m)", "range": [map_min[2], map_max[2]]},
            "aspectmode": "cube",
            "camera": {"eye": {"x": 1.4, "y": -1.5, "z": 1.1}},
        },
        margin={"l": 0, "r": 0, "t": 40, "b": 0},
        legend={"x": 0.02, "y": 0.98},
    )
    fig.write_html(str(out_html), include_plotlyjs=True, full_html=True)
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot Cartesian path planner A* test cases from JSON.")
    parser.add_argument("--input", required=True, help="Directory containing case_*.json")
    parser.add_argument("--output", default="", help="Output directory for PNGs (default: same as input)")
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Also generate interactive 3D HTML (requires plotly).",
    )
    parser.add_argument(
        "--show-3d",
        action="store_true",
        help="Show interactive matplotlib 3D window for each case (rotate/zoom with mouse).",
    )
    args = parser.parse_args()

    in_dir = Path(args.input).expanduser().resolve()
    out_dir = Path(args.output).expanduser().resolve() if args.output else in_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    json_files = sorted(in_dir.glob("case_*.json"))
    if not json_files:
        raise SystemExit(f"No case_*.json found in {in_dir}")

    for jf in json_files:
        case = _load_case(jf)
        out_xy = out_dir / f"{jf.stem}_xy.png"
        out_3d = out_dir / f"{jf.stem}_3d.png"
        plot_case(case, out_xy)
        plot_case_3d(case, out_3d)
        print(f"[OK] {out_xy}")
        print(f"[OK] {out_3d}")
        if args.interactive:
            out_html = out_dir / f"{jf.stem}_3d_interactive.html"
            if plot_case_3d_interactive(case, out_html):
                print(f"[OK] {out_html}")
            else:
                print("[WARN] plotly is not installed. Skip interactive HTML.")
        if args.show_3d:
            show_case_3d(case)


if __name__ == "__main__":
    main()
