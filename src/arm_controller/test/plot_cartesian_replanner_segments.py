#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

try:
    import plotly.graph_objects as go
except ImportError as exc:
    go = None


def load_data(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def load_link_sphere_data(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def add_sphere(fig, center, radius):
    u = np.linspace(0, 2 * np.pi, 36)
    v = np.linspace(0, np.pi, 24)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    fig.add_trace(
        go.Surface(
            x=x,
            y=y,
            z=z,
            opacity=0.30,
            showscale=False,
            colorscale=[[0.0, "#f87171"], [1.0, "#dc2626"]],
            name="obstacle_sphere",
        )
    )


def _ellipsoid_surface(center, radii, rotation, nu=18, nv=12):
    u = np.linspace(0.0, 2.0 * np.pi, nu)
    v = np.linspace(0.0, np.pi, nv)
    uu, vv = np.meshgrid(u, v, indexing="ij")
    xl = radii[0] * np.cos(uu) * np.sin(vv)
    yl = radii[1] * np.sin(uu) * np.sin(vv)
    zl = radii[2] * np.cos(vv)
    local = np.stack([xl, yl, zl], axis=-1)
    R = np.array(rotation, dtype=float).reshape(3, 3)
    world = local @ R.T + np.array(center, dtype=float)
    return world[..., 0], world[..., 1], world[..., 2]


def _add_ik_ellipsoids_plotly(fig, data, tag):
    ik_records = data.get("ik_records", [])
    shown_labels = set()
    for rec in ik_records:
        label = rec.get("label", "ik")
        ik_ok = bool(rec.get("ik_ok", False))
        collision_free = bool(rec.get("collision_free", False))
        external_ik_ok = bool(rec.get("external_ik_ok", ik_ok))
        fallback_ik_used = bool(rec.get("fallback_ik_used", False))
        if not ik_ok:
            color = "#ef4444"
            marker_symbol = "x"
        elif not collision_free:
            color = "#f59e0b"
            marker_symbol = "diamond-open"
        elif fallback_ik_used and not external_ik_ok:
            color = "#14b8a6"
            marker_symbol = "square"
        elif "handoff" in label:
            color = "#a855f7"
            marker_symbol = "diamond"
        elif "goal" in label:
            color = "#f59e0b"
            marker_symbol = "diamond"
        else:
            color = "#22c55e"
            marker_symbol = "diamond"
        for ell in rec.get("ellipsoids", []):
            x, y, z = _ellipsoid_surface(
                ell.get("center_world", [0, 0, 0]),
                ell.get("radii", [0.01, 0.01, 0.01]),
                ell.get("rotation_world", [1, 0, 0, 0, 1, 0, 0, 0, 1]),
                nu=14,
                nv=10,
            )
            fig.add_trace(
                go.Surface(
                    x=x,
                    y=y,
                    z=z,
                    opacity=0.12,
                    showscale=False,
                    colorscale=[[0.0, color], [1.0, color]],
                    name=f"{tag} {label} ik_ellipsoid",
                    hoverinfo="skip",
                    showlegend=False,
                )
            )
        p = rec.get("target_position", None)
        if p is not None and len(p) == 3:
            fig.add_trace(
                go.Scatter3d(
                    x=[p[0]],
                    y=[p[1]],
                    z=[p[2]],
                    mode="markers+text",
                    marker={"size": 5, "color": color, "symbol": marker_symbol},
                    text=[f"{tag}:{label}:t{rec.get('tick', 0)}"],
                    textposition="top center",
                    showlegend=False,
                    hovertemplate=f"{tag} {label}<extra></extra>",
                )
            )
        target_R = rec.get("target_orientation", None)
        if p is not None and target_R is not None and len(p) == 3 and len(target_R) == 9:
            R = np.array(target_R, dtype=float).reshape(3, 3)
            axis_len = 0.035
            axes = [
                ("x", "#ef4444", R[:, 0]),
                ("y", "#22c55e", R[:, 1]),
                ("z", "#3b82f6", R[:, 2]),
            ]
            for axis_name, axis_color, d in axes:
                fig.add_trace(
                    go.Scatter3d(
                        x=[p[0], p[0] + float(d[0]) * axis_len],
                        y=[p[1], p[1] + float(d[1]) * axis_len],
                        z=[p[2], p[2] + float(d[2]) * axis_len],
                        mode="lines",
                        line={"color": axis_color, "width": 6},
                        showlegend=False,
                        hovertemplate=f"{tag} {label} axis_{axis_name}<extra></extra>",
                    )
                )
            if label not in shown_labels:
                fig.add_trace(
                    go.Scatter3d(
                        x=[p[0]],
                        y=[p[1]],
                        z=[p[2]],
                        mode="markers+text",
                        marker={"size": 6, "color": color, "symbol": marker_symbol},
                        text=[f"{tag}:{label}:t{rec.get('tick', 0)}"],
                        textposition="bottom center",
                        name=f"{tag} {label}",
                        showlegend=True,
                        hovertemplate=f"{tag} {label}<extra></extra>",
                    )
                )
                shown_labels.add(label)


def _add_ik_ellipsoids_matplotlib(ax, data, tag):
    ik_records = data.get("ik_records", [])
    shown_labels = set()
    for rec in ik_records:
        label = rec.get("label", "ik")
        ik_ok = bool(rec.get("ik_ok", False))
        collision_free = bool(rec.get("collision_free", False))
        external_ik_ok = bool(rec.get("external_ik_ok", ik_ok))
        fallback_ik_used = bool(rec.get("fallback_ik_used", False))
        if not ik_ok:
            color = "#dc2626"
            marker = "x"
        elif not collision_free:
            color = "#f59e0b"
            marker = "D"
        elif fallback_ik_used and not external_ik_ok:
            color = "#0f766e"
            marker = "s"
        elif "handoff" in label:
            color = "#a855f7"
            marker = "D"
        elif "goal" in label:
            color = "#f59e0b"
            marker = "D"
        else:
            color = "#16a34a"
            marker = "D"
        for ell in rec.get("ellipsoids", []):
            x, y, z = _ellipsoid_surface(
                ell.get("center_world", [0, 0, 0]),
                ell.get("radii", [0.01, 0.01, 0.01]),
                ell.get("rotation_world", [1, 0, 0, 0, 1, 0, 0, 0, 1]),
                nu=14,
                nv=10,
            )
            ax.plot_wireframe(x, y, z, color=color, linewidth=0.5, alpha=0.35)
        p = rec.get("target_position", None)
        target_R = rec.get("target_orientation", None)
        if p is not None and target_R is not None and len(p) == 3 and len(target_R) == 9:
            R = np.array(target_R, dtype=float).reshape(3, 3)
            axis_len = 0.035
            ax.quiver(
                [p[0]], [p[1]], [p[2]],
                [R[0, 0] * axis_len], [R[1, 0] * axis_len], [R[2, 0] * axis_len],
                color="#ef4444", linewidth=1.6, normalize=False
            )
            ax.quiver(
                [p[0]], [p[1]], [p[2]],
                [R[0, 1] * axis_len], [R[1, 1] * axis_len], [R[2, 1] * axis_len],
                color="#22c55e", linewidth=1.6, normalize=False
            )
            ax.quiver(
                [p[0]], [p[1]], [p[2]],
                [R[0, 2] * axis_len], [R[1, 2] * axis_len], [R[2, 2] * axis_len],
                color="#3b82f6", linewidth=1.6, normalize=False
            )
            if label not in shown_labels:
                ax.scatter(
                    [p[0]], [p[1]], [p[2]],
                    color=color, marker=marker, s=42,
                    label=f"{tag} {label}",
                )
                shown_labels.add(label)


def _segment_orientation_dirs(seg, fixed_dir=None):
    pts = seg.get("points", [])
    dirs = []
    if fixed_dir is not None:
        d = np.asarray(fixed_dir, dtype=float)
        n = np.linalg.norm(d)
        if n < 1e-9:
            d = np.array([0.0, 0.0, 1.0], dtype=float)
        else:
            d = d / n
        return [d.copy() for _ in pts]
    ori = seg.get("orientations", [])
    if isinstance(ori, list) and len(ori) == len(pts):
        for flat in ori:
            if not isinstance(flat, list) or len(flat) != 9:
                dirs.append(np.array([0.0, 0.0, 1.0]))
                continue
            R = np.array(flat, dtype=float).reshape(3, 3)
            # Use tool z-axis as orientation direction (closer to real EE pointing direction).
            d = R[:, 2]
            n = np.linalg.norm(d)
            dirs.append(np.array([0.0, 0.0, 1.0]) if n < 1e-9 else (d / n))
        return dirs

    # Backward-compatible fallback: use local tangent as direction.
    n_pts = len(pts)
    if n_pts <= 1:
        return [np.array([0.0, 0.0, 1.0])] * n_pts
    for i in range(n_pts):
        if i + 1 < n_pts:
            d = np.array(pts[i + 1], dtype=float) - np.array(pts[i], dtype=float)
        else:
            d = np.array(pts[i], dtype=float) - np.array(pts[i - 1], dtype=float)
        dn = np.linalg.norm(d)
        dirs.append(np.array([0.0, 0.0, 1.0]) if dn < 1e-9 else (d / dn))
    return dirs


def _has_valid_orientation_list(seg):
    pts = seg.get("points", [])
    ori = seg.get("orientations", [])
    return isinstance(ori, list) and len(ori) == len(pts)


def _get_map_bounds(data_list):
    mins = np.array([d["map_min"] for d in data_list], dtype=float)
    maxs = np.array([d["map_max"] for d in data_list], dtype=float)
    return mins.min(axis=0), maxs.max(axis=0)


def _expand_bounds_with_link_spheres(map_min, map_max, link_sphere_list):
    if not link_sphere_list:
        return map_min, map_max
    mins = [np.array(map_min, dtype=float)]
    maxs = [np.array(map_max, dtype=float)]
    for data in link_sphere_list:
        for e in data.get("ellipsoids", []):
            c = np.array(e.get("center_world", [0, 0, 0]), dtype=float)
            r = np.array(e.get("radii", [0.01, 0.01, 0.01]), dtype=float)
            mins.append(c - r)
            maxs.append(c + r)
        for s in data.get("spheres", []):
            c = np.array(s.get("center_world", [0, 0, 0]), dtype=float)
            rr = float(s.get("radius", 0.01))
            r = np.array([rr, rr, rr], dtype=float)
            mins.append(c - r)
            maxs.append(c + r)
    mins = np.array(mins, dtype=float)
    maxs = np.array(maxs, dtype=float)
    return mins.min(axis=0), maxs.max(axis=0)


def _draw_link_spheres_plotly(fig, link_sphere_list):
    for data_idx, data in enumerate(link_sphere_list):
        tag = Path(data.get("_source_path", f"link_{data_idx}")).stem
        ellipsoids = data.get("ellipsoids", [])
        spheres = data.get("spheres", [])
        if ellipsoids:
            for e in ellipsoids:
                x, y, z = _ellipsoid_surface(
                    e.get("center_world", [0, 0, 0]),
                    e.get("radii", [0.01, 0.01, 0.01]),
                    e.get("rotation_world", [1, 0, 0, 0, 1, 0, 0, 0, 1]),
                    nu=18,
                    nv=12,
                )
                fig.add_trace(
                    go.Surface(
                        x=x,
                        y=y,
                        z=z,
                        opacity=0.20,
                        showscale=False,
                        colorscale=[[0.0, "#f59e0b"], [1.0, "#f59e0b"]],
                        name=f"{tag} real_ellipsoid",
                        hoverinfo="skip",
                        showlegend=False,
                    )
                )
        else:
            for s in spheres:
                c = s.get("center_world", [0, 0, 0])
                r = float(s.get("radius", 0.01))
                u = np.linspace(0, 2 * np.pi, 24)
                v = np.linspace(0, np.pi, 16)
                x = c[0] + r * np.outer(np.cos(u), np.sin(v))
                y = c[1] + r * np.outer(np.sin(u), np.sin(v))
                z = c[2] + r * np.outer(np.ones_like(u), np.cos(v))
                fig.add_trace(
                    go.Surface(
                        x=x, y=y, z=z,
                        opacity=0.20,
                        showscale=False,
                        colorscale=[[0.0, "#f59e0b"], [1.0, "#f59e0b"]],
                        name=f"{tag} real_sphere",
                        hoverinfo="skip",
                        showlegend=False,
                    )
                )


def _draw_link_spheres_matplotlib(ax, link_sphere_list):
    for data in link_sphere_list:
        ellipsoids = data.get("ellipsoids", [])
        spheres = data.get("spheres", [])
        if ellipsoids:
            for e in ellipsoids:
                x, y, z = _ellipsoid_surface(
                    e.get("center_world", [0, 0, 0]),
                    e.get("radii", [0.01, 0.01, 0.01]),
                    e.get("rotation_world", [1, 0, 0, 0, 1, 0, 0, 0, 1]),
                    nu=18,
                    nv=12,
                )
                ax.plot_wireframe(x, y, z, color="#f59e0b", linewidth=0.6, alpha=0.55)
        else:
            for s in spheres:
                c = s.get("center_world", [0, 0, 0])
                r = float(s.get("radius", 0.01))
                u = np.linspace(0, 2 * np.pi, 24)
                v = np.linspace(0, np.pi, 16)
                x = c[0] + r * np.outer(np.cos(u), np.sin(v))
                y = c[1] + r * np.outer(np.sin(u), np.sin(v))
                z = c[2] + r * np.outer(np.ones_like(u), np.cos(v))
                ax.plot_wireframe(x, y, z, color="#f59e0b", linewidth=0.6, alpha=0.55)


def build_figure(data_list, segment_label="distance-sampled", link_sphere_list=None):
    map_min, map_max = _get_map_bounds(data_list)
    map_min, map_max = _expand_bounds_with_link_spheres(map_min, map_max, link_sphere_list or [])

    fig = go.Figure()
    for data in data_list:
        sphere = data["sphere"]
        add_sphere(fig, sphere["center"], sphere["radius"])
    _draw_link_spheres_plotly(fig, link_sphere_list or [])

    colors = [
        "#2563eb", "#16a34a", "#ea580c", "#9333ea", "#0f766e",
        "#dc2626", "#4f46e5", "#0891b2", "#65a30d", "#b45309",
    ]

    global_idx = 0
    for data_idx, data in enumerate(data_list):
        segments = data["segments"]
        arm_tag = Path(data.get("_source_path", f"input_{data_idx}")).stem
        req_s = data.get("request_start", None)
        req_g = data.get("request_goal", None)
        if isinstance(req_s, list) and len(req_s) == 3:
            fig.add_trace(
                go.Scatter3d(
                    x=[req_s[0]], y=[req_s[1]], z=[req_s[2]],
                    mode="markers+text",
                    marker={"size": 6, "color": "#06b6d4", "symbol": "circle"},
                    text=[f"{arm_tag}:req_S"],
                    textposition="top center",
                    showlegend=False,
                    hovertemplate=f"{arm_tag} request_start<extra></extra>",
                )
            )
        if isinstance(req_g, list) and len(req_g) == 3:
            fig.add_trace(
                go.Scatter3d(
                    x=[req_g[0]], y=[req_g[1]], z=[req_g[2]],
                    mode="markers+text",
                    marker={"size": 7, "color": "#f97316", "symbol": "diamond"},
                    text=[f"{arm_tag}:req_G"],
                    textposition="top center",
                    showlegend=False,
                    hovertemplate=f"{arm_tag} request_goal<extra></extra>",
                )
            )
        fixed_real_dir = None
        for i, seg in enumerate(segments):
            pts = seg.get("points", [])
            if not pts:
                continue
            col = colors[global_idx % len(colors)]
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            zs = [p[2] for p in pts]

            fig.add_trace(
                go.Scatter3d(
                    x=xs,
                    y=ys,
                    z=zs,
                    mode="lines+markers+text",
                    line={"color": col, "width": 6},
                    marker={"size": 4, "color": col, "symbol": "circle"},
                    text=[f"{arm_tag}:{i}:{k}" for k in range(len(xs))],
                    textposition="top center",
                    textfont={"size": 9, "color": col},
                    name=f"{arm_tag} {segment_label} seg {i}",
                    hovertemplate="segment %{text}<br>x=%{x:.3f}, y=%{y:.3f}, z=%{z:.3f}<extra></extra>",
                )
            )
            fig.add_trace(
                go.Scatter3d(
                    x=[xs[0]],
                    y=[ys[0]],
                    z=[zs[0]],
                    mode="markers+text",
                    marker={"size": 5, "color": col, "symbol": "circle"},
                    text=[f"S{arm_tag}:{i}"],
                    textposition="top center",
                    showlegend=False,
                    hovertemplate=f"segment {arm_tag}:{i} start<extra></extra>",
                )
            )
            fig.add_trace(
                go.Scatter3d(
                    x=[xs[-1]],
                    y=[ys[-1]],
                    z=[zs[-1]],
                    mode="markers+text",
                    marker={"size": 6, "color": col, "symbol": "diamond"},
                    text=[f"E{arm_tag}:{i}"],
                    textposition="top center",
                    showlegend=False,
                    hovertemplate=f"segment {arm_tag}:{i} end<extra></extra>",
                )
            )
            dirs = _segment_orientation_dirs(
                seg,
                fixed_dir=(fixed_real_dir if not _has_valid_orientation_list(seg) else None),
            )
            if len(dirs) == len(pts):
                arrow_len = 0.03
                axx, ayy, azz = [], [], []
                for p, d in zip(pts, dirs):
                    axx.extend([p[0], p[0] + float(d[0]) * arrow_len, None])
                    ayy.extend([p[1], p[1] + float(d[1]) * arrow_len, None])
                    azz.extend([p[2], p[2] + float(d[2]) * arrow_len, None])
                fig.add_trace(
                    go.Scatter3d(
                        x=axx,
                        y=ayy,
                        z=azz,
                        mode="lines",
                        line={"color": "#ef4444", "width": 4},
                        showlegend=(global_idx == 0),
                        name="orientation (all sampled points)",
                        hoverinfo="skip",
                    )
                )
            global_idx += 1
        _add_ik_ellipsoids_plotly(fig, data, arm_tag)

    fig.update_layout(
        title=f"Replanner 3D (Interactive) | {segment_label} | S*=start, E*=end",
        scene={
            "xaxis": {"title": "X (m)", "range": [map_min[0], map_max[0]]},
            "yaxis": {"title": "Y (m)", "range": [map_min[1], map_max[1]]},
            "zaxis": {"title": "Z (m)", "range": [map_min[2], map_max[2]]},
            "aspectmode": "cube",
            "camera": {"eye": {"x": 1.45, "y": -1.5, "z": 1.15}},
        },
        margin={"l": 0, "r": 0, "t": 42, "b": 0},
    )
    return fig


def build_figure_from_key(data_list, key, link_sphere_list=None):
    copied_list = []
    for data in data_list:
        copied = dict(data)
        copied["segments"] = data.get(key, data.get("segments", []))
        copied_list.append(copied)
    label = "raw-grid" if key == "raw_grid_segments" else "distance-sampled(segment_sample_step_m)"
    return build_figure(copied_list, segment_label=label, link_sphere_list=link_sphere_list)


def main():
    parser = argparse.ArgumentParser(description="Generate interactive 3D replanner visualization.")
    parser.add_argument("--input", nargs="+", required=True, help="Path(s) to replanner_segments.json")
    parser.add_argument("--link-spheres", nargs="*", default=[], help="Optional path(s) to link_spheres_*.json for real arm ellipsoids")
    parser.add_argument("--output", default="", help="Output html path (default: same dir/replanner_segments_3d_interactive.html)")
    parser.add_argument("--show-3d", action="store_true", help="Show draggable matplotlib 3D window (MATLAB-like).")
    parser.add_argument("--use-raw-grid", action="store_true", help="Visualize raw_grid_segments instead of distance-sampled segments.")
    args = parser.parse_args()

    in_paths = [Path(p).expanduser().resolve() for p in args.input]
    out_path = (
        Path(args.output).expanduser().resolve()
        if args.output
        else in_paths[0].parent / "replanner_segments_3d_interactive.html"
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)

    data_list = []
    for p in in_paths:
        d = load_data(p)
        d["_source_path"] = str(p)
        data_list.append(d)
    link_paths = list(args.link_spheres)
    if not link_paths:
        default_left = Path("/tmp/link_sphere_viz/link_spheres_left_arm.json")
        default_right = Path("/tmp/link_sphere_viz/link_spheres_right_arm.json")
        if default_left.exists():
            link_paths.append(str(default_left))
        if default_right.exists():
            link_paths.append(str(default_right))

    link_sphere_list = []
    for p in link_paths:
        pp = Path(p).expanduser().resolve()
        if not pp.exists():
            continue
        d = load_link_sphere_data(pp)
        d["_source_path"] = str(pp)
        link_sphere_list.append(d)
    seg_key = "raw_grid_segments" if args.use_raw_grid else "segments"

    if go is not None:
        fig = build_figure_from_key(data_list, seg_key, link_sphere_list=link_sphere_list)
        fig.write_html(str(out_path), include_plotlyjs=True, full_html=True)
        print(f"[OK] {out_path}")
    else:
        print("[WARN] plotly is not installed. Skip HTML export.")

    if args.show_3d:
        map_min, map_max = _get_map_bounds(data_list)
        map_min, map_max = _expand_bounds_with_link_spheres(map_min, map_max, link_sphere_list)

        fig = plt.figure(figsize=(8, 7))
        ax = fig.add_subplot(111, projection="3d")

        colors = plt.cm.tab10(np.linspace(0.0, 1.0, max(1, sum(len(d.get(seg_key, d.get("segments", []))) for d in data_list))))
        seg_label = "raw-grid" if seg_key == "raw_grid_segments" else "distance-sampled(segment_sample_step_m)"
        global_idx = 0
        for data_idx, data in enumerate(data_list):
            sphere = data["sphere"]
            c = sphere["center"]
            r = sphere["radius"]
            u = np.linspace(0, 2 * np.pi, 30)
            v = np.linspace(0, np.pi, 18)
            x = c[0] + r * np.outer(np.cos(u), np.sin(v))
            y = c[1] + r * np.outer(np.sin(u), np.sin(v))
            z = c[2] + r * np.outer(np.ones_like(u), np.cos(v))
            ax.plot_surface(x, y, z, color="#dc2626", alpha=0.18, linewidth=0, shade=False)

            segments = data.get(seg_key, data.get("segments", []))
            arm_tag = Path(data.get("_source_path", f"input_{data_idx}")).stem
            req_s = data.get("request_start", None)
            req_g = data.get("request_goal", None)
            if isinstance(req_s, list) and len(req_s) == 3:
                ax.scatter([req_s[0]], [req_s[1]], [req_s[2]], color="#06b6d4", marker="o", s=48)
                ax.text(req_s[0], req_s[1], req_s[2], f"{arm_tag}:req_S", color="#06b6d4", fontsize=7)
            if isinstance(req_g, list) and len(req_g) == 3:
                ax.scatter([req_g[0]], [req_g[1]], [req_g[2]], color="#f97316", marker="D", s=56)
                ax.text(req_g[0], req_g[1], req_g[2], f"{arm_tag}:req_G", color="#f97316", fontsize=7)
            fixed_real_dir = None
            for i, seg in enumerate(segments):
                pts = seg.get("points", [])
                if not pts:
                    continue
                xs = [p[0] for p in pts]
                ys = [p[1] for p in pts]
                zs = [p[2] for p in pts]
                col = colors[global_idx % len(colors)]
                ax.plot(xs, ys, zs, color=col, linewidth=2.0, label=f"{arm_tag} {seg_label} seg {i}")
                ax.scatter([xs[0]], [ys[0]], [zs[0]], color=col, marker="o", s=45)
                ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], color=col, marker="X", s=55)
                ax.text(xs[0], ys[0], zs[0], f"S{arm_tag}:{i}")
                ax.text(xs[-1], ys[-1], zs[-1], f"E{arm_tag}:{i}")
                dirs = _segment_orientation_dirs(
                    seg,
                    fixed_dir=(fixed_real_dir if not _has_valid_orientation_list(seg) else None),
                )
                if len(dirs) == len(pts):
                    qx = [p[0] for p in pts]
                    qy = [p[1] for p in pts]
                    qz = [p[2] for p in pts]
                    qu = [float(d[0]) * 0.03 for d in dirs]
                    qv = [float(d[1]) * 0.03 for d in dirs]
                    qw = [float(d[2]) * 0.03 for d in dirs]
                    ax.quiver(qx, qy, qz, qu, qv, qw, color="#ef4444", linewidth=0.8, normalize=False)
                global_idx += 1
            _add_ik_ellipsoids_matplotlib(ax, data, arm_tag)
        _draw_link_spheres_matplotlib(ax, link_sphere_list)

        ax.set_xlim(map_min[0], map_max[0])
        ax.set_ylim(map_min[1], map_max[1])
        ax.set_zlim(map_min[2], map_max[2])
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title(f"Replanner 3D (show-3d) [{seg_label}]")
        ax.view_init(elev=22, azim=-55)
        ax.legend(loc="upper left")
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
