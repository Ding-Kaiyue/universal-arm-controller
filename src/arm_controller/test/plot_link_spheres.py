#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def _draw_sphere(ax, center, radius, color):
    u = np.linspace(0, 2 * np.pi, 28)
    v = np.linspace(0, np.pi, 18)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, color=color, alpha=0.20, linewidth=0, shade=False)


def _basis_from_direction(direction):
    d = np.asarray(direction, dtype=float)
    n = np.linalg.norm(d)
    if n < 1e-9:
        d = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        d = d / n
    helper = np.array([0.0, 0.0, 1.0], dtype=float)
    if abs(np.dot(d, helper)) > 0.95:
        helper = np.array([0.0, 1.0, 0.0], dtype=float)
    x_axis = np.cross(helper, d)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(d, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    # Columns: local x/y/z in world
    return np.column_stack((x_axis, y_axis, d))


def _draw_oriented_ellipsoid(ax, center, R, radii, color):
    # Parametric ellipsoid in local frame, then rotate to world.
    u = np.linspace(0, 2 * np.pi, 36)
    v = np.linspace(0, np.pi, 20)
    cu, su = np.cos(u), np.sin(u)
    sv, cv = np.sin(v), np.cos(v)
    X = radii[0] * np.outer(cu, sv)
    Y = radii[1] * np.outer(su, sv)
    Z = radii[2] * np.outer(np.ones_like(u), cv)
    pts = np.stack((X, Y, Z), axis=-1)  # (nu, nv, 3)
    pts_world = pts @ R.T + np.asarray(center, dtype=float)[None, None, :]
    ax.plot_surface(
        pts_world[:, :, 0],
        pts_world[:, :, 1],
        pts_world[:, :, 2],
        color=color,
        alpha=0.20,
        linewidth=0,
        shade=False,
    )


def _fit_link_ellipsoid(centers, radii):
    pts = np.asarray(centers, dtype=float)
    rs = np.asarray(radii, dtype=float)
    c_mean = np.mean(pts, axis=0)
    if len(pts) >= 2:
        cov = np.cov((pts - c_mean).T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        axis = eigvecs[:, int(np.argmax(eigvals))]
        axis /= (np.linalg.norm(axis) + 1e-12)
    else:
        axis = np.array([0.0, 0.0, 1.0], dtype=float)

    t = (pts - c_mean) @ axis
    t_min, t_max = float(np.min(t)), float(np.max(t))
    long_half = 0.5 * (t_max - t_min) + float(np.mean(rs)) * 0.85
    short_half = float(np.mean(rs)) * 0.62

    center = c_mean + 0.5 * (t_min + t_max) * axis
    R = _basis_from_direction(axis)
    radii_ell = np.array([short_half, short_half, long_half], dtype=float)
    return center, R, radii_ell


def main():
    parser = argparse.ArgumentParser(description="Plot link collision spheres in world frame.")
    parser.add_argument(
        "--input",
        required=True,
        nargs="+",
        help="Path(s) to link_spheres_*.json, e.g. left and right arm together",
    )
    parser.add_argument("--show-3d", action="store_true", help="Show interactive matplotlib 3D window.")
    parser.add_argument("--output", default="", help="Output png path (default: input stem + _3d.png)")
    parser.add_argument(
        "--draw-mode",
        choices=["ellipsoid", "sphere"],
        default="ellipsoid",
        help="ellipsoid: one fitted ellipsoid per link (recommended), sphere: draw every sphere",
    )
    args = parser.parse_args()

    datasets = []
    for in_item in args.input:
        in_path = Path(in_item).expanduser().resolve()
        with in_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        spheres = data.get("spheres", [])
        ellipsoids = data.get("ellipsoids", [])
        if not spheres and not ellipsoids:
            continue
        datasets.append((in_path, data, spheres, ellipsoids))
    if not datasets:
        raise SystemExit("No spheres in input JSON(s).")

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")

    arm_colors = plt.cm.Set2(np.linspace(0.0, 1.0, max(len(datasets), 3)))
    xs, ys, zs = [], [], []
    legend_names = []
    for di, (_, data, spheres, ellipsoids) in enumerate(datasets):
        mapping = data.get("mapping", f"arm_{di}")
        color = arm_colors[di % len(arm_colors)]
        legend_names.append(mapping)

        link_groups = {}
        if ellipsoids:
            for e in ellipsoids:
                ln = e["link_name"]
                c = np.asarray(e["center_world"], dtype=float)
                radii = np.asarray(e["radii"], dtype=float)
                R_raw = np.asarray(e["rotation_world"], dtype=float)
                R = R_raw.reshape(3, 3)
                xs.append(c[0])
                ys.append(c[1])
                zs.append(c[2])
                if args.draw_mode == "sphere":
                    _draw_sphere(ax, c, float(np.max(radii)), color=color)
                else:
                    _draw_oriented_ellipsoid(ax, c, R, radii, color=color)
                    d = R[:, 2]
                    l = radii[2]
                    ax.quiver(
                        c[0], c[1], c[2],
                        d[0] * l, d[1] * l, d[2] * l,
                        color=color, linewidth=1.2, arrow_length_ratio=0.12
                    )
                ax.scatter([c[0]], [c[1]], [c[2]], color=color, s=10)
                ax.text(c[0], c[1], c[2], ln, fontsize=7, color=color)
            continue
        else:
            for s in spheres:
                ln = s["link_name"]
                c = np.asarray(s["center_world"], dtype=float)
                r = float(s["radius"])
                link_groups.setdefault(ln, {"centers": [], "radii": []})
                link_groups[ln]["centers"].append(c)
                link_groups[ln]["radii"].append(r)
                xs.append(c[0])
                ys.append(c[1])
                zs.append(c[2])

        if args.draw_mode == "sphere":
            for ln, g in link_groups.items():
                for c, r in zip(g["centers"], g["radii"]):
                    _draw_sphere(ax, c, r, color=color)
                    ax.scatter([c[0]], [c[1]], [c[2]], color=color, s=12)
                c0 = np.mean(np.asarray(g["centers"]), axis=0)
                ax.text(c0[0], c0[1], c0[2], ln, fontsize=7, color=color)
        else:
            # Recommended: one ellipsoid per link, long-axis aligned with link direction.
            for ln, g in link_groups.items():
                center, R, radii_ell = _fit_link_ellipsoid(g["centers"], g["radii"])
                _draw_oriented_ellipsoid(ax, center, R, radii_ell, color=color)
                ax.scatter([center[0]], [center[1]], [center[2]], color=color, s=10)
                # Axis arrow for intuition (link direction = local z of ellipsoid)
                d = R[:, 2]
                l = radii_ell[2]
                ax.quiver(
                    center[0], center[1], center[2],
                    d[0] * l, d[1] * l, d[2] * l,
                    color=color, linewidth=1.2, arrow_length_ratio=0.12
                )
                ax.text(center[0], center[1], center[2], ln, fontsize=7, color=color)

    # Axis equal-ish
    min_xyz = np.array([min(xs), min(ys), min(zs)], dtype=float)
    max_xyz = np.array([max(xs), max(ys), max(zs)], dtype=float)
    center = 0.5 * (min_xyz + max_xyz)
    span = float(np.max(max_xyz - min_xyz) + 0.15)
    span = max(span, 0.4)

    ax.set_xlim(center[0] - span / 2, center[0] + span / 2)
    ax.set_ylim(center[1] - span / 2, center[1] + span / 2)
    ax.set_zlim(center[2] - span / 2, center[2] + span / 2)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    mode_txt = "Link Ellipsoids (long-axis along link)" if args.draw_mode == "ellipsoid" else "Raw Link Spheres"
    robot_name = datasets[0][1].get("robot_type", "")
    ax.set_title(f"{mode_txt} | mappings={','.join(legend_names)} | robot={robot_name}")
    ax.view_init(elev=22, azim=-58)
    plt.tight_layout()

    if args.output:
        out_png = Path(args.output).expanduser().resolve()
    else:
        if len(datasets) == 1:
            out_png = datasets[0][0].with_name(datasets[0][0].stem + f"_{args.draw_mode}_3d.png")
        else:
            out_png = Path("/tmp/link_sphere_viz/dual_arms_" + args.draw_mode + "_3d.png")
            out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=160)
    print(f"[OK] {out_png}")

    if args.show_3d:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    main()
