import os
import cv2
import numpy as np
import open3d as o3d


def main(dirpath, interpolate_3d=True):
    txts = sorted([f for f in os.listdir(dirpath) if f.endswith(".txt")])
    Ys = []
    for txt in txts:
        base = txt[:-4]
        img = cv2.imread(os.path.join(dirpath, base + ".jpg"))
        y0 = np.loadtxt(os.path.join(dirpath, txt)).astype(np.float32)
        h, w = img.shape[:2]
        y = np.interp(np.linspace(0, len(y0) - 1, 500), np.arange(len(y0)), y0)
        y = np.clip(y, 0, h - 1)
        x_px = np.linspace(0, w - 1, 500).astype(np.int32)
        ov = img.copy()
        cv2.polylines(
            ov,
            [np.stack([x_px, y.astype(np.int32)], 1).reshape(-1, 1, 2)],
            False,
            (0, 255, 0),
            2,
        )
        cv2.imwrite(os.path.join(dirpath, base + "_pred.jpg"), ov)
        Ys.append(h - y)

    Ys = np.asarray(Ys)
    t_src = np.arange(len(Ys), dtype=np.float32)
    t_dst = np.linspace(0, len(Ys) - 1, 500).astype(np.float32)
    Z = np.vstack([np.interp(t_dst, t_src, Ys[:, j]) for j in range(500)]).T
    X = np.tile(np.arange(500), (500, 1))
    Y = np.tile(np.arange(500).reshape(-1, 1), (1, 500))
    P = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()]).astype(np.float64)

    z = P[:, 2]
    z = (z - z.min()) / (z.max() - z.min() + 1e-8)
    pc_rgb = (
        cv2.applyColorMap((z * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET)[
            :, 0, :
        ][:, ::-1]
        / 255.0
    )

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(P)
    pcd.colors = o3d.utility.Vector3dVector(pc_rgb)

    draw = [
        o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])
    ]

    if interpolate_3d:
        idx = np.arange(500 * 500).reshape(500, 500)
        lines = np.column_stack([idx[:, :-1].ravel(), idx[:, 1:].ravel()]).astype(
            np.int32
        )
        zseg = ((Z[:, :-1] + Z[:, 1:]) / 2.0).ravel()
        zseg = (zseg - zseg.min()) / (zseg.max() - zseg.min() + 1e-8)
        ls_rgb = (
            cv2.applyColorMap(
                (zseg * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET
            )[:, 0, :][:, ::-1]
            / 255.0
        )
        ls = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(P),
            lines=o3d.utility.Vector2iVector(lines),
        )
        ls.colors = o3d.utility.Vector3dVector(ls_rgb)
        draw.append(ls)
    else:
        draw.append(pcd)

    obb = pcd.get_minimal_oriented_bounding_box()
    obb.color = (1, 0, 0)
    draw.append(obb)

    o3d.visualization.draw_geometries(draw)


if __name__ == "__main__":
    main("data/labeled/2025-07-29T21-28-34")
