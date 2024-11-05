import cv2
import numpy as np
import os
import shutil
import open3d as o3d


def segment(image):
    dpoints = []
    height, width = image.shape
    for x in range(width):
        intensity = image[:, x]

        is_black = intensity < 128
        black_regions = np.where(is_black)[0]

        if len(black_regions) > 0:
            # Find contiguous black regions
            diff = np.diff(black_regions)
            split_indices = np.where(diff > 1)[0] + 1
            segments = np.split(black_regions, split_indices)

            # Select the largest segment
            max_segment = max(segments, key=len)
            detected_y = int(max_segment.min())
            dpoints.append((x, detected_y))

    dpoints = np.array(dpoints)
    dpoints_x = dpoints[:, 0]
    dpoints_y = dpoints[:, 1].astype(np.float32)

    dpoints_y = cv2.GaussianBlur(dpoints_y, (0, 0), sigmaX=40)
    dpoints_y = np.squeeze(dpoints_y)

    for i in range(len(dpoints_x) - 1):
        pt1 = (int(dpoints_x[i]), int(dpoints_y[i]))
        pt2 = (int(dpoints_x[i + 1]), int(dpoints_y[i + 1]))
        cv2.line(image, pt1, pt2, (255, 0, 0), 2)

    ret_coord = np.column_stack([dpoints_x, dpoints_y])
    return image, ret_coord


def detect_lines(image_path, save_dir):
    img = cv2.imread(image_path)
    if img is None:
        print("Error: Could not load image.")
        return

    base_name = os.path.splitext(os.path.basename(image_path))[0]
    base_name = os.path.join(save_dir, base_name)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    cv2.imwrite(base_name + "_raw.jpg", img)

    alpha = 1.5  # Contrast control (1.0-3.0)
    beta = 100  # Brightness control (0-100)
    img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
    # cv2.imwrite(base_name + "_adjusted.jpg", img)

    # img = 255 - img
    # _, img = cv2.threshold(img, 180, 255, cv2.THRESH_BINARY)
    # cv2.imwrite(base_name + "_mask.jpg", img)

    img = cv2.Laplacian(img, cv2.CV_64F, ksize=3)
    img = cv2.convertScaleAbs(img)
    # cv2.imwrite(base_name + "_laplacian.jpg", img)

    img, ret_coords = segment(img)
    cv2.imwrite(base_name + "_detected.jpg", img)
    return ret_coords


def lines_3d(file_list, data_path, result_path, interval, acq_interval=False):
    pc_3d = []
    num_frames = interval
    increments = 499 / (num_frames - 1)  # Adjust increments to cover 0-499
    for i, file in enumerate(file_list):
        if os.path.isfile(os.path.join(data_path, file)):
            pc = detect_lines(os.path.join(data_path, file), result_path)
            idx = i % interval
            z_val = np.zeros(500) + idx * increments
            pc_z = np.column_stack([pc, z_val])
            pc_3d.append(pc_z)
            if acq_interval and len(pc_3d) == interval:
                break
    return np.array(pc_3d)


def interpolate_3d(data_frames, interval):
    total_frames = 500  # Fixed total number of frames
    num_frames = len(data_frames)
    num_x_indices = 500  # Assuming x ranges from 0 to 499

    # Initialize arrays to hold y values across frames
    y_values = np.zeros((num_frames, num_x_indices))

    # Collect z-values from key frames
    z_key_values = np.zeros(num_frames)

    # Populate y_values array and collect z-values
    for frame_idx, frame in enumerate(data_frames):
        y_values[frame_idx, :] = frame[:, 1]
        z_key_values[frame_idx] = frame[0, 2]  # All z-values in a frame are the same

    # Number of segments between frames
    num_segments = num_frames - 1

    # Total number of interpolated frames between key frames
    total_interpolated_frames = total_frames - num_frames

    # Number of interpolated frames per segment
    interpolated_frames_per_segment = total_interpolated_frames // num_segments

    # Remainder frames to distribute among segments
    remainder_frames = total_interpolated_frames % num_segments

    # List to hold all frames
    all_frames = []
    interpolated_frames = []
    idx_start = 0

    # Loop over each segment between frames
    for i in range(num_segments):
        # Determine number of interpolated frames for this segment
        frames_this_segment = interpolated_frames_per_segment
        if i < remainder_frames:
            frames_this_segment += 1

        # For each segment, generate interpolated frames
        for p in range(frames_this_segment):
            t = (p + 1) / (frames_this_segment + 1)
            y_interp = (1 - t) * y_values[i, :] + t * y_values[i + 1, :]
            x_indices = np.arange(num_x_indices)
            interpolated_frame = np.column_stack((x_indices, y_interp))
            interpolated_frames.append(interpolated_frame)

        # Assemble frames
        all_frames.append(data_frames[i][:, [0, 1]])  # Only x and y
        idx_end = idx_start + frames_this_segment
        all_frames.extend(interpolated_frames[idx_start:idx_end])
        idx_start = idx_end

    # Add the last original frame
    all_frames.append(data_frames[-1][:, [0, 1]])

    # Now, assign z-values to all frames to cover 0-499
    total_frames = len(all_frames)
    z_values = np.linspace(0, 499, total_frames)

    # Add z-values to each frame
    for i, frame in enumerate(all_frames):
        z_val = np.full((num_x_indices,), z_values[i])
        all_frames[i] = np.column_stack((frame, z_val))

    return np.array(all_frames)


from scipy.spatial.transform import Rotation as R


# Step 2: Convert normals to quaternions
def normal_to_quaternion(normal, reference=[0, 0, 1]):
    normal = normal / np.linalg.norm(normal)
    axis = np.cross(reference, normal)
    angle = np.arccos(np.clip(np.dot(reference, normal), -1.0, 1.0))
    if np.isclose(angle, 0):
        return np.array([0, 0, 0, 1])
    if np.isclose(angle, np.pi):
        orthogonal = (
            np.array([1, 0, 0])
            if not np.allclose(reference, [1, 0, 0])
            else np.array([0, 1, 0])
        )
        axis = np.cross(reference, orthogonal)
        axis = axis / np.linalg.norm(axis)
    quat = R.from_rotvec(axis * angle).as_quat()
    return quat


# Step 3: Average the quaternions
def average_quaternions(quaternions):
    quaternions = quaternions / np.linalg.norm(quaternions, axis=1)[:, np.newaxis]
    ref_quat = quaternions[0]
    for i in range(len(quaternions)):
        if np.dot(ref_quat, quaternions[i]) < 0:
            quaternions[i] = -quaternions[i]
    avg_quat = np.mean(quaternions, axis=0)
    avg_quat = avg_quat / np.linalg.norm(avg_quat)
    return avg_quat


def main(data_path, result_path, interval):
    shutil.rmtree(result_path)
    os.makedirs(result_path)
    file_list = sorted(os.listdir(data_path), key=lambda x: int(os.path.splitext(x)[0]))

    pc_lines = lines_3d(file_list, data_path, result_path, interval, False)
    pc_lines = pc_lines.reshape(-1, 3)
    pc_lines[:, [2, 1]] = pc_lines[:, [1, 2]]
    pcd_lines = o3d.geometry.PointCloud()
    pcd_lines.points = o3d.utility.Vector3dVector(pc_lines)
    ref_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=300, origin=[0, 0, 0]
    )
    boundbox = pcd_lines.get_minimal_oriented_bounding_box()
    boundbox.color = (1, 0, 0)
    rot_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=300, origin=np.mean(pc_lines, axis=0)
    )
    rot_coor = rot_coor.rotate(boundbox.R)
    o3d.visualization.draw_geometries(
        [pcd_lines, ref_coor, boundbox, rot_coor], point_show_normal=True
    )

    pc_3d = lines_3d(file_list, data_path, result_path, interval, True)
    pc_3d = interpolate_3d(pc_3d, interval)

    # pc_3d = pc_3d.reshape(-1, 3)
    pc_3d = np.vstack(pc_3d)
    pc_3d[:, [2, 1]] = pc_3d[:, [1, 2]]
    print(pc_3d)
    print(pc_3d.shape)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_3d)

    pcd = pcd.voxel_down_sample(voxel_size=2.0)
    print(f"Downsampled point cloud size: {len(pcd.points)}")
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    )
    # pcd.orient_normals_consistent_tangent_plane(100)
    print(np.asarray(pcd.normals))
    print(pcd.detect_planar_patches())
    planar = pcd.detect_planar_patches(
        normal_variance_threshold_deg=45,  # Try lower values
        coplanarity_deg=80,  # Try higher values
        outlier_ratio=0.6,  # Try higher values
        min_plane_edge_length=0.1,  # Try lower values
        min_num_points=100,  # Adjust based on your point cloud density
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=50),
    )
    print(planar)
    boundbox = pcd.get_minimal_oriented_bounding_box()
    # boundbox = (pcd.get_oriented_bounding_box())
    boundbox.color = (1, 0, 0)

    pc_3d_mean = np.mean(pc_3d, axis=0)
    print(pc_3d_mean)
    ref_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=300, origin=[0, 0, 0]
    )
    rot_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=300, origin=pc_3d_mean
    )
    robot_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=300, origin=pc_3d_mean
    )
    rot_coor = rot_coor.rotate(boundbox.R)
    o3d.visualization.draw_geometries(
        [pcd, ref_coor, boundbox, rot_coor], point_show_normal=True
    )

    # normals = pcd.normals
    # quaternions = np.array([normal_to_quaternion(n) for n in normals])
    # avg_quat = average_quaternions(quaternions)
    # # Step 4: Use the average quaternion
    # rotation = R.from_quat(avg_quat)
    # rotation_matrix = rotation.as_matrix()
    # euler_angles = rotation.as_euler("xyz", degrees=True)
    # print("Average Quaternion:", avg_quat)
    # print("Rotation Matrix:\n", rotation_matrix)
    # print("Euler Angles (degrees):", euler_angles)

    hull, _ = pcd.compute_convex_hull()
    hull.compute_vertex_normals()
    hull.compute_triangle_normals()
    print(np.asarray(hull.triangle_normals))

    triangles = np.asarray(hull.triangles)
    normals = np.asarray(hull.triangle_normals)
    vertices = np.asarray(hull.vertices)
    # Compute triangle centers
    centers = np.mean(vertices[triangles], axis=1)
    # Create a point cloud for the triangle centers
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(centers)
    pcd.normals = o3d.utility.Vector3dVector(normals)

    # Create a coordinate frame for reference
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.5, origin=[0, 0, 0]
    )

    # Visualize the hull, normal points, and coordinate frame
    o3d.visualization.draw_geometries(
        [hull, pcd, coord_frame], point_show_normal=True, mesh_show_wireframe=True
    )
    # o3d.visualization.draw_geometries([hull], point_show_normal=True)


# path = "data/skin"
# path = "data/skin_oct_motion"
# path = "data/skin_octa_static"

# path = "data/3d_not_aligned"
# path = "data/3d_aligned"
path = "data/skin_oct"
# path = "data/skin_octa"

save_path = "data/result"
interval = 4
main(path, save_path, interval)
