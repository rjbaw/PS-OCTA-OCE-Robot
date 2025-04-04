import enum
import cv2
import numpy as np
import os
import shutil
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt


def draw_line(image, ret_coord):
    for i in range(len(ret_coord) - 1):
        pt1 = (int(ret_coord[i, 0]), int(ret_coord[i, 1]))
        pt2 = (int(ret_coord[i + 1, 0]), int(ret_coord[i + 1, 1]))
        cv2.line(image, pt1, pt2, (255, 0, 0), 2)
    return image


def get_max_coor(img):
    ret_coords = []
    height, width = img.shape
    for x in range(width):
        intensity = img[:, x]
        detected_y = np.argmax(intensity)
        ret_coords.append((x, detected_y))
    return np.array(ret_coords)


def detect_lines_savgol(image_path, save_dir):
    from scipy.ndimage import median_filter, gaussian_filter, uniform_filter1d
    from scipy.signal import savgol_filter

    img_raw = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img_raw is None:
        print("Error: Could not load image.")
        return

    base_name = os.path.splitext(os.path.basename(image_path))[0]
    base_name = os.path.join(save_dir, base_name)
    # img = cv2.cvtColor(img_raw, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(base_name + "_raw.jpg", img_raw)
    img = cv2.medianBlur(img_raw, 5)
    # img = img_raw.astype(np.float32)
    img = img.copy() - np.mean(img, axis=1, keepdims=True)

    cv2.imwrite(base_name + "_sub.jpg", img)

    img = median_filter(img, size=(3, 11))
    img = gaussian_filter(img, sigma=3)

    # np.gradient returns (dIMG/drow, dIMG/dcol) for a 2D array
    gy, gx = np.gradient(img)
    surface = np.argmax(gy, axis=0)

    cv2.imwrite(
        base_name + "_max.jpg",
        draw_line(img_raw.copy(), np.column_stack([np.arange(500), surface])),
    )

    observations = surface

    x_k = np.median(observations[:50])
    P_k = 1
    Q = 0.01
    R = 0.5
    x_k_estimates = []
    P_k_values = []
    for z_k in observations:
        x_k_pred = x_k
        P_k_pred = P_k + Q
        K_k = P_k_pred / (P_k_pred + R)
        x_k = x_k_pred + K_k * (z_k - x_k_pred)
        P_k = (1 - K_k) * P_k_pred
        x_k_estimates.append(x_k)
        P_k_values.append(P_k)
    surface[:] = x_k_estimates

    obs_length = len(observations)
    window = max(1, int(obs_length / 10))
    old_val = 0
    for i, pt in enumerate(observations):
        start = max(0, i - window)
        end = min(obs_length, i + window + 1)
        mu = np.mean(observations[start:end])
        sigma = np.std(observations[start:end])
        med = np.median(observations[start:end])
        if sigma != 0:
            z = (pt - mu) / sigma
        else:
            z = pt - mu
        if z > 0.5:
            if ((med - mu) / sigma) > 3:
                observations[i] = mu
            else:
                observations[i] = med
            # observations[i] = med
    surface[:] = observations

    surface_smooth = savgol_filter(surface + 1, window_length=15, polyorder=3)
    ret_coords = surface_smooth
    ret_coords = np.column_stack([np.arange(500), ret_coords])

    cv2.imwrite(
        base_name + "_detected.jpg",
        draw_line(img_raw.copy(), ret_coords),
    )

    return ret_coords


def zero_dc(img, zidx, window):
    img = img.astype(np.float32)
    for i in zidx:
        start_idx = max(i - window // 2, 0)
        end_idx = min(i + window // 2, 512)
        filter_window = img[start_idx:end_idx, :]
        mean_col = np.mean(filter_window, axis=1, keepdims=True)
        img[start_idx:end_idx, :] -= mean_col

    img = np.clip(img, 0, 255).astype(np.uint8)

    return img


def detect_lines_old(image_path, save_dir):
    img_raw = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    height, width = img_raw.shape
    if img_raw is None:
        print("Error: Could not load image.")
        return

    base_name = os.path.splitext(os.path.basename(image_path))[0]
    base_name = os.path.join(save_dir, base_name)
    cv2.imwrite(base_name + "_raw.jpg", img_raw)
    img = img_raw

    # img = cv2.line(img, (0, 140), (499, 140), (255, 0, 0), 5)

    img = cv2.Sobel(img, cv2.CV_8U, 0, 1, ksize=5)
    cv2.imwrite(base_name + "_sobel.jpg", img)

    img = cv2.medianBlur(img, 11)
    img = cv2.GaussianBlur(img, (11, 11), 0)

    mean_col = np.mean(img, axis=1, keepdims=True)

    # ret_coords[:, 1] = medfilt(ret_coords[:, 1], kernel_size=15)

    from scipy.signal import find_peaks

    plt.figure()
    plt.plot(mean_col[:150])
    peaks, _ = find_peaks(mean_col[:150].squeeze())
    plt.plot(peaks, mean_col[:150][peaks], "ro", label="Peaks")
    # img = zero_dc(img, peaks, 12)
    img = zero_dc(img, (0, 5, 12, 24, 37, 51, 63, 75, 87, 99, 112, 126), 14)
    mean_col = np.mean(img, axis=1, keepdims=True)

    plt.plot(mean_col[:150])
    plt.savefig(base_name + "_plot.jpg")
    plt.close()
    # plt.show()

    cv2.imwrite(base_name + "_sub.jpg", img)

    ret_coords = get_max_coor(img)
    cv2.imwrite(
        base_name + "_max.jpg",
        draw_line(img_raw.copy(), ret_coords),
    )

    observations = ret_coords[:, 1]

    obs_length = len(observations)
    window = max(1, int(obs_length / 20))
    old_val = 0
    for i, pt in enumerate(observations):
        start = max(0, i - window)
        end = min(obs_length, i + window + 1)
        mu = np.mean(observations[start:end])
        sigma = np.std(observations[start:end])
        med = np.median(observations[start:end])
        if sigma != 0:
            z = abs(pt - mu) / sigma
        else:
            z = abs(pt - mu)
        if z > 0.5:
            observations[i] = med

    ret_coords[:, 1] = observations
    cv2.imwrite(
        base_name + "_outlier.jpg",
        draw_line(img_raw.copy(), ret_coords),
    )

    obs_length = len(observations)
    window = max(1, int(obs_length / 5))
    z_max = 18.0
    for i in range(obs_length):
        start = max(0, i - window)
        end = min(obs_length, i - 1)
        mu = np.mean(observations[start:end])
        sigma = np.std(observations[start:end])
        med = np.median(observations[start:end])
        if sigma != 0:
            z = abs(observations[i] - mu) / sigma
        else:
            z = abs(observations[i] - mu)
        if z > z_max:
            print(z)
            found = False
            for j in range(window):
                idx = i + j
                if idx >= obs_length:
                    break
                if sigma != 0:
                    z = abs(observations[idx] - mu) / sigma
                else:
                    z = abs(observations[idx] - mu)
                if z < z_max:
                    found = True
                    prev_idx = max(0, i - 1)
                    dy = (observations[idx] - observations[prev_idx]) / (j + 1)
                    for k in range(j + 1):
                        if (i + k) < obs_length:
                            observations[i + k] = observations[prev_idx] + k * dy
                    # observations[i] = med
                    break

            end = min(obs_length, i - 1)
            if not found:
                observations[i : (i + window + 1)] = observations[i - 1]

    ret_coords[:, 1] = observations
    cv2.imwrite(
        base_name + "_linear.jpg",
        draw_line(img_raw.copy(), ret_coords),
    )

    signal = observations
    window_size = 21
    half_win = window_size // 2
    padded_signal = np.pad(signal, (half_win, half_win), mode="edge")
    filtered_signal = np.zeros_like(signal)
    for i in range(len(signal)):
        window = padded_signal[i : i + window_size]
        observations[i] = np.median(window)

    # observations = ret_coords[:, 1][::-1]
    x_k = np.median(observations[:10])
    P_k = 1
    Q = 0.01
    R = 0.5
    x_k_estimates = []
    P_k_values = []
    for z_k in observations:
        x_k_pred = x_k
        P_k_pred = P_k + Q
        K_k = P_k_pred / (P_k_pred + R)
        x_k = x_k_pred + K_k * (z_k - x_k_pred)
        P_k = (1 - K_k) * P_k_pred
        x_k_estimates.append(x_k)
        P_k_values.append(P_k)
    ret_coords[:, 1] = x_k_estimates

    cv2.imwrite(
        base_name + "_detected_pre.jpg",
        draw_line(img_raw.copy(), ret_coords),
    )

    ret_coords[:, 1] = height - ret_coords[:, 1]

    return ret_coords


def spatial_filter(base_name, img):
    def wiener_filter(dft_complex):
        dft_shifted = np.fft.fftshift(
            dft_complex, axes=[0, 1]
        )  # shift zero-freq to center
        real_part, imag_part = cv2.split(dft_shifted)
        magnitude = cv2.magnitude(real_part, imag_part)

        S_xx = (cv2.medianBlur(magnitude.astype(np.float32), 3)) ** 2
        background = np.sqrt(S_xx)
        threshold_factor = 2.0
        noise_mask = magnitude > (threshold_factor * np.sqrt(background))
        noise_power = np.zeros_like(magnitude)
        noise_power[noise_mask] = magnitude[noise_mask] ** 2
        S_nn = noise_power

        eps = 1e-8
        H = S_xx / (S_xx + S_nn + eps)

        filtered_real = real_part * H
        filtered_imag = imag_part * H

        dft_filtered_shifted = cv2.merge([filtered_real, filtered_imag])
        dft_filtered = np.fft.ifftshift(dft_filtered_shifted, axes=[0, 1])

        return dft_filtered

    img_float = np.float32(img)
    dft_complex = cv2.dft(img_float, flags=cv2.DFT_COMPLEX_OUTPUT)
    dft_shifted = np.fft.fftshift(dft_complex, axes=[0, 1])

    real_s, imag_s = cv2.split(dft_shifted)
    mag_s = cv2.magnitude(real_s, imag_s)
    title = "log_spectrum"
    mag_log = np.log1p(mag_s)  # log(1 + M)
    mag_norm = cv2.normalize(mag_log, None, 0, 255, cv2.NORM_MINMAX)
    mag_uint8 = np.uint8(np.around(mag_norm))
    plt.figure()
    plt.plot(np.mean(img, axis=1))
    plt.savefig(base_name + "_" + title + "_2d_x.jpg")
    plt.close()

    plt.figure()
    plt.plot(np.mean(img, axis=0))
    plt.savefig(base_name + "_" + title + "_2d_y.jpg")
    cv2.imwrite(base_name + "_" + title + ".jpg", mag_uint8)
    plt.close()

    dft_filtered = wiener_filter(dft_complex)

    dft_filtered = np.fft.fftshift(dft_filtered, axes=[0, 1])
    real_f, imag_f = cv2.split(dft_filtered)
    mag_f = cv2.magnitude(real_f, imag_f)
    title = "shifted_filtered_spectrum"
    mag_log = np.log1p(mag_f)  # log(1 + M)
    mag_norm = cv2.normalize(mag_log, None, 0, 255, cv2.NORM_MINMAX)
    mag_uint8 = np.uint8(np.around(mag_norm))
    plt.figure()
    plt.plot(np.mean(img, axis=1))
    plt.savefig(base_name + "_" + title + "_2d_x.jpg")
    plt.close()

    plt.figure()
    plt.plot(np.mean(img, axis=0))
    plt.savefig(base_name + "_" + title + "_2d_y.jpg")
    cv2.imwrite(base_name + "_" + title + ".jpg", mag_uint8)
    plt.close()

    img_back = cv2.idft(dft_filtered, flags=cv2.DFT_REAL_OUTPUT | cv2.DFT_SCALE)
    img_back_norm = cv2.normalize(img_back, None, 0, 255, cv2.NORM_MINMAX)
    img_back_uint8 = np.uint8(np.around(img_back_norm))

    return img_back_uint8


def detect_lines(image_path, save_dir):
    img_raw = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    height, width = img_raw.shape
    if img_raw is None:
        print("Error: Could not load image.")
        return

    base_name = os.path.splitext(os.path.basename(image_path))[0]
    base_name = os.path.join(save_dir, base_name)
    cv2.imwrite(base_name + "_raw.jpg", img_raw)
    img = img_raw

    img = spatial_filter(base_name, img)
    cv2.imwrite(base_name + "_spatial.jpg", img)

    ret_coords = get_max_coor(img)
    cv2.imwrite(
        base_name + "_max.jpg",
        draw_line(img_raw.copy(), ret_coords),
    )

    observations = ret_coords[:, 1]

    obs_length = len(observations)
    window = max(1, int(obs_length / 50))
    old_val = 0
    for i, pt in enumerate(observations):
        start = max(0, i - window)
        end = min(obs_length, i + window + 1)
        if len(observations[start:end]) > 0:
            mu = np.mean(observations[start:end])
            sigma = np.std(observations[start:end])
            med = np.median(observations[start:end])
            if sigma != 0:
                z = abs(pt - mu) / sigma
            else:
                z = abs(pt - mu)
            if z > 0.5:
                observations[i] = med

    ret_coords[:, 1] = observations
    cv2.imwrite(
        base_name + "_outlier.jpg",
        draw_line(img_raw.copy(), ret_coords),
    )

    obs_length = len(observations)
    window = max(1, int(obs_length / 5))
    z_max = 18.0
    for i in range(obs_length):
        start = max(0, i - window)
        end = min(obs_length, i - 1)
        if len(observations[start:end]) > 0:
            mu = np.mean(observations[start:end])
            sigma = np.std(observations[start:end])
            med = np.median(observations[start:end])
            if sigma != 0:
                z = abs(observations[i] - mu) / sigma
            else:
                z = abs(observations[i] - mu)
            if z > z_max:
                print(z)
                found = False
                for j in range(window):
                    idx = i + j
                    if idx >= obs_length:
                        break
                    if sigma != 0:
                        z = abs(observations[idx] - mu) / sigma
                    else:
                        z = abs(observations[idx] - mu)
                    if z < z_max:
                        found = True
                        prev_idx = max(0, i - 1)
                        dy = (observations[idx] - observations[prev_idx]) / (j + 1)
                        for k in range(j + 1):
                            if (i + k) < obs_length:
                                observations[i + k] = observations[prev_idx] + k * dy
                        # observations[i] = med
                        break

                end = min(obs_length, i - 1)
                if not found:
                    observations[i : (i + window + 1)] = observations[i - 1]
    ret_coords[:, 1] = observations
    cv2.imwrite(
        base_name + "_linear.jpg",
        draw_line(img_raw.copy(), ret_coords),
    )

    signal = observations
    window_size = 21
    half_win = window_size // 2
    padded_signal = np.pad(signal, (half_win, half_win), mode="edge")
    filtered_signal = np.zeros_like(signal)
    for i in range(len(signal)):
        window = padded_signal[i : i + window_size]
        observations[i] = np.median(window)

    cv2.imwrite(
        base_name + "_median.jpg",
        draw_line(img_raw.copy(), ret_coords),
    )

    # observations = ret_coords[:, 1][::-1]
    x_k = np.median(observations[:10])
    P_k = 1
    Q = 0.01
    R = 0.5
    x_k_estimates = []
    P_k_values = []
    for z_k in observations:
        x_k_pred = x_k
        P_k_pred = P_k + Q
        K_k = P_k_pred / (P_k_pred + R)
        x_k = x_k_pred + K_k * (z_k - x_k_pred)
        P_k = (1 - K_k) * P_k_pred
        x_k_estimates.append(x_k)
        P_k_values.append(P_k)
    ret_coords[:, 1] = x_k_estimates

    cv2.imwrite(
        base_name + "_detected_pre.jpg",
        draw_line(img_raw.copy(), ret_coords),
    )

    ret_coords[:, 1] = height - ret_coords[:, 1]

    return ret_coords


def segment_white(image):
    dpoints = []
    height, width = image.shape
    for x in range(width):
        intensity = image[:, x]

        is_white = intensity > 20
        white_regions = np.where(is_white)[0]

        diff = np.diff(white_regions)
        split_indices = np.where(diff > 1)[0] + 1
        segments = np.split(white_regions, split_indices)
        max_segment = max(segments, key=len)
        detected_y = int(max_segment.min())
        dpoints.append((x, detected_y))

        # if len(white_regions) > 0:
        #     diff = np.diff(white_regions)
        #     split_indices = np.where(diff > 1)[0] + 1
        #     segments = np.split(white_regions, split_indices)
        #     max_segment = max(segments, key=len)
        #     detected_y = int(max_segment.min())
        #     dpoints.append((x, detected_y))

    dpoints = np.array(dpoints)
    dpoints_x = dpoints[:, 0]
    dpoints_y = dpoints[:, 1].astype(np.float32)

    dpoints_y = cv2.GaussianBlur(dpoints_y, (0, 0), sigmaX=40)
    dpoints_y = np.squeeze(dpoints_y)

    ret_coord = np.column_stack([dpoints_x, dpoints_y])
    return ret_coord


def lines_3d(
    file_list, data_path, result_path, interval, detect_func, acq_interval=False
):
    pc_3d = []
    num_frames = interval
    increments = 499 / (num_frames - 1)  # Adjust increments to cover 0-499
    for i, file in enumerate(file_list):
        if os.path.isfile(os.path.join(data_path, file)):
            pc = detect_func(os.path.join(data_path, file), result_path)
            idx = i % interval
            z_val = np.zeros(500) + idx * increments
            pc_z = np.column_stack([pc, z_val])
            pc_z[:, [2, 1]] = pc_z[:, [1, 2]]
            pc_3d.append(pc_z)
            if acq_interval and len(pc_3d) == interval:
                break
    pc_3d = np.array(pc_3d)
    return pc_3d


def interpolate_3d(data_frames, interval):
    total_frames = 500
    num_frames = len(data_frames)
    num_x_indices = 500
    z_values = np.zeros((num_frames, num_x_indices))

    y_key_values = np.zeros(num_frames)

    for frame_idx, frame in enumerate(data_frames):
        z_values[frame_idx, :] = frame[:, 2]
        y_key_values[frame_idx] = frame[0, 1]

    num_segments = num_frames - 1

    total_interpolated_frames = total_frames - num_frames

    interpolated_frames_per_segment = total_interpolated_frames // num_segments

    remainder_frames = total_interpolated_frames % num_segments

    all_frames = []
    interpolated_frames = []
    idx_start = 0

    for i in range(num_segments):

        frames_this_segment = interpolated_frames_per_segment
        if i < remainder_frames:
            frames_this_segment += 1

        for p in range(frames_this_segment):
            t = (p + 1) / (frames_this_segment + 1)
            z_interp = (1 - t) * z_values[i, :] + t * z_values[i + 1, :]
            x_indices = np.arange(num_x_indices)
            interpolated_frame = np.column_stack((x_indices, z_interp))
            interpolated_frames.append(interpolated_frame)

        all_frames.append(data_frames[i][:, [0, 2]])
        idx_end = idx_start + frames_this_segment
        all_frames.extend(interpolated_frames[idx_start:idx_end])
        idx_start = idx_end

    all_frames.append(data_frames[-1][:, [0, 2]])

    total_frames = len(all_frames)
    y_values = np.linspace(0, 499, total_frames)

    for i, frame in enumerate(all_frames):
        y_val = np.full((num_x_indices,), y_values[i])
        all_frames[i] = np.column_stack((frame[:, 0], y_val, frame[:, 1]))

    return np.array(all_frames)


def align_to_direction(rot_matrix):
    out_matrix = np.zeros((3, 3))
    for col in range(3):
        max_idx = np.argmax(np.abs(rot_matrix[:, col]))
        if col != max_idx:
            out_matrix[:, max_idx] = rot_matrix[:, col]
        else:
            out_matrix[:, col] = rot_matrix[:, col]
    for col in range(3):
        if out_matrix[col, col] < 0:
            out_matrix[:, col] *= -1
    return out_matrix


def test_basic(file_list, data_path, result_path, interval):
    print("\n******************** TEST simple ********************\n")
    pc_lines = lines_3d(
        file_list, data_path, result_path, interval, detect_lines, acq_interval=False
    )
    pc_lines = np.vstack(pc_lines)
    print(pc_lines)

    pcd_lines = o3d.geometry.PointCloud()
    pcd_lines.points = o3d.utility.Vector3dVector(pc_lines)
    ref_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=300, origin=[0, 0, 0]
    )
    boundbox = pcd_lines.get_minimal_oriented_bounding_box(robust=True)
    # boundbox = pcd_lines.get_oriented_bounding_box()
    boundbox.color = (1, 0, 0)
    # box_pt = np.asarray(boundbox.get_box_points())
    rot_mat = align_to_direction(boundbox.R)
    rpy = R.from_matrix(np.array(rot_mat)).as_euler("xyz", degrees=True)
    print("rotation from bounding box rpy: ", rpy)
    print("rotation from bounding box: \n", rot_mat)

    final_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=300, origin=np.mean(pc_lines, axis=0)
    )
    final_coor = final_coor.rotate(rot_mat)

    o3d.visualization.draw_geometries(
        [pcd_lines, ref_coor, boundbox, final_coor], point_show_normal=False
    )


def test_3d(file_list, data_path, result_path, interval):
    print("\n******************** TEST 3D interpolation ********************\n")
    pc_3d = lines_3d(
        file_list, data_path, result_path, interval, detect_lines, acq_interval=True
    )
    pc_3d = interpolate_3d(pc_3d, interval)
    # print(pc_3d)
    # print(pc_3d.shape)

    pc_3d = np.vstack(pc_3d)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_3d)

    pcd = pcd.voxel_down_sample(voxel_size=2.0)
    print(f"Downsampled point cloud size: {len(pcd.points)}")
    # pcd.estimate_normals(
    #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    # )
    # pcd.orient_normals_consistent_tangent_plane(100)

    boundbox = pcd.get_minimal_oriented_bounding_box()
    # boundbox = pcd.get_oriented_bounding_box()
    # boundbox = pcd.get_axis_aligned_bounding_box()
    # box_pt = np.asarray(boundbox.get_box_points())
    boundbox.color = (1, 0, 0)

    pc_3d_mean = np.mean(pc_3d, axis=0)
    print("mean centre point:", pc_3d_mean)

    ref_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=300, origin=[0, 0, 0]
    )
    robot_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=300, origin=[249, 249, 499]
    )
    robot_frame = robot_coor.get_rotation_matrix_from_xyz(
        (np.radians(180), np.radians(0), np.radians(-90))
    )
    robot_coor.rotate(robot_frame)

    final_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=300, origin=pc_3d_mean
    )
    final_coor = final_coor.rotate(robot_frame)

    # rpy = R.from_matrix(np.array(boundbox.R)).as_euler("xyz", degrees=True)
    # print("rotation from bounding box rpy: ", rpy)
    # print("rotation from bounding box: \n", boundbox.R)

    rot_mat = align_to_direction(boundbox.R)
    final_coor = final_coor.rotate(rot_mat)

    rpy = R.from_matrix(np.array(rot_mat)).as_euler("xyz", degrees=True)
    print("rotation from bounding box rpy: ", rpy)
    print("rotation from bounding box: \n", rot_mat)

    o3d.visualization.draw_geometries(
        [pcd, ref_coor, boundbox, final_coor, robot_coor], point_show_normal=False
    )


def main(data_path, result_path, interval):
    start_idx = 0
    shutil.rmtree(result_path)
    os.makedirs(result_path)
    file_list = sorted(os.listdir(data_path), key=lambda x: int(os.path.splitext(x)[0]))
    file_list = file_list[(interval * start_idx) :]

    test_basic(file_list, data_path, result_path, interval)
    # test_3d(file_list, data_path, result_path, interval)
    # test_ml(file_list, data_path, result_path, interval)


if __name__ == "__main__":
    # path = "data/skin"
    # path = "data/skin_oct_motion"
    # path = "data/skin_octa_static"

    # path = "data/3d_not_aligned"
    # path = "data/3d_aligned"
    # path = "data/skin_oct"
    # path = "data/skin_octa"
    path = "data/no_background"
    # path = "data/current"

    path = "data/real"
    # path = "data/current"
    # path = "data/real2"

    save_path = "data/result"
    interval = 6
    main(path, save_path, interval)
