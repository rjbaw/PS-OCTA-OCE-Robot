import enum
import cv2
import numpy as np
import os
import shutil
import open3d as o3d
from scipy.ndimage import _support_alternative_backends
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


def zero_dc_refined(img):
    # (0, 5, 12, 24, 37, 51, 63, 75, 87, 99, 112, 126), 14
    # zidx = [(0, 5), (19, 40), (26, 46), (50, 67), (70, 91), (90, 130)]
    zidx = [(0, 200)]
    img = img.astype(np.float32)
    for i in range(len(zidx)):
        start_idx = zidx[i][0]
        end_idx = zidx[i][1]
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
    plt.show()
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
            print("z =", z, "> zmax =", z_max)
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


def get_psd():
    n = 8
    power_spectrum = np.zeros((512, 500))
    for i in range(n):
        img = cv2.imread(
            "data/background/current/" + str(i + 1) + ".jpg",
            flags=cv2.IMREAD_GRAYSCALE,
        )
        img = img.astype(float)
        dft = np.fft.fft2(img)
        dft = np.fft.fftshift(dft)
        power_spectrum += np.abs(dft) ** 2
    power_spectrum /= n
    return power_spectrum


def load_bg():
    n = 8
    img_avg = None
    for i in range(n):
        img = cv2.imread(
            "data/background/current/" + str(i + 1) + ".jpg",
            flags=cv2.IMREAD_GRAYSCALE,
        )
        if i == 0:
            img_avg = img.astype(float)
        img = img.astype(float)
        img_avg = (img + img_avg) / 2.0
    return img_avg


def med_bg_filter(raw):
    row_median = np.median(raw, axis=1, keepdims=True)
    tmp = raw - row_median
    col_median = np.median(tmp, axis=0, keepdims=True)
    clean = tmp - col_median
    clean -= clean.min()
    clean /= clean.max()
    clean = (clean * 255).astype(np.uint8)
    return clean


def reg_bg_filter(raw):
    bg = load_bg().astype(float)
    raw = raw.astype(float)
    h, w = raw.shape
    clean = np.zeros_like(raw)
    for i in range(h):
        r = raw[i]
        b = bg[i]
        s = (r * b).sum() / (b * b).sum()
        clean[i] = r - s * b
    clean -= clean.min()
    clean /= clean.max()
    clean_u8 = (clean * 255).astype(np.uint8)
    return clean_u8


def bg_filter(raw):
    bg = load_bg()
    sub = raw.astype(np.int32) - bg.astype(np.int32)
    sub -= sub.min()
    sub = (sub / sub.max() * 255).astype(np.uint8)

    # clahe = cv2.createCLAHE(
    #     clipLimit=2.0,
    #     tileGridSize=(8, 8),
    # )
    # sub_clahe = clahe.apply(sub)
    # return sub_clahe
    return sub


def clahe_bg_filter(raw):
    median_k = 51  # odd kernel for medianBlur
    clahe_clip = 2.0  # CLAHE parameters
    clahe_grid = (8, 8)

    raw32 = raw.astype(np.float32)
    bg32 = load_bg().astype(np.float32)
    ratio = raw32 / (bg32 + 1e-8)

    ratio_norm = cv2.normalize(ratio, None, 0, 255, cv2.NORM_MINMAX)
    ratio_u8 = ratio_norm.astype(np.uint8)

    # high‑pass
    if median_k % 2 == 0:
        median_k += 1
    low_pass = cv2.medianBlur(ratio_u8, median_k)
    high_pass = cv2.subtract(ratio_u8, low_pass)

    clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=clahe_grid)
    out = clahe.apply(high_pass)

    return out


def spatial_bg_filter(img_float):
    # dft_complex = np.fft.fft2(img_float)
    # dft_shifted = np.fft.fftshift(dft_complex, axes=[0, 1])
    # real_part = np.real(dft_shifted)
    # imag_part = np.imag(dft_shifted)
    # S_bg = get_psd()
    # H = 1 / (np.sqrt(S_bg) + 1e-8)
    # filtered_real = real_part * H
    # filtered_imag = imag_part * H
    # dft_filtered_shifted = cv2.merge([filtered_real, filtered_imag])
    # dft_filtered = np.fft.ifftshift(dft_filtered_shifted, axes=[0, 1])
    # img_back = cv2.idft(dft_filtered, flags=cv2.DFT_REAL_OUTPUT | cv2.DFT_SCALE)
    # img_back_norm = cv2.normalize(img_back, None, 0, 255, cv2.NORM_MINMAX)
    # img_back_uint8 = np.uint8(np.around(img_back_norm))

    dft_shifted = np.fft.fftshift(np.fft.fft2(img_float))
    bg = load_bg()
    S_bg = np.fft.fftshift(np.fft.fft2(bg))
    S_filtered = dft_shifted * 1 / (np.abs(S_bg))
    filtered = np.fft.ifft2(np.fft.ifftshift(S_filtered))
    filtered = np.abs(filtered)
    # filtered = np.real(filtered)

    # filtered = cv2.bilateralFilter(
    #     filtered.astype(np.float32), d=5, sigmaColor=15, sigmaSpace=3
    # )
    # img_back_norm = cv2.normalize(filtered, None, 0, 255, cv2.NORM_MINMAX)
    filtered -= filtered.min()
    filtered /= filtered.max()
    img_back = np.uint8(np.around(filtered * 255))
    img_back_uint8 = img_back

    return img_back_uint8


def spatial_filter(base_name, img):
    def wiener_filter_dc(img_float):
        dc_section = img_float[:130, :]
        dft_complex = np.fft.fft2(dc_section)
        dft_shifted = np.fft.fftshift(dft_complex, axes=[0, 1])
        real_part = np.real(dft_shifted)
        imag_part = np.imag(dft_shifted)
        magnitude = np.abs(dft_shifted)

        S_xx = (cv2.medianBlur(magnitude.astype(np.float32), 3)) ** 2
        S_nn = get_psd()[:130, :]

        H = S_xx / (S_xx + S_nn + 1e-8)

        filtered_real = real_part * H
        filtered_imag = imag_part * H

        dft_filtered_shifted = cv2.merge([filtered_real, filtered_imag])
        dft_filtered = np.fft.ifftshift(dft_filtered_shifted, axes=[0, 1])

        img_back = cv2.idft(dft_filtered, flags=cv2.DFT_REAL_OUTPUT | cv2.DFT_SCALE)
        img_back_norm = cv2.normalize(img_back, None, 0, 255, cv2.NORM_MINMAX)
        img_back = np.uint8(np.around(img_back_norm))
        img_float[:130, :] = img_back

        return img_float

    def wiener_filter(img_float):
        # img_float = wiener_filter_dc(img_float)
        img_float = zero_dc_refined(img_float)
        # img_float = bg_filter(img_float)
        # img_float = zero_dc(
        #     img_float, (0, 5, 12, 24, 37, 51, 63, 75, 87, 99, 112, 126), 14
        # )
        # dft_complex = cv2.dft(img_float, flags=cv2.DFT_COMPLEX_OUTPUT)
        dft_complex = np.fft.fft2(img_float)
        dft_shifted = np.fft.fftshift(dft_complex, axes=[0, 1])
        real_part = np.real(dft_shifted)
        imag_part = np.imag(dft_shifted)
        magnitude = np.abs(dft_shifted)

        S_nn = get_psd()

        # f = np.fft.fft2(img_float)
        # fshift = np.fft.fftshift(f)
        # fshift_filtered = np.abs(fshift**2 / S_nn)
        # S_xx = fshift_filtered

        # S_xx = (cv2.medianBlur(magnitude.astype(np.float32), 3)) ** 2
        # S_xx = np.max((magnitude.astype(np.float32) ** 2) - S_nn, 0)
        S_xx = np.max((magnitude.astype(np.float32) ** 2), 0)

        # background = np.sqrt(S_xx)
        # noise_mask = magnitude > (1.0 * np.sqrt(background))
        # noise_power = np.zeros_like(magnitude)
        # noise_power[noise_mask] = magnitude[noise_mask] ** 2
        # S_nn = noise_power

        H = S_xx / (S_xx + S_nn + 1e-8)

        filtered_real = real_part * H
        filtered_imag = imag_part * H
        dft_filtered_shifted = cv2.merge([filtered_real, filtered_imag])
        dft_filtered = np.fft.ifftshift(dft_filtered_shifted, axes=[0, 1])

        return dft_filtered

    img_float = np.float32(img)
    dft_complex = np.fft.fft2(img_float)
    dft_shifted = np.fft.fftshift(dft_complex, axes=[0, 1])

    mag_s = np.abs(dft_shifted)
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

    dft_filtered = wiener_filter(img_float)

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


def linearization(ret_coords):
    observations = ret_coords[:, 1]
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
                print("z =", z, "> zmax =", z_max)
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
    return ret_coords


def outlier_removal(ret_coords):
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

    return ret_coords


def kalman_filter(ret_coords):
    # observations = ret_coords[:, 1][::-1]
    observations = ret_coords[:, 1]
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
    return ret_coords


def median_filter(ret_coords):
    observations = ret_coords[:, 1]
    signal = observations
    window_size = 21
    half_win = window_size // 2
    padded_signal = np.pad(signal, (half_win, half_win), mode="edge")
    for i in range(len(signal)):
        window = padded_signal[i : i + window_size]
        observations[i] = np.median(window)
    ret_coords[:, 1] = observations
    return ret_coords


def med_filter(observations, window_size):
    signal = observations
    window_size = max(1, window_size)
    half_win = window_size // 2
    padded_signal = np.pad(signal, (half_win, half_win), mode="edge")
    for i in range(len(signal)):
        window = padded_signal[i : i + window_size]
        observations[i] = np.median(window)
    return observations


def ol_removal(ret_coords):
    observations = ret_coords[:, 1]
    obs_length = len(observations)
    window = max(1, obs_length // 3)
    z_max = 40.0
    m = 0
    sigma_seg = np.inf
    for w in range(obs_length // 3):
        start = max(0, w * window)
        end = min((w + 1) * window, obs_length - 1)
        segment = np.copy(observations[start:end])
        if len(segment) > 0:
            sigma = np.std(segment)
            segment_f = med_filter(segment, window)
            if sigma < sigma_seg:
                sigma_seg = sigma
                start_seg = segment_f[0]
                end_seg = segment_f[-1]
                m_new = (end_seg - start_seg) / len(segment_f)
                if abs(m_new) < z_max:
                    m = m_new

    for i, pt in enumerate(observations):
        if i > 0:
            # mu = np.mean(observations[start:end])
            # sigma = np.std(observations[start:end])
            # med = np.median(observations[start:end])
            # z = abs(pt - mu) / (sigma + 1e-8)

            # print("z =", z, "> zmax =", z_max)
            prev_pt = observations[i - 1]
            mse = np.sqrt((pt - prev_pt) ** 2)
            if mse > z_max:
                print(mse)
                # observations[i] = m * i + np.median(observations[:10])
                observations[i] = m + prev_pt
                # found = False
                # for j in range(window):
                #     idx = i + j
                #     if idx >= obs_length:
                #         break
                #     z = abs(pt - mu) / (sigma + 1e-8)
                #     if z < z_max:
                #         found = True
                #         prev_idx = max(0, i - 1)
                #         dy = (observations[idx] - observations[prev_idx]) / (j + 1)
                #         for k in range(j + 1):
                #             if (i + k) < obs_length:
                #                 observations[i + k] = observations[prev_idx] + k * dy
                #         # observations[i] = med
                #         break

                # end = min(obs_length, i - 1)
                # if not found:
                #     observations[i : (i + window + 1)] = observations[i - 1]
        else:
            observations[0] = np.median(observations[:30])

    ret_coords[:, 1] = observations

    return ret_coords


def gradient(img):
    img_f = img.astype(np.float32)
    kx = np.array([[0, 0, 0], [-0.5, 0, 0.5], [0, 0, 0]], np.float32)
    ky = kx.T
    gy = cv2.filter2D(img_f, -1, ky, borderType=cv2.BORDER_REPLICATE)
    gx = cv2.filter2D(img_f, -1, kx, borderType=cv2.BORDER_REPLICATE)
    return np.sqrt(0.65 * gy**2 + gx**2, dtype=np.float32)


def build_gaussian_filter(nx, ny):
    x = np.arange(nx, dtype=np.float32) - (nx - 1) / 2
    y = np.arange(ny, dtype=np.float32) - (ny - 1) / 2
    xx, yy = np.meshgrid(x, y)
    k = np.exp(-(xx**2) / (nx / 4) ** 2) * np.exp(-(yy**2) / (ny / 4) ** 2)
    k /= k.sum()
    return k


def lowpass(img, nx, ny):
    # f = np.fft.fft2(img)
    # fshift = np.fft.fftshift(f)
    # rows, cols = img.shape
    # crow, ccol = rows // 2, cols // 2
    # mask = np.zeros((rows, cols), np.uint8)
    # mask[crow - nx : crow + nx, ccol - ny : ccol + ny] = 0
    # fshift_filtered = fshift * mask
    # f_ishift = np.fft.ifftshift(fshift_filtered)
    # img_back = np.fft.ifft2(f_ishift)
    # img = np.abs(img_back).astype(np.uint8)

    k = build_gaussian_filter(nx, ny)
    img = cv2.filter2D(img, ddepth=-1, kernel=k, borderType=cv2.BORDER_REPLICATE)

    return img


def central_gradient(img):
    f = img.astype(np.float32)
    gx = np.empty_like(f)
    gy = np.empty_like(f)

    gx[:, 1:-1] = 0.5 * (f[:, 2:] - f[:, :-2])
    gy[1:-1, :] = 0.5 * (f[2:, :] - f[:-2, :])

    gx[:, 0] = f[:, 1] - f[:, 0]
    gx[:, -1] = f[:, -1] - f[:, -2]
    gy[0, :] = f[1, :] - f[0, :]
    gy[-1, :] = f[-1, :] - f[-2, :]

    return np.sqrt(0.65 * gy**2 + gx**2, dtype=np.float32)


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

    # # img = ed_bg_filter(img)
    # # img = spatial_bg_filter(img)
    # img = reg_bg_filter(img)
    # # img = bg_filter(img).astype(np.float32)
    # # img = clahe_bg_filter(img)
    # # img = spatial_filter(base_name, img)

    # cv2.imwrite(base_name + "_spatial.jpg", img)

    # # img = cv2.GaussianBlur(img, (5, 11), 5 / 2, 11 / 2)
    # img = lowpass(img, 11, 5)
    # cv2.imwrite(base_name + "_lowpass.jpg", img)

    # img_diffy = cv2.Sobel(img.astype(np.float32), cv2.CV_32F, 0, 1, ksize=3)
    # img_diffx = cv2.Sobel(img.astype(np.float32), cv2.CV_32F, 1, 0, ksize=3)
    # img_diff = np.sqrt(0.65 * img_diffy**2 + img_diffx**2)
    # # img_diff = gradient(img)
    # # img_diff = central_gradient(img)
    # # img_diff = cv2.GaussianBlur(img_diff, (3, 1), 3 / 2, 1 / 2)
    # img_diff = lowpass(img_diff, 1, 3)
    # cv2.imwrite(base_name + "_gradient.jpg", img_diff)
    # img = img.astype(np.float32) * img_diff.astype(np.float32)
    # img -= img.min()
    # img /= img.max()
    # img = (img * 255).astype(np.uint8)

    # cv2.imwrite(base_name + "_mult.jpg", img)

    img = spatial_filter(base_name, img)
    cv2.imwrite(base_name + "_spatial.jpg", img)

    ret_coords = get_max_coor(img)
    cv2.imwrite(
        base_name + "_max.jpg",
        draw_line(img_raw.copy(), ret_coords),
    )
    # from scipy.ndimage import median_filter
    # ret_coords[:, 1] = median_filter(ret_coords[:, 1], 15)
    signal = ret_coords[:, 1]
    window_size = 15
    half_win = window_size // 2
    padded_signal = np.pad(signal, (half_win, half_win), mode="edge")
    filtered_signal = np.zeros_like(signal)
    for i in range(len(signal)):
        window = padded_signal[i : i + window_size]
        ret_coords[i, 1] = np.median(window)
    cv2.imwrite(
        base_name + "_med.jpg",
        draw_line(img_raw.copy(), ret_coords),
    )

    ret_coords = ol_removal(ret_coords)
    cv2.imwrite(
        base_name + "_ol.jpg",
        draw_line(img_raw.copy(), ret_coords),
    )

    # ret_coords = outlier_removal(ret_coords)
    # cv2.imwrite(
    #     base_name + "_outlier.jpg",
    #     draw_line(img_raw.copy(), ret_coords),
    # )

    # ret_coords = linearization(ret_coords)
    # cv2.imwrite(
    #     base_name + "_linear.jpg",
    #     draw_line(img_raw.copy(), ret_coords),
    # )

    # ret_coords = median_filter(ret_coords)
    # cv2.imwrite(
    #     base_name + "_median.jpg",
    #     draw_line(img_raw.copy(), ret_coords),
    # )

    ret_coords = kalman_filter(ret_coords)

    cv2.imwrite(
        base_name + "_detected_pre.jpg",
        draw_line(img_raw.copy(), ret_coords),
    )

    ret_coords[:, 1] = height - ret_coords[:, 1]

    return ret_coords


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
    # path = "data/no_background"

    path = "data/real"
    # path = "data/current"
    # path = "data/real2"
    path = "data/new_capture"

    save_path = "data/result"
    interval = 6
    main(path, save_path, interval)
