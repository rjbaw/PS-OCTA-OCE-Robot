import cv2
import numpy as np


def params_image():
    n = 8
    mean_vals = []
    std_vals = []
    for i in range(n):
        img = cv2.imread(
            "data/background/new/" + str(i + 1) + ".jpg", flags=cv2.IMREAD_GRAYSCALE
        )
        img = img.astype(float)
        img = img[250:350, 200:300]
        cv2.imwrite("patch.jpg", img)
        mean_vals.append(np.mean(img))
        std_vals.append(np.sqrt(np.var(img)))
    mean_val = np.mean(mean_vals)
    std_val = np.mean(std_vals)
    var_val = std_val**2
    print(mean_val)
    print(var_val)
    print(std_val)
    return mean_val, var_val


def get_psd():
    n = 8
    power_spectrum = np.zeros((512, 500))
    for i in range(n):
        img = cv2.imread(
            "data/background/new/" + str(i + 1) + ".jpg", flags=cv2.IMREAD_GRAYSCALE
        )
        img = img.astype(float)
        dft = np.fft.fft2(img)
        dft = np.fft.fftshift(dft)
        power_spectrum += np.abs(dft) ** 2
    power_spectrum /= n
    print(power_spectrum)
    return power_spectrum


if __name__ == "__main__":
    params_image()
    power_spectrum = get_psd()
    fs = cv2.FileStorage("psd.yml", cv2.FILE_STORAGE_WRITE)
    fs.write("psd", power_spectrum.astype(np.float32))
    fs.release()
