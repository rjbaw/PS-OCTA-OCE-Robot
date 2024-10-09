import cv2
import numpy as np
import math
import os

#image = cv2.imread('initial_test/test.jpg')
#alpha = 1.5 # Contrast control (1.0-3.0)
#beta = 0 # Brightness control (0-100)
#adjusted = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
#cv2.imwrite('adjusted.jpg', adjusted)
def skeletonize(image):
    size = np.size(image)
    skel = np.zeros(image.shape, np.uint8)

    # Get a cross-shaped structuring element
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))

    # Repeat steps until erosion erodes the whole image
    while True:
        # Erode the image
        eroded = cv2.erode(image, element)
        # Open the image
        temp = cv2.dilate(eroded, element)
        temp = cv2.subtract(image, temp)
        # Accumulate the result
        skel = cv2.bitwise_or(skel, temp)
        image = eroded.copy()
        # If no white pixels left, exit the loop
        if cv2.countNonZero(image) == 0:
            break

    return skel

def non_max_suppression(image):
    kernel = np.ones((3, 3), np.uint8)
    dilated = cv2.dilate(image, kernel)
    suppressed = np.where(image == dilated, image, 0)
    return suppressed.astype(np.uint8)

def detect_lines(image_path, save_dir):
    img = cv2.imread(image_path)
    if img is None:
        print("Error: Could not load image.")
        return

    base_name = os.path.splitext(os.path.basename(image_path))[0]
    base_name = os.path.join(save_dir, base_name)
    #hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #lower_white = np.array([0, 0, 245], dtype=np.uint8)
    #upper_white = np.array([180, 15, 255], dtype=np.uint8)
    #mask = cv2.inRange(hsv, lower_white, upper_white)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(img, (5, 5), 0)
    #cv2.imwrite(base_name + "_blurred.jpg", blurred)

    # Filter based on image intensity range (for bright white areas)
    _, mask = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)
    cv2.imwrite(base_name + "_mask.jpg", mask)

    #suppressed = non_max_suppression(mask).astype(np.uint8)
    #cv2.imwrite(base_name + "_suppressed.jpg", suppressed)

    edges = cv2.Canny(mask, 50, 150)
    cv2.imwrite(base_name + "_edges.jpg", edges)
    gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    skeleton = skeletonize(gray)
    cv2.imwrite(base_name + "_skeleton.jpg", skeleton)

    lines = cv2.HoughLines(skeleton, 1, np.pi / 180, 160)
    #lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=50, maxLineGap=10)

    if lines is None:
        print("No lines were detected.", image_path)
        return

    print("Number of lines detected by hough transform", len(lines), image_path)

    if lines is None:
        print("No lines were detected.")
        return

    # Draw the detected lines
    #output_img = img.copy()
    #for line in lines:
    #    x1, y1, x2, y2 = line[0]
    #    cv2.line(output_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    #    # Calculate the angle of the line
    #    angle = math.degrees(np.arctan2(y2 - y1, x2 - x1))
    #    print(f"Line detected with angle: {angle:.2f} degrees")


    detected_lines = []
    for line in lines:
        rho, theta = line[0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        detected_lines.append(((x1, y1), (x2, y2), rho, theta))

    #double_lines = []
    #for i in range(len(detected_lines)):
    #    for j in range(i + 1, len(detected_lines)):
    #        rho1, theta1 = detected_lines[i][2], detected_lines[i][3]
    #        rho2, theta2 = detected_lines[j][2], detected_lines[j][3]
    #        if abs(theta1 - theta2) < 0.1 and abs(rho1 - rho2) > 10 and abs(rho1 - rho2) < 50:
    #            double_lines.append((detected_lines[i], detected_lines[j]))

    output_img = img.copy()
    for line in detected_lines:
        cv2.line(output_img, line[0], line[1], (0, 255, 0), 2)
        
        angle = math.degrees(line[3])
        print(f"Line detected with angle: {angle:.2f} degrees")

    cv2.imwrite(base_name + "_detected_lines.jpg", output_img)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # Draw the detected contours
    output_img = img.copy()
    for contour in contours:
        if cv2.arcLength(contour, False) > 100:  # Filter small contours
            cv2.drawContours(output_img, [contour], -1, (0, 255, 0), 2)

    #height, width, _ = img.shape
    #for contour in contours:
    #    if cv2.arcLength(contour, False) > 100:  # Filter small contours
    #        # Fit a line to the contour
    #        [vx, vy, x, y] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)

    #        # Calculate the endpoints of the line extending from left to right edge
    #        left_y = int(((-x * vy / vx) + y)[0])
    #        right_y = int((((width - x) * vy / vx) + y)[0])

    #        # Draw the line
    #        cv2.line(output_img, (0, left_y), (width - 1, right_y), (0, 255, 0), 2)
    cv2.imwrite(base_name + "_contour.jpg", output_img)


#detect_lines('initial_test/test.jpg', 'detected_lines.jpg')
path = "3d_aligned"
for file in os.listdir(path):
    if os.path.isfile(os.path.join(path, file)):
        detect_lines(os.path.join(path, file), 'result')
