import cv2
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import time

def calculate_center_of_mass(image, k):
    total_x_u = 0
    total_y_u = 0
    total_u = 0

    total_x_l = 0
    total_y_l = 0
    total_l = 0

    count = 0
    center = [int(image.shape[0]/2), int(image.shape[1]/2)]
    # Tạo danh sách giá trị từ -center[1] đến image.shape[1] - center[1]
    y1 = center[0]
    
    # Tạo danh sách giá trị từ -center[0] đến image.shape[0] - center[0]
    x1 = center[1]
    # Lặp qua tất cả các pixel trên ảnh
    for y in range(image.shape[0]):
        for x in range(image.shape[1]):
            # Kiểm tra điều kiện y > kx
            if y - y1 < k * (x - x1):
                total_x_u += x * image[y, x]
                total_y_u += y * image[y, x]
                total_u += image[y, x]
                count += 1
            else:
                total_x_l += x * image[y, x]
                total_y_l += y * image[y, x]
                total_l += image[y, x]
                count += 1
    
    # Tính trọng tâm
    if count != 0:
        centroid_x_u = total_x_u / total_u
        centroid_y_u = total_y_u / total_u
        centroid_u = np.array([centroid_x_u, centroid_y_u])

        centroid_x_l = total_x_l / total_l
        centroid_y_l = total_y_l / total_l
        centroid_l = np.array([centroid_x_l, centroid_y_l])
    else:
        centroid_u = None
        centroid_l = None
    #print(y1)
    #print(x1)
    return centroid_u, centroid_l


def estimate_orientation(image, threshold=5):

    _, binary_image = cv2.threshold(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 20, 255, cv2.THRESH_BINARY)
    # binary_image = cv2.bitwise_not(binary_image)
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Khởi tạo danh sách các hộp giới hạn
    bounding_boxes = []

    # Lặp qua danh sách contours và tính toán hộp giới hạn cho mỗi contour
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        bounding_boxes.append((x, y, w, h))

    # Tìm chỉ số của contour có kích thước lớn nhất
    max_index = np.argmax([w for x, y, w, h in bounding_boxes])

    # Lấy hộp giới hạn của contour có kích thước lớn nhất
    x, y, w, h = bounding_boxes[max_index]
    center = [int(x+w/2), int(y+h/2)]

    #Calculate area
    area = cv2.contourArea(contours[max_index])

        # Distinguish small and big nuts
    if area > 500:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
    elif 100 < area < 500:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    binary_image = binary_image[y:y+h, x:x+w]
    # cv2.imshow("binary", binary_image)
    # Đảo ngược giá trị của pixel
    # binary_image = cv2.bitwise_not(binary_image)
    # binary_image = detect_image(image)
    # Biến đổi giá trị 255 thành 1
    binary_image[binary_image == 255] = 1
    
    # Khởi tạo giá trị hướng ban đầu và hướng sau
    orientation = 0
    new_orientation = 90
    k = 0
    it = 0
    
    # Lặp cho đến khi góc giữa hai đường chia nhỏ hơn ngưỡng threshold
    while abs(new_orientation - orientation) >= threshold:
        it += 1
        # Tính trọng tâm của hai nửa ảnh
        start = time.time()
        top_mass_center, bottom_mass_center = calculate_center_of_mass(binary_image, -k)

        k = (bottom_mass_center[0] - top_mass_center[0])/( bottom_mass_center[1] - top_mass_center[1])
        
        
        # Tính hướng mới
        orientation = new_orientation
        new_orientation = 90 - np.arctan2(bottom_mass_center[1] - top_mass_center[1], bottom_mass_center[0] - top_mass_center[0]) * 180 / np.pi
    
    return new_orientation, center




def run(image_path,is_scaled=False):

    image = cv2.imread(image_path)
    desired_width = 50

    # Calculate the aspect ratio
    ratio = desired_width / image.shape[1]

    # Calculate the new height based on the aspect ratio
    desired_height = int(image.shape[0] * ratio)

    # Resize the image
    resized_image = cv2.resize(image, (desired_width, desired_height))
    if(is_scaled):
        estimated_orientation, center = estimate_orientation(resized_image)
    else:
        estimated_orientation, center = estimate_orientation(image)

    return estimated_orientation, center

image_path = 'Object.jpg'

cap = cv2.VideoCapture(0)
while True:
    _, frame = cap.read()
    
    belt = frame[ 315:400, 417:530]
    run(belt)

