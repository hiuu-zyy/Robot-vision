import cv2
import numpy as np
import pickle
# import conveyor_lib

cap = cv2.VideoCapture(0)

# Conveyor belt library
# relay = conveyor_lib.Conveyor()
i=1
def calculate_world_position(intrinsics_matrix, rvecs, tvecs, pixel_coords, image_index):
        # Chọn rvecs và tvecs tương ứng với hình ảnh đang xử lý
    rvec = rvecs[image_index]
    tvec = tvecs[image_index]

        # Chuyển đổi từ toạ độ pixel sang toạ độ 3D trong không gian camera
    pixel_coords_homogeneous = np.concatenate((pixel_coords, np.ones((pixel_coords.shape[0], 1))), axis=1)
    camera_coords = np.dot(np.linalg.inv(intrinsics_matrix), pixel_coords_homogeneous.T).T[:, :3]

        # Chuyển đổi từ không gian camera sang không gian thế giới
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    world_coords = np.dot(rotation_matrix.T, camera_coords.T - tvec.reshape(-1, 1))
    return world_coords


while True:
    _, frame = cap.read()

    belt = frame[ 315:400, 417:530]
    gray_belt = cv2.cvtColor(belt, cv2.COLOR_BGR2GRAY)
    _, threshold = cv2.threshold(gray_belt, 50, 255, cv2.THRESH_BINARY)

    #Detect the Nuts
    contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        (x, y, w, h) = cv2.boundingRect(cnt)

        #Calculate area
        area = cv2.contourArea(cnt)

        # Distinguish small and big nuts
        if area > 500:
            cv2.rectangle(belt, (x, y), (x + w, y + h), (0, 0, 255), 1)
        elif 100 < area < 500:
            cv2.rectangle(belt, (x, y), (x + w, y + h), (0, 255, 0), 1)
            
        objcenter = np.array([[x + w/2 + 315, y + h/2 + 417]])
        cv2.circle(belt, (int(x+w/2), int(y+h/2)), 3, (0 , 0, 255), -1)

    # Đọc dữ liệu từ file .pkl
    with open('tvecs.pkl', 'rb') as f:
        tvecs = pickle.load(f)

    with open('rvecs.pkl', 'rb') as f:
        rvecs = pickle.load(f)

    with open('cameraMatrix.pkl', 'rb') as f:
        cameraMatrix = pickle.load(f)
    print(tvecs)
    

    

        # Tính toán vị trí của vật trong không gian thế giới cho hình ảnh thứ ba
    image_index = 2
    world_coords = calculate_world_position(cameraMatrix, rvecs, tvecs, objcenter, image_index)


    print("Center image point:\n {},{} \n".format(x+w/2+315, y+h/2+417))    
    print("Coordinate in real world:\n {} \n".format(world_coords))


    cv2.imshow("Frame", frame)
    cv2.imshow("belt", belt)
    cv2.imshow("threshold", threshold)

    key = cv2.waitKey(1)
    if key == 27:
       break
    #elif key == ord('n'):
     #   relay.turn_on()
    #elif key == ord("m"):
     #   relay.turn_off()

cap.release()
cv2.destroyAllWindows()