import sys

from detection import *
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QIcon, QFont
from PyQt5.QtWidgets import (QApplication, QWidget, QPushButton, QToolTip, QGridLayout, QLabel, QLineEdit, QSlider,
                             QVBoxLayout, QHBoxLayout)

import cobot_R1
import cobot_R2

import time

cap = cv2.VideoCapture(0)

class MyApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.execute_once = False
    def calculate_center_of_mass(self, image, k):
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
    
    def estimate_orientation(self, image, threshold=5):

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
        
        # Lặp cho đến khi góc giữa hai đường chia nhỏ hơn ngưỡng threshold
        while abs(new_orientation - orientation) >= threshold:
            # Tính trọng tâm của hai nửa ảnh
            top_mass_center, bottom_mass_center = calculate_center_of_mass(binary_image, -k)

            k = (bottom_mass_center[0] - top_mass_center[0])/( bottom_mass_center[1] - top_mass_center[1])
            
            
            # Tính hướng mới
            orientation = new_orientation
            new_orientation = 90 - np.arctan2(bottom_mass_center[1] - top_mass_center[1], bottom_mass_center[0] - top_mass_center[0]) * 180 / np.pi
        
        return new_orientation, center




    def run(self, cap, is_scaled=False):

        _, frame = cap.read()
    
        image = frame[ 315:400, 417:530]

        # image = cv2.imread(image_path)
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
    

    def predefine_pose(self):
        pass

    def move_Linear(self, pose, dx = 0, dy = 0, dz = 0, drx = 0, dry = 0, drz = 0,R = 1):
        x = pose[0] + dx
        y = pose[1] + dy
        z = pose[2] + dz
        rx = pose[3] + drx
        ry = pose[4] + dry
        rz = pose[5] + drz

        pose_new = [x,y,z,rx,ry,rz]
        
        if(R == 1):
            print("R1")
            cobot_R1.MoveL(float(x), float(y), float(z), float(rx), float(ry), float(rz), float(300), float(400))
        elif(R == 2):
            print("R2")
            cobot_R2.MoveL(float(x), float(y), float(z), float(rx), float(ry), float(rz), float(300), float(400))

        while(True):
            if(MyApp.check_if_arrived_at_dest_tcp(self,pose_new,R)):
                print("break!")
                break
        pass

    def move_Joint(self, pose, dj1 = 0, dj2 = 0, dj3 = 0, dj4 = 0, dj5 = 0, dj6 = 0, R = 1):
        j1 = pose[0] + dj1
        j2 = pose[1] + dj2
        j3 = pose[2] + dj3
        j4 = pose[3] + dj4
        j5 = pose[4] + dj5
        j6 = pose[5] + dj6

        pose_new = [j1,j2,j3,j4,j5,j6]

        if(R == 1):
            cobot_R1.MoveJ(float(j1), float(j2), float(j3), float(j4), float(j5), float(j6), float(300), float(400))
        elif(R == 2):
            cobot_R2.MoveJ(float(j1), float(j2), float(j3), float(j4), float(j5), float(j6), float(300), float(400))

        while(True):
            if(MyApp.check_if_arrived_at_dest_joint(self,pose_new,R)):
                break
        pass
    
    def check_if_arrived_at_dest_tcp(self, pose, R = 1):
        if(R == 1):
            cur_pose = [cobot_R1.GetCurrentTCP().x,cobot_R1.GetCurrentTCP().y,cobot_R1.GetCurrentTCP().z,cobot_R1.GetCurrentTCP().rx,cobot_R1.GetCurrentTCP().ry,cobot_R1.GetCurrentTCP().rz]

        elif(R == 2):
            cur_pose = [cobot_R2.GetCurrentTCP().x,cobot_R2.GetCurrentTCP().y,cobot_R2.GetCurrentTCP().z,cobot_R2.GetCurrentTCP().rx,cobot_R2.GetCurrentTCP().ry,cobot_R2.GetCurrentTCP().rz]

        for i in range(6):

            diff = float(abs(pose[i]) - abs(cur_pose[i]))
            if(diff < 0.5):
                return_result = True
            else:
                return_result = False
                break

            print(diff,return_result)
        return return_result
    
    def check_if_arrived_at_dest_joint(self, pose, R=1):
        if(R == 1):
            cur_pose = [cobot_R1.GetCurrentJoint().j0,cobot_R1.GetCurrentJoint().j1,cobot_R1.GetCurrentJoint().j2,cobot_R1.GetCurrentJoint().j3,cobot_R1.GetCurrentJoint().j4,cobot_R1.GetCurrentJoint().j5]
            print("R1")
        elif(R == 2):
            cur_pose = [cobot_R2.GetCurrentJoint().j0,cobot_R2.GetCurrentJoint().j1,cobot_R2.GetCurrentJoint().j2,cobot_R2.GetCurrentJoint().j3,cobot_R2.GetCurrentJoint().j4,cobot_R2.GetCurrentJoint().j5]
            print("R2")

        for i in range(6):
            diff = float(abs(pose[i]) - abs(cur_pose[i]))
            print(diff)
            if(diff < 0.5):
                return_result = True
            else:
                return_result = False
                break

            print(return_result)

        return return_result
    
    def R1_movement(self):
        print(cobot_R1.GetDigitalIn()[8])
        capture_image_pos = [] 
        home_pos = [500.00,0.00,250.00,90.00,0.00,90.00]
        pos_place = [674.54,547.68,54.96,176.93,89.93,-94.08]
        
        campos = [680.00,-30.00,305.00,89.99,0.01,0.00]
        campos_180 = [-191.81,-24.64,-52.87,-12.50,-89.99,102.00]
        put_pos = [680.00,-70.00,55.00,90.00,-0.01,0.00]
        put_pos_alt = [674.54,80.00,55.00,176.91,89.93,-94.10]
        pick_top = [693.22,75.00,70.00,89.99,0.01,0.00]
        campos_vert = [680.00,-30.00,315.00,89.99,0.01,0.00]

        go_put_joint = [-131.17,-54.97,-73.85,-51.23,138.8,-90.09]
        put_pos_alt_joint = [-131.17,-50.96,-73.48,-55.62,138.8,-90.09]
        go_home_joint = [-192.68,2.10,-93.34,1.25,-90.01,-77.26]
        go_cam_joint = [-191.81,-25.04,-50.79,-14.18,-89.99,-78.14]
        go_pick_top_joint = [-182.88,-27.38,-76.04,13.41,-89.99,-87.07]
        cobot_R1.SetBaseSpeed(40.00 * 0.01)

        MyApp.move_Linear(self,home_pos, R=1)
        #time.sleep(5)
        cobot_R1.ManualScript("gripper_jrt_jegb4140_init(0)")
        #cobot_R2.ManualScript("gripper_jrt_jegb485_init(0)")
        time.sleep(6)
        cobot_R1.ManualScript("gripper_jrt_jegb4140_set(0, 50,50,20)")
        #cobot_R2.ManualScript("gripper_jrt_jegb485_set(0, 50,50,20)")
        time.sleep(5)
        cobot_R1.ManualScript("gripper_jrt_jegb4140_go(0, 39)")
        time.sleep(2)
        while(not cobot_R1.GetDigitalIn()[8]):
            pass

        # move to cature image position
        MyApp.move_Linear(self, capture_image_pos)
        time.sleep(3)
        angle, center = MyApp.run(self, cap, is_scaled = True )
        pick_pos = [center[0],center[1],55,90,angle]
        #ManualScript("wait(JRT_JEGB > 36 out)")
        MyApp.move_Linear(self,pick_pos, dz = 50)
        time.sleep(1)
        MyApp.move_Linear(self,pick_pos)
        time.sleep(2)
        cobot_R1.ManualScript("gripper_jrt_jegb4140_go(0, 20)")
        time.sleep(1)
        MyApp.move_Linear(self,pick_pos, dz = 50)
        time.sleep(1)
        MyApp.move_Linear(self,home_pos)
        time.sleep(1)
        
        MyApp.move_Joint(self,campos_180, dj6 = -50)
        time.sleep(0.5)
        MyApp.move_Joint(self,campos_180)
        time.sleep(0.5)
        MyApp.move_Joint(self,campos_180, dj6 = 50)
        time.sleep(1)

        MyApp.move_Joint(self,campos_180,dj6 = -130)
        time.sleep(1)

        MyApp.move_Linear(self,campos, dry = -12)
        #time.sleep(1)
        MyApp.move_Linear(self,campos, drz = 12)
        #time.sleep(1)
        MyApp.move_Linear(self,campos, dry = 12)
        #time.sleep(1)
        MyApp.move_Linear(self,campos, drz = -12)
        #time.sleep(1)
        MyApp.move_Linear(self,campos, dry = -12)
        #time.sleep(1)

        MyApp.move_Linear(self,campos)
        time.sleep(1)

        MyApp.move_Linear(self,put_pos)
        time.sleep(1)
        cobot_R1.ManualScript("gripper_jrt_jegb4140_go(0, 39)")
        #time.sleep(5)
        MyApp.move_Linear(self,put_pos, dx = -17)
        time.sleep(1)
        cobot_R1.ManualScript("gripper_jrt_jegb4140_go(0, 12)")
        time.sleep(1)
        MyApp.move_Linear(self,home_pos)
        time.sleep(1)
        MyApp.move_Joint(self,go_put_joint)
        #time.sleep(8)
        cobot_R1.ManualScript("gripper_jrt_jegb4140_go(0, 39)")
        time.sleep(1)
        MyApp.move_Joint(self,put_pos_alt_joint)
        time.sleep(1)
        MyApp.move_Joint(self,go_home_joint)
        time.sleep(1)
        #MyApp.move_Linear(self,pick_top, dz = 50)
        #time.sleep(1)
        #MyApp.move_Linear(self,pick_top)
        ##time.sleep(5)
        #cobot_R1.ManualScript("gripper_jrt_jegb4140_go(0, 12)")
        #time.sleep(1)
        #MyApp.move_Joint(self,go_cam_joint)
        #time.sleep(1)
        #MyApp.move_Joint(self,go_cam_joint,dj6 = 318)
        #time.sleep(1)
        #MyApp.move_Joint(self,go_pick_top_joint)
        #time.sleep(1)
        #MyApp.move_Linear(self,pick_top)
        #time.sleep(5)
        #cobot_R1.ManualScript("gripper_jrt_jegb4140_go(0, 39)")
        #time.sleep(1)
        #MyApp.move_Linear(self,pick_top, dz = 50)
        #time.sleep(1)
        #MyApp.move_Linear(self,home_pos)
        #time.sleep(1)

        #MyApp.move_Linear(self,home_pos)
        #ManualScript("gripper_jrt_jegb4140_go(0, 28)")
        #ManualScript("wait(JRT_JEGB < 20 out)")
        #print(systemstat_global)
    
    def R2_movement(self):
        home_pos_R2 = [400.00,0.00,400.00,90.00,0.00,90.00]
        pick_pos_up_joint = [-156.86,-36.47,-70.50,16.97,-89.95,-23.16]
        pick_pos_tcp = [830.00,475.00,85.00,90.00,0.00,90.00]
        cam_pos_joint = [-225.50,9.83,-109.83,10,-89.91,45.47]
        anchor_pos_joint = [-180.79, -22.93, -98.65,31.59,-89.95,0.74]

        #time.sleep(5)
        cobot_R2.ManualScript("gripper_jrt_jegb485_init(0)")
        time.sleep(6)
        cobot_R2.ManualScript("gripper_jrt_jegb485_set(0, 50,50,20)")
        time.sleep(6)
        cobot_R2.ManualScript("gripper_jrt_jegb485_go(0, 22)")
        time.sleep(3)
        cobot_R2.ManualScript("gripper_jrt_jegb485_go(0, 60)")
        time.sleep(3)

        MyApp.move_Joint(self,pick_pos_up_joint,R = 2)
        MyApp.move_Linear(self,pick_pos_tcp,R = 2)
        time.sleep(2)
        cobot_R2.ManualScript("gripper_jrt_jegb485_go(0, 22)")
        time.sleep(2)
        MyApp.move_Linear(self,pick_pos_tcp,dz = 50, R = 2)
        time.sleep(1)
        MyApp.move_Joint(self,cam_pos_joint,R = 2)
        MyApp.move_Joint(self,cam_pos_joint,dj6 = -285.47,R = 2)
        MyApp.move_Joint(self,cam_pos_joint,R = 2)

        time.sleep(1)

        MyApp.move_Joint(self,anchor_pos_joint,R = 2)
        time.sleep(1)
        MyApp.move_Linear(self,home_pos_R2, R=2)
        pass

    def send_home(self):
        cobot_R1.SetProgramMode(cobot_R1.PG_MODE.REAL)
        cobot_R2.SetProgramMode(cobot_R2.PG_MODE.REAL)


        home_pos_R1 = [500.00,0.00,250.00,90.00,0.00,90.00]
        cobot_R1.SetBaseSpeed(40.00 * 0.01)
        MyApp.move_Linear(self,home_pos_R1, R=1)
        
        home_pos_R2 = [400.00,0.00,400.00,90.00,0.00,90.00]
        cobot_R2.SetBaseSpeed(40.00 * 0.01)
        MyApp.move_Linear(self,home_pos_R2, R=2)

        pass
    def task(self):
        MyApp.send_home(self)
        MyApp.R1_movement(self)
        #input("Press Enter to Continue....")
        MyApp.R2_movement(self)



        pass


    def test_function(self):
        cobot_R1.SetProgramMode(cobot_R1.PG_MODE.REAL)
        cobot_R2.SetProgramMode(cobot_R2.PG_MODE.REAL)
        cobot_R1.SetBaseSpeed(30.00 * 0.01)
        time.sleep(1)
        cobot_R2.SetBaseSpeed(30.00 * 0.01)
        time.sleep(1)
        cur_pose_R1 = [cobot_R1.GetCurrentTCP().x,cobot_R1.GetCurrentTCP().y,cobot_R1.GetCurrentTCP().z,cobot_R1.GetCurrentTCP().rx,cobot_R1.GetCurrentTCP().ry,cobot_R1.GetCurrentTCP().rz]
        MyApp.move_Linear(self,cur_pose_R1,dx = 50,R = 1)
        cur_pose_R2 = [cobot_R2.GetCurrentTCP().x,cobot_R2.GetCurrentTCP().y,cobot_R2.GetCurrentTCP().z,cobot_R2.GetCurrentTCP().rx,cobot_R2.GetCurrentTCP().ry,cobot_R2.GetCurrentTCP().rz]
        MyApp.move_Linear(self,cur_pose_R2,dx = 50, R = 2)
        pass

    def do_task_for_both(cases):
        if(cases == 1):
            cobot_R1.SetBaseSpeed(float(setspd_sld.value()) * 0.01)
            #cobot_R2.SetBaseSpeed(float(setspd_sld.value()) * 0.01)
        elif(cases == 2):
            cobot_R1.SetProgramMode(mode_R1)
            #cobot_R2.SetProgramMode(mode_R2)

    def initUI(self):
        grid = QGridLayout()
        self.setLayout(grid)
        global spd, setspd_sld
        global ip_get, ip

        #####
        global btn_R1, btn_R2
        btn_R1 = QPushButton('Connect R1', self)
        btn_R2 = QPushButton('Connect R2', self)
        #ip = QLineEdit('192.168.0.10')
        ip_R1 = '192.168.0.10'
        ip_R2 = '192.168.0.20'
        init_btn_R1 = QPushButton('Initialize R1')
        init_btn_R2 = QPushButton('Initialize R2')
        grid.addWidget(init_btn_R1,0,1)
        grid.addWidget(init_btn_R2,1,1)
        grid.addWidget(btn_R1, 0, 0)
        grid.addWidget(btn_R2, 1, 0)
        btn_R1.setToolTip('This is a <b>QPushButton</b> widget')
        btn_R1.resize(btn_R1.sizeHint())
        btn_R1.setStyleSheet("background-color: red")

        btn_R2.setToolTip('This is a <b>QPushButton</b> widget')
        btn_R2.resize(btn_R2.sizeHint())
        btn_R2.setStyleSheet("background-color: red")
        # btn.clicked.connect(QCoreApplication.instance().quit)
        btn_R1.clicked.connect(lambda: cobot_R1.ToCB(ip_R1))
        init_btn_R1.clicked.connect(cobot_R1.CobotInit)

        btn_R2.clicked.connect(lambda: cobot_R2.ToCB(ip_R2))
        init_btn_R2.clicked.connect(cobot_R2.CobotInit)

        #####
        timer = QTimer(self)
        timer.setInterval(10)
        timer.timeout.connect(self.datareset)
        timer.start()

        #####
        status_hgrid = QHBoxLayout()
        mode_btn = QPushButton('Mode')
        global mode_lb, robot_lb
        mode_lb = QLabel('Mode : ')
        robot_lb = QLabel('Robot : ')
        grid.addWidget(mode_btn, 2, 0)
        status_hgrid.addWidget(mode_lb)
        status_hgrid.addWidget(robot_lb)
        grid.addLayout(status_hgrid, 2, 1)

        global mode_R1, mode_R2
        mode_R1 = cobot_R1.PG_MODE.SIMULATION
        #mode_R2 = cobot_R2.PG_MODE.SIMULATION
        mode_btn.clicked.connect(lambda: cobot_R1.SetProgramMode(mode_R1))

        #####
        setspd_hgrid = QHBoxLayout()
        setspd_btn = QPushButton('Speed Bar')
        setspd_sld = QSlider(Qt.Horizontal)
        setspd_sld.setTickPosition(1)
        setspd_sld.setMaximum(100)
        setspd_sld.setMinimum(0)
        spd = float(setspd_sld.value()) * 0.01
        global spd_lb
        spd_lb = QLabel(str(float(setspd_sld.value()) * 0.01))
        setspd_hgrid.addWidget(spd_lb)
        setspd_hgrid.addWidget(setspd_sld)
        grid.addLayout(setspd_hgrid, 3, 1)
        grid.addWidget(setspd_btn, 3, 0)
        setspd_btn.clicked.connect(lambda: cobot_R1.SetBaseSpeed(float(setspd_sld.value()) * 0.01))

        ####
        mscript_btn_R1 = QPushButton('Manual Script R1')
        mscript_le_R1 = QLineEdit()
        grid.addWidget(mscript_btn_R1, 4, 0)
        grid.addWidget(mscript_le_R1, 4, 1)
        mscript_btn_R2 = QPushButton('Manual Script R2')
        mscript_le_R2 = QLineEdit()
        grid.addWidget(mscript_btn_R2, 5, 0)
        grid.addWidget(mscript_le_R2, 5, 1)
        
        ####
        resume_btn = QPushButton('Resume')
        grid.addWidget(resume_btn, 6, 0)
        pause_btn = QPushButton('Pause')
        grid.addWidget(pause_btn, 6, 1)
        stop_btn = QPushButton('Stop')
        grid.addWidget(stop_btn, 6, 2)
        execute_task_btn = QPushButton('Execute task')
        grid.addWidget(execute_task_btn, 7, 0)
        test_fun = QPushButton('Test')
        grid.addWidget(test_fun, 7, 1)

        mscript_btn_R1.clicked.connect(lambda: cobot_R1.ManualScript(mscript_le_R1.text()))
        mscript_btn_R2.clicked.connect(lambda: cobot_R2.ManualScript(mscript_le_R2.text()))
        pause_btn.clicked.connect(lambda: cobot_R1.MotionPause())
        resume_btn.clicked.connect(lambda: cobot_R1.MotionResume())
        stop_btn.clicked.connect(lambda: cobot_R1.MotionHalt())
        execute_task_btn.clicked.connect(lambda: MyApp.task(self))
        test_fun.clicked.connect(lambda: MyApp.test_function(self))
        #####
        QToolTip.setFont(QFont('SansSerif', 10))
        self.setToolTip('This is a <b>QWidget</b> widget')

        self.setWindowTitle('Rainbow Robotics')
        self.setWindowIcon(QIcon('robotics-ci-1.png'))
        self.setGeometry(300, 300, 300, 300)
        self.show()

    def datareset(self):
        # global ip_get, ip
        # global spd, setspd_sld
        global mode, mode_lb, robot_lb
        global spd_lb
        global btn
        # global j0_le, j0_le, j0_le, j0_le, j0_le, j0_le, j0_le, j0_le

        setspd_f2 = round(float(setspd_sld.value()) * 0.01, 2)

        spd_lb.setText(str(setspd_f2))

        if cobot_R1.IsRobotReal() == True:
            mode_lb.setText('Mode : REAL')
            mode = cobot_R1.PG_MODE.SIMULATION
        elif cobot_R1.IsRobotReal() == False:
            mode_lb.setText('Mode : SIMULATION')
            mode = cobot_R1.PG_MODE.REAL

        if cobot_R1.IsIdle() == True:
            robot_lb.setText('ROBOT : IDLE')
        elif cobot_R1.IsIdle() == False:
            robot_lb.setText('ROBOT : RUN')

        if cobot_R1.IsCommandSockConnect() == True & cobot_R1.IsDataSockConnect() == True:
            btn_R1.setStyleSheet("background-color: green")
            btn_R1.setText('Connect')

        else:
            btn_R1.setStyleSheet("background-color: red")
            btn_R1.setText('Disconnect')

        if cobot_R2.IsCommandSockConnect() == True & cobot_R2.IsDataSockConnect() == True:
            btn_R2.setStyleSheet("background-color: green")
            btn_R2.setText('Connect')


        else:
            btn_R2.setStyleSheet("background-color: red")
            btn_R2.setText('Disconnect')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MyApp()
    sys.exit(app.exec_())

# connect = ConnectToCB('192.168.1.200')
#
# IsCommandSockConnect()
# IsDataSockConnect()
#
# # cobot.DisConnectToCB()
# time.sleep(1.5)
# jnt = Joint(0, 0, 90, 0, 0, 0)
# MoveJ(jnt, 100., 200.)
