import sys
import time
import cv2
import traceback
import numpy as np
import math
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from myunitree_robot import myunitree
from myDialog import myDialog
from PositionDialog import PositionDialog
from View3DDialog import View3DDialog

class Tread1(QThread):
    def __init__(self,parent):
        super().__init__(parent)
        self.parent = parent

    def run(self):
        try:
            while True:
                time.sleep(0.01)
                self.parent.sendCmd()
        except Exception as e:
            print("Tread1에서 예외 발생:")
            traceback.print_exc()

class CameraThread(QThread):
    update_image = pyqtSignal(QImage)

    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent
        self.camera = cv2.VideoCapture(0)
        # self.camera = cv2.VideoCapture('rtsp://192.168.10.223:554/live.sdp')

    def run(self):
        while True:
            ret, frame = self.camera.read()
            if ret:
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                q_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.update_image.emit(q_image)

class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi(r'./control_gui.ui', self) # Ui 연결
        self.myunitree_b1 = myunitree() # myunitree  class 불러와서 명명
        #----- 변수 초기화 ------------------------------------------
        self.velocity_0_Front_value = 0
        self.velocity_0_Back_value = 0
        self.velocity_1_Left_value = 0
        self.velocity_1_Right_value = 0
        self.yawspeed_value_L = 0
        self.yawspeed_value_R = 0
        self.vel_euler_0 = 0
        self.vel_euler_1 = 0
        self.vel_euler_2 = 0
        self.move_vel_0 = 0
        self.move_vel_1 = 0
        self.AutoMode_flag = False

        #------ Dialog ----------------------------------------------------
        self.actionGraph = QAction("Open Graph", self)
        self.actionPositionGraph = QAction("Position View", self)
        self.view_robot_3D = QAction("3D View", self)
        self.actionGraph.triggered.connect(self.open_graph_window)
        self.actionPositionGraph.triggered.connect(self.open_position_window)
        self.view_robot_3D.triggered.connect(self.open_view_robot_3D)

        self.fileMenu = self.menuBar().addMenu("Graph")
        self.fileMenu.addAction(self.actionGraph)
        self.fileMenu.addAction(self.actionPositionGraph)
        self.fileMenu.addAction(self.view_robot_3D)

        # ------ 버튼 -----------------------------------------------------
        self.connect_btn.clicked.connect(self.udp_connect) # 통신 연결 버튼
        self.camera_on_btn.clicked.connect(self.camera_on)
        # 컨트롤러 버튼
        self.N_btn.pressed.connect(self.Click_Front_Btn)
        self.N_btn.released.connect(self.Release_Front_Btn)
        self.S_btn.pressed.connect(self.Click_Back_Btn)
        self.S_btn.released.connect(self.Release_Back_Btn)
        self.W_btn.pressed.connect(self.Click_Left_Btn)
        self.W_btn.released.connect(self.Release_Left_Btn)
        self.E_btn.pressed.connect(self.Click_Right_Btn)
        self.E_btn.released.connect(self.Release_Right_Btn)

        self.Stop_btn.clicked.connect(self.Click_Stop_Btn)

        self.L_btn.pressed.connect(self.Click_Turn_L_Btn)
        self.L_btn.released.connect(self.Release_Turn_L_Btn)
        self.R_btn.pressed.connect(self.Click_Turn_R_Btn)
        self.R_btn.released.connect(self.Release_Turn_R_Btn)

        self.Up_btn.pressed.connect(self.Click_Up_Btn)
        self.Down_btn.pressed.connect(self.Click_Down_Btn)
        # euler, height 설정 버튼
        self.euler_btn.pressed.connect(self.Click_Euler_Setting_Btn)
        self.height_btn.pressed.connect(self.CLick_Height_Setting_Btn)

        self.auto_start_position_btn.pressed.connect(self.click_auto_start_Position)
        self.auto_end_position_btn.pressed.connect(self.click_auto_end_Position)

        self.Front_btn_pressed_state = False
        self.Back_btn_pressed_state = False
        self.Left_btn_pressed_state = False
        self.Right_btn_pressed_state = False
        self.Turn_L_btn_pressed_state = False
        self.Turn_R_btn_pressed_state = False
        # ------ 값 입력 ----------------------------------------------------
        self.input_vel_0.valueChanged.connect(self.vel_0_value_changed)
        self.input_vel_1.valueChanged.connect(self.vel_1_value_changed)
        self.input_yawspeed.valueChanged.connect(self.yawspeed_value_changed)

        self.input_euler_0.valueChanged.connect(self.vel_euler_value_0_changed)
        self.input_euler_1.valueChanged.connect(self.vel_euler_value_1_changed)
        self.input_euler_2.valueChanged.connect(self.vel_euler_value_2_changed)
        self.input_height.valueChanged.connect(self.bodyHeight_value_changed)

        self.input_position_0.valueChanged.connect(self.vel_position_value_0_changed)
        self.input_position_1.valueChanged.connect(self.vel_position_value_1_changed)

        # ------ Label -----------------------------------------------------
        self.SOC_label = self.findChild(QLabel, "SOC_label")
        self.Mode_label = self.findChild(QLabel, "mode_label")
        self.GaitType_label = self.findChild(QLabel, "gaittype_label")
        self.State_Position_0_label = self.findChild(QLabel, "state_position_0_label")
        self.State_Position_1_label = self.findChild(QLabel, "state_position_1_label")
        self.Yawspeed_value_label = self.findChild(QLabel, "yawspeed_value_label")
        self.Heading_value_label = self.findChild(QLabel,"Heading_label")
        #------ ComboBox ---------------------------------------------------
        self.Mode_ComboBox = self.findChild(QComboBox,"mode_comboBox")
        self.Mode_ComboBox.currentIndexChanged.connect(self.Change_mode_combobox)
        self.GaitType_ComboBox = self.findChild(QComboBox, "gaittype_comboBox")
        self.GaitType_ComboBox.currentIndexChanged.connect(self.Change_gaittype_comboBox)

#------ Dialog Window 띄우기 ----------------------
    def open_graph_window(self):
        self.graph_window = myDialog(self)
        self.graph_window.show()
    def open_position_window(self):
        self.graph_window = PositionDialog(self)
        self.graph_window.show()
    def open_view_robot_3D(self):
        self.view_window = View3DDialog(self)
        self.view_window.show()
#------ SendCmd -------------------------------------
    def sendCmd(self):
        self.myunitree_b1.sendCmd()

        self.highstate_textBrowser.append(self.myunitree_b1.highstate_info)

        self.data_SOC = self.myunitree_b1.hstate_bms_SOC
        self.data_mode = self.myunitree_b1.hstate_mode
        self.data_gaitType =self.myunitree_b1.hstate_gaitType
        self.data_yawspeed = self.myunitree_b1.hstate_yawspeed

        self.plot_data_bodyHeight = self.myunitree_b1.hstate_bodyHeight
        self.plot_data_footforce = self.myunitree_b1.hstate_footforce
        self.data_position_hstate = self.myunitree_b1.hstate_position

        self.view_data_rpy = self.myunitree_b1.hstate_rpy
        self.view_data_motorQ = self.myunitree_b1.hstate_motorQ
        self.view_data_quaternion =self.myunitree_b1.hstate_quaternion

        # 시계방향으로 증가시키면서 조합을 찾은 heading 값
        self.heading = math.degrees(2*(math.atan2(-self.view_data_quaternion[3],self.view_data_quaternion[0])))
        if self.heading < 0:
            self.heading = self.heading + 360
        if self.heading > 360:
            self.heading = self.heading - 360

        self.update_label()

        # Auto 모드
        if self.AutoMode_flag == True:
            self.Auto_move_2()

#------데이터 입력 이벤트------------
    def vel_0_value_changed(self, value):
        self.velocity_0_Front_value = value
        self.velocity_0_Back_value = -value
    def vel_1_value_changed(self, value):
        self.velocity_1_Left_value = value
        self.velocity_1_Right_value = -value
    def yawspeed_value_changed(self, value):
        self.yawspeed_value_L = value
        self.yawspeed_value_R = -value

    def vel_euler_value_0_changed(self,value):
        self.vel_euler_0 = value
    def vel_euler_value_1_changed(self,value):
        self.vel_euler_1 = value
    def vel_euler_value_2_changed(self,value):
        self.vel_euler_2 = value
    def bodyHeight_value_changed(self, value):
        self.vel_bodyheight = value

    def vel_position_value_0_changed(self,value):
        self.position_0_InputValue = value
    def vel_position_value_1_changed(self,value):
        self.position_1_InputValue = value

#------버튼 클릭 이벤트--------------
    def Click_Front_Btn(self):
        self.Front_btn_pressed_state = True
        self.N_btn.setStyleSheet("background-color: rgb(172, 206, 255);")
        self.myunitree_b1.Move_Front(self.velocity_0_Front_value)
    def Click_Back_Btn(self):
        self.Back_btn_pressed_state = True
        self.S_btn.setStyleSheet("background-color: rgb(172, 206, 255);")
        self.myunitree_b1.Move_Back(self.velocity_0_Back_value)
    def Click_Left_Btn(self):
        self.Left_btn_pressed_state = True
        self.W_btn.setStyleSheet("background-color: rgb(172, 206, 255);")
        self.myunitree_b1.Move_Left(self.velocity_1_Left_value)
    def Click_Right_Btn(self):
        self.Right_btn_pressed_state = True
        self.E_btn.setStyleSheet("background-color: rgb(172, 206, 255);")
        self.myunitree_b1.Move_Right(self.velocity_1_Right_value)
    def Click_Stop_Btn(self):
        self.myunitree_b1.Robot_force_Stop()
    def Click_Turn_L_Btn(self):
        self.Turn_L_btn_pressed_state = True
        self.L_btn.setStyleSheet("background-color: rgb(206, 206, 206);")
        self.myunitree_b1.Turn_Left(self.yawspeed_value_L)
    def Click_Turn_R_Btn(self):
        self.Turn_R_btn_pressed_state = True
        self.R_btn.setStyleSheet("background-color: rgb(206, 206, 206);")
        self.myunitree_b1.Turn_Right(self.yawspeed_value_R)
    def Click_Up_Btn(self):
        self.myunitree_b1.Stand_Up()
    def Click_Down_Btn(self):
        self.myunitree_b1.Stand_Down()
    def Click_Euler_Setting_Btn(self):
        self.myunitree_b1.SetUp_Euler(self.vel_euler_0,self.vel_euler_1,self.vel_euler_2)
    def CLick_Height_Setting_Btn(self):
        self.myunitree_b1.SetUp_Height(self.vel_bodyheight)

    def click_auto_start_Position(self):
        self.AutoMode_flag = True
    def click_auto_end_Position(self):
        self.AutoMode_flag = False
        self.myunitree_b1.Robot_Stop()

    def Release_Front_Btn(self):
        self.Front_btn_pressed_state = False
        self.N_btn.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.myunitree_b1.Robot_Stop()
    def Release_Back_Btn(self):
        self.Back_btn_pressed_state = False
        self.S_btn.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.myunitree_b1.Robot_Stop()
    def Release_Left_Btn(self):
        self.Left_btn_pressed_state = False
        self.W_btn.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.myunitree_b1.Robot_Stop()
    def Release_Right_Btn(self):
        self.Right_btn_pressed_state = False
        self.E_btn.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.myunitree_b1.Robot_Stop()
    def Release_Turn_L_Btn(self):
        self.Turn_L_btn_pressed_state = False
        self.L_btn.setStyleSheet("background:rgb(112, 112, 112);"
                                 "color:rgb(255, 255, 255);")
        self.myunitree_b1.Robot_Stop()
    def Release_Turn_R_Btn(self):
        self.Turn_R_btn_pressed_state = False
        self.R_btn.setStyleSheet("background:rgb(112, 112, 112);"
                                 "color:rgb(255, 255, 255);")
        self.myunitree_b1.Robot_Stop()

#------ 콤보 박스 메소드 --------------
    def Change_mode_combobox(self, index):
        selected_item = self.Mode_ComboBox.currentText()
        print(f"Selected Mode: {selected_item}")

        if selected_item == "IDLE (0)":
            self.myunitree_b1.Change_Mode_to_IDLE()
        elif selected_item == "Force Stand (1)":
            self.myunitree_b1.Change_Mode_to_Force_Stand()
        # elif selected_item == "Vel Walk (2)":
            # self.myunitree_b1.Change_Mode_to_VEL_WALK()
        elif selected_item == "Stand Down (5)":
            self.myunitree_b1.Change_Mode_to_STAND_DOWN()
        elif selected_item == "Stand Up (6)":
            self.myunitree_b1.Change_Mode_to_STAND_UP()

    def Change_gaittype_comboBox(self, index):
        selected_item = self.GaitType_ComboBox.currentText()
        print(f"Selected GaitType: {selected_item}")

        if selected_item == "IDLE (0)":
            self.myunitree_b1.Change_GaitType_to_IDLE()
        elif selected_item == "Trot (1)":
            self.myunitree_b1.Change_GaitType_to_Trot()
        elif selected_item == "Climb Stair (2)":
            self.myunitree_b1.Change_GaitType_to_CLIMB_STAIR()
        elif selected_item == "Trot Obstacle (3)":
            self.myunitree_b1.Change_GaitType_to_TROT_OBSTACLE()
#---------------------------------------------------------------------
    def udp_connect(self):
        try:
            self.myunitree_b1.connect()
            h1 = Tread1(self)
            h1.start()
        except Exception as e:
            print("udp_connect에서 예외 발생:")
            traceback.print_exc()

    def update_label(self):
        self.SOC_label.setText("{:.1f}".format(self.data_SOC))
        self.Mode_label.setText("{:.1f}".format(self.data_mode))
        self.GaitType_label.setText("{:.1f}".format(self.data_gaitType))
        self.State_Position_0_label.setText("{:.1f}".format(self.data_position_hstate[0]))
        self.State_Position_1_label.setText("{:.1f}".format(self.data_position_hstate[1]))
        self.Yawspeed_value_label.setText("{:.01f}".format(self.data_yawspeed))
        self.Heading_value_label.setText("{:.01f}".format(self.heading))

#------ 카메라 관련 메소드 ------------------------------------
    def camera_on(self):

        self.camera_thread = CameraThread(self)
        self.camera_thread.update_image.connect(self.update_camera_view)
        self.camera_thread.start()

    # 이미지에서 선을 감지하고 그려주는 함수
    def detect_lines(self, image):
        gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) # 입력 이미지를 흑백 이미지로 변환
        # 에지 검출을 통해 엣지 이미지 생성
        edges = cv2.Canny(gray_image, threshold1=50, threshold2=150)
        # 허프 변환을 사용하여 선 감지
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=5)

        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2) # 이미지에 감지된 선을 그림

        return image

    # 카메라 뷰를 업데이트하는 슬롯 함수
    @pyqtSlot(QImage)
    def update_camera_view(self, image):
        cv2_image = self.qimage_to_cv(image)  # QImage를 OpenCV 이미지로 변환
        image_with_lines = self.detect_lines(cv2_image) # 선 감지 함수 적용한 이미지 생성
        self.camera_view.setPixmap(QPixmap.fromImage(self.cv_to_qimage(image_with_lines)))  # 카메라 뷰 업데이트

    # QImage를 OpenCV 이미지로 변환하는 함수
    def qimage_to_cv(self, qimage):
        qimage = qimage.convertToFormat(QImage.Format_RGB888)
        width = qimage.width()
        height = qimage.height()
        ptr = qimage.bits()
        ptr.setsize(qimage.byteCount())
        return np.array(ptr).reshape(height, width, 3)

    # OpenCV 이미지를 QImage로 변환하는 함수
    def cv_to_qimage(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        qimage = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        return qimage

#------ Auto Position 메서드---------------------
    def Auto_move_1(self):
        # data_position_hstate: 현재 좌표 [y,x,높이]
        # position_0_InputValue: 지정한 y좌표
        # position_1_InputValue: 지정한 x좌표

        # y좌표 방향 확인
        if self.position_0_InputValue > self.data_position_hstate[0]:
            print(f"현재 위치1-1 :({self.data_position_hstate[0]}, {self.data_position_hstate[1]})")
            # y좌표 좌표 격차 확인
            if abs(self.position_0_InputValue - self.data_position_hstate[0]) > 0.1:
                print("1-1 격차가 0.1 초과")
                self.move_vel_0 = self.velocity_0_Front_value
            elif abs(self.position_0_InputValue - self.data_position_hstate[0]) < 0.1:
                print("1-1 격차가 0.1 미만")
                self.move_vel_0 = 0
        elif self.position_0_InputValue < self.data_position_hstate[0]:
            print(f"현재 위치1-2 :({self.data_position_hstate[0]}, {self.data_position_hstate[1]})")
            # y좌표 좌표 격차 확인
            if abs(self.position_0_InputValue - self.data_position_hstate[0]) > 0.1:
                print("1-2 격차가 0.1 초과")
                self.move_vel_0 = self.velocity_0_Back_value
            elif abs(self.position_0_InputValue - self.data_position_hstate[0]) < 0.1:
                print("1-2 격차가 0.1 미만")
                self.move_vel_0 = 0

        # x좌표 방향 확인
        # left(+)/right(-)
        if self.position_1_InputValue > self.data_position_hstate[1]:
            print(f"현재 위치2-1 :({self.data_position_hstate[0]}, {self.data_position_hstate[1]})")
            # x좌표 좌표 격차 확인
            if abs(self.position_1_InputValue - self.data_position_hstate[1]) > 0.1:
                print("2-1 격차가 0.1 초과")
                self.move_vel_1 = self.velocity_1_Left_value
            elif abs(self.position_1_InputValue - self.data_position_hstate[1]) <= 0.1:
                print("2-1 격차가 0.1 미만")
                self.move_vel_1 = 0
        elif self.position_1_InputValue < self.data_position_hstate[1]:
            print(f"현재 위치2-2 :({self.data_position_hstate[0]}, {self.data_position_hstate[1]})")
            # x좌표 좌표 격차 확인
            if abs(self.position_1_InputValue - self.data_position_hstate[1]) > 0.1:
                print("2-2 격차가 0.1 초과")
                self.move_vel_1 = self.velocity_1_Right_value
            elif abs(self.position_1_InputValue - self.data_position_hstate[1]) < 0.1:
                print("2-2 격차가 0.1 미만")
                self.move_vel_1 = 0

        # 좌표 지정 -> 이동
        self.myunitree_b1.Move_mult(self.move_vel_0, self.move_vel_1)

        # 좌표이동 완료
        if abs(self.position_1_InputValue - self.data_position_hstate[1]) < 0.1 and abs(self.position_0_InputValue - self.data_position_hstate[0]) < 0.1:
            print("좌표 지정 완료")
            self.click_auto_end_Position()

    def Auto_move_2(self):
        # data_position_hstate: 현재 좌표 [y,x,높이]
        # position_0_InputValue: 지정한 y좌표
        # position_1_InputValue: 지정한 x좌표

        f = open("datalog.csv","a")

        # 내가 어디를 바라보고 있는가
        # self.atan2_value_radian = 2*(math.atan2(self.view_data_quaternion[0],self.view_data_quaternion[3]))
        # self.atan2_value_degree = math.degrees(self.atan2_value_radian)

        # self.rotation_angle = math.radians(-90 + self.atan2_value_degree)
        self.rotation_angle = -math.radians(self.heading)

        # 타겟 좌표
        self.target_x = (self.position_1_InputValue) - (self.data_position_hstate[1])
        self.target_y = (self.position_0_InputValue) - (self.data_position_hstate[0])

        # 축회전: 절대 -> 상대
        self.target_x_t = (self.target_x)*(math.cos(self.rotation_angle))+(self.target_y) * (math.sin(self.rotation_angle))
        self.target_y_t = -(self.target_x)*(math.sin(self.rotation_angle))+(self.target_y) * (math.cos(self.rotation_angle))


        # y좌표 방향 확인
        if abs(self.target_y_t - 0) > 0.2:
            if self.target_y_t > 0:
                self.move_vel_0 = self.velocity_0_Front_value
            else:
                self.move_vel_0 = self.velocity_0_Back_value
        elif abs(self.target_y_t - 0) < 0.2:
            self.move_vel_0 = 0

        # x좌표 방향 확인
        # left(+)/right(-)
        if abs(self.target_x_t - 0) > 0.2:
            if self.target_x_t > 0:
                self.move_vel_1 = self.velocity_1_Left_value
            else:
                self.move_vel_1 = self.velocity_1_Right_value
        elif abs(self.target_x_t - 0) <= 0.2:
            self.move_vel_1 = 0

        data = "%f,%f,%f,%f,%f,%f,%f,%f,%f\n" % (self.data_position_hstate[1], self.data_position_hstate[0],self.rotation_angle,self.target_x,self.target_y,self.target_x_t,self.target_y_t,self.move_vel_0,self.move_vel_1)
        f.write(data)

        # 좌표 지정 -> 이동
        self.myunitree_b1.Move_mult(self.move_vel_0, self.move_vel_1)

        # 좌표이동 완료
        if abs(self.target_x_t - 0) < 0.2 and abs(self.target_y_t - 0) < 0.2:
            # print("좌표 지정 완료")
            self.click_auto_end_Position()
        # 좌표이동 완료
        if abs(self.position_1_InputValue - self.data_position_hstate[1]) < 0.2 and abs(self.position_0_InputValue - self.data_position_hstate[0]) < 0.2:
            # print("좌표 지정 완료")
            self.click_auto_end_Position()

        f.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    app.exec_()