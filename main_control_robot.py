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
import pyqtgraph as pg
from myunitree_robot import myunitree
from View3DDialog import View3DDialog

class Tread1(QThread):
    def __init__(self, parent):
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
        uic.loadUi(r'./sample.ui', self)  # Ui 연결
        self.myunitree_b1 = myunitree()  # myunitree  class 불러와서 명명
        # ----- 변수 초기화 ------------------------------------------
        self.velocity_0_Front_value = 0
        self.velocity_0_Back_value = 0
        self.velocity_1_Left_value = 0
        self.velocity_1_Right_value = 0
        self.yawspeed_value_L = 0
        self.yawspeed_value_R = 0
        self.move_vel_0 = 0
        self.move_vel_1 = 0
        self.AutoMode_flag = False

        # ------ Dialog ----------------------------------------------------
        self.actionGraph = QAction("Open Graph", self)
        self.actionPositionGraph = QAction("Position View", self)
        self.view_robot_3D = QAction("3D View", self)
        self.view_robot_3D.triggered.connect(self.open_view_robot_3D)

        self.fileMenu = self.menuBar().addMenu("Graph")
        self.fileMenu.addAction(self.actionGraph)
        self.fileMenu.addAction(self.actionPositionGraph)
        self.fileMenu.addAction(self.view_robot_3D)

        # ------ 버튼 -----------------------------------------------------
        self.connect_btn.clicked.connect(self.udp_connect)      # 통신 연결 버튼
        self.disconnect_btn.clicked.connect(self.udp_disconnect)
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

        # self.auto_start_position_btn.pressed.connect(self.click_auto_start_Position)
        # self.auto_end_position_btn.pressed.connect(self.click_auto_end_Position)

        self.auto_target_start_position_btn.pressed.connect(self.click_auto_start_Position)
        self.auto_stop_position_btn.pressed.connect(self.click_auto_end_Position)

        self.Front_btn_pressed_state = False
        self.Back_btn_pressed_state = False
        self.Left_btn_pressed_state = False
        self.Right_btn_pressed_state = False
        self.Turn_L_btn_pressed_state = False
        self.Turn_R_btn_pressed_state = False
        # ------ 값 입력 ----------------------------------------------------
        self.input_vel_0.valueChanged.connect(self.vel_0_value_changed)
        self.input_vel_1.valueChanged.connect(self.vel_1_value_changed)

        self.input_position_0.valueChanged.connect(self.vel_position_value_0_changed)
        self.input_position_1.valueChanged.connect(self.vel_position_value_1_changed)

        # ------ Label -----------------------------------------------------
        self.SOC_label = self.findChild(QLabel, "SOC_label")
        self.Mode_label = self.findChild(QLabel, "mode_label")
        self.GaitType_label = self.findChild(QLabel, "gaittype_label")
        self.State_Position_0_label = self.findChild(QLabel, "state_position_0_label")
        self.State_Position_1_label = self.findChild(QLabel, "state_position_1_label")
        self.State_Connect_label = self.findChild(QLabel, "state_connect_label")
        self.BQ_NTC_label = self.findChild(QLabel, "BQ_NTC_label")
        self.MCU_NTC_label = self.findChild(QLabel, "MCU_NTC_label")
        # ------ ComboBox ---------------------------------------------------
        self.Mode_ComboBox = self.findChild(QComboBox, "mode_comboBox")
        self.Mode_ComboBox.currentIndexChanged.connect(self.Change_mode_combobox)
        self.GaitType_ComboBox = self.findChild(QComboBox, "gaittype_comboBox")
        self.GaitType_ComboBox.currentIndexChanged.connect(self.Change_gaittype_comboBox)

        self.time_data = []
        self.y_data = []
        self.x_data = []
        self.position_state_graph.plotItem.showGrid(True, True, 1) # 그리드 표시 설정
        self.position_state_graph.plotItem.setRange(xRange=(-5, 5), yRange=(-5, 5)) # 좌표 범위 설정
        self.position_state_graph.plotItem.setPos(0, 0) # 그래프 위치 설정
        self.plot_widget_position_dot = self.findChild(pg.PlotWidget, "position_state_graph")
        # ----------------------XY graph-----------------------
        # self.setupUi(self)
        self.position_view.plotItem.showGrid(True, True, 1) # 그리드 표시 설정
        self.position_view.plotItem.setRange(xRange=(-5, 5), yRange=(-5, 5)) # 좌표 범위 설정
        self.position_view.plotItem.setPos(0, 0) # 그래프 위치 설정
        self.scatter_item = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(255, 0, 0)) # 산점도 아이템 생성
        self.position_view.addItem(self.scatter_item) # 산점도 아이템을 그래프에 추가
        # ------------마우스클릭 이벤트---------------------------
        self.position_view.scene().sigMouseClicked.connect(self.mouse_clicked)  # 마우스 클릭 이벤트 연결
        self.circle_item = pg.ScatterPlotItem(size=10, brush='r', color='red')  # 동그라미 원을 표시할 아이템 생성
        self.position_view.addItem(self.circle_item)  # 아이템을 그래프 위젯에 추가

    # ------ Dialog Window 띄우기 ----------------------
    def open_view_robot_3D(self):
        self.view_window = View3DDialog(self)
        self.view_window.show()

    # ------ SendCmd -------------------------------------
    def sendCmd(self):
        self.myunitree_b1.sendCmd()

        self.highstate_textBrowser.append(self.myunitree_b1.highstate_info)

        self.data_SOC = self.myunitree_b1.hstate_bms_SOC
        self.data_mode = self.myunitree_b1.hstate_mode
        self.data_gaitType = self.myunitree_b1.hstate_gaitType
        self.data_yawspeed = self.myunitree_b1.hstate_yawspeed
        self.data_BQ_NTC = self.myunitree_b1.hstate_bms_BQ_NTC
        self.data_MCU_NTC = self.myunitree_b1.hstate_bms_MCU_NTC

        self.plot_data_bodyHeight = self.myunitree_b1.hstate_bodyHeight
        self.plot_data_footforce = self.myunitree_b1.hstate_footforce
        self.data_position_hstate = self.myunitree_b1.hstate_position

        self.view_data_rpy = self.myunitree_b1.hstate_rpy
        self.view_data_motorQ = self.myunitree_b1.hstate_motorQ
        self.view_data_quaternion = self.myunitree_b1.hstate_quaternion

        # 시계방향으로 증가시키면서 조합을 찾은 heading 값
        self.heading = math.degrees(2 * (math.atan2(-self.view_data_quaternion[3], self.view_data_quaternion[0])))
        if self.heading < 0:
            self.heading = self.heading + 360
        if self.heading > 360:
            self.heading = self.heading - 360

        self.update_label()

        # Auto 모드
        if self.AutoMode_flag == True:
            self.Auto_target_position()

    # ------데이터 입력 이벤트------------
    def vel_0_value_changed(self, value):
        self.velocity_0_Front_value = value
        self.velocity_0_Back_value = -value

    def vel_1_value_changed(self, value):
        self.velocity_1_Left_value = value
        self.velocity_1_Right_value = -value

    def yawspeed_value_changed(self, value):
        self.yawspeed_value_L = value
        self.yawspeed_value_R = -value

    def vel_position_value_0_changed(self, value):
        self.position_0_InputValue = value

    def vel_position_value_1_changed(self, value):
        self.position_1_InputValue = value

    # ------버튼 클릭 이벤트--------------
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

    # ----------------------지정된 좌표 출력-----------------
    def mouse_clicked(self, event):
        pos = self.position_view.plotItem.vb.mapSceneToView(event.scenePos()) # 마우스 이벤트가 발생한 위치를 씬 좌표로 가져옴

        self.target_position_1_value = -pos.x() # x 좌표
        self.target_position_0_value = pos.y() # y 좌표

        self.circle_item.setData([-self.target_position_1_value], [self.target_position_0_value])  # 동그라미 원의 위치 설정

        self.position_target_1_label.setText(f'x: {-self.target_position_1_value:.2f}, ')  # 좌표값 출력
        self.position_target_0_label.setText(f'y: {self.target_position_0_value:.2f}')  # 좌표값 출력

    # ------ 콤보 박스 메소드 --------------
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

    # ---------------------------------------------------------------------
    def udp_connect(self):
        try:
            self.myunitree_b1.connect()
            h1 = Tread1(self)
            h1.start()
            self.plot_timer = QTimer(self)
            self.plot_timer.timeout.connect(self.update_position_state_plot)
            self.plot_timer.start(200)
        except Exception as e:
            print("udp_connect에서 예외 발생:")
            traceback.print_exc()
    def udp_disconnect(self):
        try:
            self.myunitree_b1.disconnect()
            h1 = Tread1(self)
            h1.start()
        except Exception as e:
            print("udp_disconnect에서 예외 발생:")
            traceback.print_exc()

    def update_label(self):
        self.SOC_label.setText("{:.1f}".format(self.data_SOC))
        self.Mode_label.setText("{:.1f}".format(self.data_mode))
        self.GaitType_label.setText("{:.1f}".format(self.data_gaitType))
        self.State_Position_0_label.setText("{:.1f}".format(self.data_position_hstate[0]))
        self.State_Position_1_label.setText("{:.1f}".format(-self.data_position_hstate[1]))
        self.BQ_NTC_label.setText("{:.1f}".format(self.data_BQ_NTC[0])) # 8.0
        self.MCU_NTC_label.setText("{:.1f}".format(self.data_MCU_NTC[0])) # 12.0

        if self.myunitree_b1.connect_flag:
            self.State_Connect_label.setText("Connect")
            self.State_Connect_label.setStyleSheet("color: blue;")
        else:
            self.State_Connect_label.setText("Disconnect")
            self.State_Connect_label.setStyleSheet("color: red;")

    # ------ 카메라 관련 메소드 ------------------------------------
    def camera_on(self):

        self.camera_thread = CameraThread(self)
        self.camera_thread.update_image.connect(self.update_camera_view)
        self.camera_thread.start()

    # 이미지에서 선을 감지하고 그려주는 함수
    def detect_lines(self, image):
        gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)  # 입력 이미지를 흑백 이미지로 변환
        # 에지 검출을 통해 엣지 이미지 생성
        edges = cv2.Canny(gray_image, threshold1=50, threshold2=150)
        # 허프 변환을 사용하여 선 감지
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=5)

        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 이미지에 감지된 선을 그림

        return image

    # 카메라 뷰를 업데이트하는 슬롯 함수
    @pyqtSlot(QImage)
    def update_camera_view(self, image):
        cv2_image = self.qimage_to_cv(image)  # QImage를 OpenCV 이미지로 변환
        image_with_lines = self.detect_lines(cv2_image)  # 선 감지 함수 적용한 이미지 생성
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

    def graph_PositionGraph_test(self, x_values, y_values):
        # x 값을 양수면 음수로, 음수면 양수로 바꾸기
        x_values = [-x if x > 0 else abs(x) for x in x_values]
        self.plot_point = self.plot_widget_position_dot.plot(x_values, y_values, pen=None, symbol='o')

        self.plot_widget_position_dot.setXRange(-5, 5)
        self.plot_widget_position_dot.setYRange(-5, 5)

    def update_position_state_plot(self):
        self.dialog_data_position = self.data_position_hstate
        if self.dialog_data_position:
            y_values = self.dialog_data_position
            x_values = range(1, len(y_values) + 1)

            y_values_2 = [self.dialog_data_position[0]]
            x_values_2 = [self.dialog_data_position[1]]

            self.graph_PositionGraph_test(x_values_2, y_values_2)

    # ------ Auto Position 메서드---------------------
    def Auto_move_2(self):
        # data_position_hstate: 현재 좌표 [y,x,높이]
        # position_0_InputValue: 지정한 y좌표
        # position_1_InputValue: 지정한 x좌표

        f = open("datalog.csv", "a")

        # 내가 어디를 바라보고 있는가
        # self.atan2_value_radian = 2*(math.atan2(self.view_data_quaternion[0],self.view_data_quaternion[3]))
        # self.atan2_value_degree = math.degrees(self.atan2_value_radian)

        # self.rotation_angle = math.radians(-90 + self.atan2_value_degree)
        self.rotation_angle = -math.radians(self.heading)

        # 타겟 좌표
        self.target_x = (self.position_1_InputValue) - (self.data_position_hstate[1])
        self.target_y = (self.position_0_InputValue) - (self.data_position_hstate[0])

        # 축회전: 절대 -> 상대
        self.target_x_t = (self.target_x) * (math.cos(self.rotation_angle)) + (self.target_y) * (
            math.sin(self.rotation_angle))
        self.target_y_t = -(self.target_x) * (math.sin(self.rotation_angle)) + (self.target_y) * (
            math.cos(self.rotation_angle))

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

        data = "%f,%f,%f,%f,%f,%f,%f,%f,%f\n" % (
        self.data_position_hstate[1], self.data_position_hstate[0], self.rotation_angle, self.target_x, self.target_y,
        self.target_x_t, self.target_y_t, self.move_vel_0, self.move_vel_1)
        f.write(data)

        # 좌표 지정 -> 이동
        self.myunitree_b1.Move_mult(self.move_vel_0, self.move_vel_1)

        # 좌표이동 완료
        if abs(self.target_x_t - 0) < 0.2 and abs(self.target_y_t - 0) < 0.2:
            # print("좌표 지정 완료")
            self.click_auto_end_Position()
        # 좌표이동 완료
        if abs(self.position_1_InputValue - self.data_position_hstate[1]) < 0.2 and abs(
                self.position_0_InputValue - self.data_position_hstate[0]) < 0.2:
            # print("좌표 지정 완료")
            self.click_auto_end_Position()

        f.close()

    def Auto_target_position(self):
        # data_position_hstate: 현재 좌표 [y,x,높이]
        # position_0_InputValue: 지정한 y좌표
        # position_1_InputValue: 지정한 x좌표

        f = open("datalog_target.csv", "a")

        # 내가 어디를 바라보고 있는가
        # self.atan2_value_radian = 2*(math.atan2(self.view_data_quaternion[0],self.view_data_quaternion[3]))
        # self.atan2_value_degree = math.degrees(self.atan2_value_radian)

        # self.rotation_angle = math.radians(-90 + self.atan2_value_degree)
        self.rotation_angle = -math.radians(self.heading)

        # 타겟 좌표
        self.target_x = (self.target_position_1_value) - (self.data_position_hstate[1])
        self.target_y = (self.target_position_0_value) - (self.data_position_hstate[0])

        # 축회전: 절대 -> 상대
        self.target_x_t = (self.target_x) * (math.cos(self.rotation_angle)) + (self.target_y) * (
            math.sin(self.rotation_angle))
        self.target_y_t = -(self.target_x) * (math.sin(self.rotation_angle)) + (self.target_y) * (
            math.cos(self.rotation_angle))

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

        data = "%f,%f,%f,%f,%f,%f,%f,%f,%f\n" % (
        self.data_position_hstate[1], self.data_position_hstate[0], self.rotation_angle, self.target_x, self.target_y,
        self.target_x_t, self.target_y_t, self.move_vel_0, self.move_vel_1)
        f.write(data)

        # 좌표 지정 -> 이동
        self.myunitree_b1.Move_mult(self.move_vel_0, self.move_vel_1)

        # 좌표이동 완료
        if abs(self.target_x_t - 0) < 0.2 and abs(self.target_y_t - 0) < 0.2:
            # print("좌표 지정 완료")
            self.click_auto_end_Position()
        # 좌표이동 완료
        if abs(self.target_position_1_value - self.data_position_hstate[1]) < 0.2 and abs(
                self.target_position_0_value - self.data_position_hstate[0]) < 0.2:
            # print("좌표 지정 완료")
            self.click_auto_end_Position()

        f.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    app.exec_()