import time
from ucl.common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from ucl.highCmd import highCmd
from ucl.highState_b1 import highState
from ucl.unitreeConnection import unitreeConnection, HIGH_WIFI_DEFAULTS, HIGH_WIRED_DEFAULTS
from ucl.enums import MotorModeHigh, GaitType
from ucl.complex import motorCmd

class myunitree:
    def __init__(self):
        i = 0
        self.connect_flag = False
    def connect(self):
        self.conn = unitreeConnection(HIGH_WIFI_DEFAULTS)  # 네트워크 연결
        self.conn.startRecv()

        self.hcmd = highCmd()
        self.hstate = highState()
        self.connect_flag = True

    def cmdInit(self):
        time.sleep(0.01)
        data = self.conn.getData()
        for paket in data:
            self.hstate.parseData(paket)

            # self.highstate_info = f"bodyHeight:\t\t{self.hstate.bodyHeight}\n"

            self.highstate_info = f"{self.hstate.imu.quaternion},{self.hstate.position},{self.hstate.yawSpeed}\n"

            self.hstate_bodyHeight = self.hstate.bodyHeight
            self.hstate_bms_SOC = self.hstate.bms.SOC
            self.hstate_footforce = self.hstate.footForce
            self.hstate_mode =self.hstate.mode
            self.hstate_gaitType =self.hstate.gaitType
            self.hstate_position = self.hstate.position
            self.hstate_yawspeed = self.hstate.yawSpeed

            self.hstate_rpy = [self.hstate.imu.rpy[0],self.hstate.imu.rpy[1],self.hstate.imu.rpy[2]]
            self.hstate_quaternion =[self.hstate.imu.quaternion[0],self.hstate.imu.quaternion[1],self.hstate.imu.quaternion[2],self.hstate.imu.quaternion[3]]
            self.hstate_motorQ = []
            for i in range(20):
                self.hstate_motorQ.append(self.hstate.motorstate[i].q)

    def sendCmd(self):
        self.cmd_bytes = self.hcmd.buildCmd(debug=False)
        self.conn.send(self.cmd_bytes)
        self.cmdInit()
        # print(self.hcmd.mode)
        # print(self.hcmd.gaitType)
    #------ 뱡향키 입력 메소드 ---------------------------------
    def Move_Front(self,velocity_0):
        self.hcmd.mode = MotorModeHigh.VEL_WALK # mode 2
        self.hcmd.velocity = [velocity_0, 0]  # -1  ~ +1
    def Move_Back(self,velocity_0):
        self.hcmd.mode = MotorModeHigh.VEL_WALK  # mode 2
        self.hcmd.velocity = [velocity_0, 0]  # -1  ~ +1
    def Move_Left(self,velocity_1):
        self.hcmd.mode = MotorModeHigh.VEL_WALK  # mode 2
        self.hcmd.velocity = [0, velocity_1]  # -1  ~ +1
    def Move_Right(self,velocity_1):
        self.hcmd.mode = MotorModeHigh.VEL_WALK  # mode 2
        self.hcmd.velocity = [0, velocity_1]  # -1  ~ +1
    def Robot_Stop(self):
        self.hcmd.mode = MotorModeHigh.FORCE_STAND
        self.hcmd.velocity = [0,0]  # -1  ~ +1
        self.hcmd.yawSpeed = 0
    def Turn_Left(self,yawspeed_value):
        self.hcmd.mode = MotorModeHigh.VEL_WALK
        self.hcmd.yawSpeed = yawspeed_value
    def Turn_Right(self,yawspeed_value):
        self.hcmd.mode = MotorModeHigh.VEL_WALK
        self.hcmd.yawSpeed = yawspeed_value
    def Robot_force_Stop(self):
        self.hcmd.mode = MotorModeHigh.IDLE
        self.hcmd.gaitType = GaitType.IDLE
        self.hcmd.velocity = [0,0]  # -1  ~ +1
        self.hcmd.yawSpeed = 0
        self.hcmd.euler = [0, 0, 0]
        print("강제 STOP")

    def Move_mult(self,velocity_0,velocity_1):
        self.cmdInit()
        self.hcmd.mode = MotorModeHigh.VEL_WALK
        self.hcmd.velocity = [velocity_0,velocity_1]
    # -------------------------------------------------------
    def Stand_Up(self):
        self.hcmd.mode = MotorModeHigh.STAND_UP
    def Stand_Down(self):
        self.hcmd.mode = MotorModeHigh.STAND_DOWN

    def SetUp_Euler(self,row_value,pitch_value,yaw_value): #self,row_value,pitch_value,yaw_value
        self.cmdInit()

        # euler = [Roll, Pitch, Yaw]
        # Row: (+)왼쪽, (-)오른쪽
        # Pitch: (+)고개 숙이기 ,(-)고개 들기
        # Yaw: (+)왼쪽, (-)오른쪽
        self.hcmd.euler = [row_value,pitch_value,yaw_value]
    def SetUp_Height(self,bodyHeight_value):
        self.cmdInit()
        # self.hcmd.mode = MotorModeHigh.FORCE_STAND
        self.hcmd.bodyHeight = bodyHeight_value # default: 0.28m

    # ------ Mode ComboBox 입력 메소드 --------------------
    def Change_Mode_to_IDLE(self):
        self.cmdInit()
        self.hcmd.mode = MotorModeHigh.IDLE # mode  0
    def Change_Mode_to_Force_Stand(self):
        self.cmdInit()
        self.hcmd.mode = MotorModeHigh.FORCE_STAND # mode  1
    def Change_Mode_to_VEL_WALK(self):
        self.cmdInit()
        self.hcmd.mode = MotorModeHigh.VEL_WALK # mode 2
    def Change_Mode_to_STAND_DOWN(self):
        self.cmdInit()
        self.hcmd.mode = MotorModeHigh.STAND_DOWN # mode 5
    def Change_Mode_to_STAND_UP(self):
        self.cmdInit()
        self.hcmd.mode = MotorModeHigh.STAND_UP # mode 6

    #------ GaitType ComboBox 입력 메소드 --------------------
    def Change_GaitType_to_IDLE(self):
        self.cmdInit()
        self.hcmd.gaitType = GaitType.IDLE  # GaitType 0
    def Change_GaitType_to_Trot(self):
        self.cmdInit()
        self.hcmd.gaitType = GaitType.TROT  # GaitType 1
    def Change_GaitType_to_CLIMB_STAIR(self):
        self.cmdInit()
        self.hcmd.gaitType = GaitType.CLIMB_STAIR  # GaitType 2
    def Change_GaitType_to_TROT_OBSTACLE(self):
        self.cmdInit()
        self.hcmd.gaitType = GaitType.TROT_OBSTACLE  # GaitType 3
