import threading
from .dobot_api_v3 import DobotApiDashboard, DobotApiMove, DobotApi, alarmAlarmJsonFile, MyType
from time import sleep
import re
import numpy as np
from .gripper1 import Gripper1

current_actual = [-1]
algorithm_queue = -1
enableStatus_robot = -1
robotErrorState = False
globalLockValue = threading.Lock()

r = [-177.583, -2, 175.047]

class DobotCR5ApiV3:
    def __init__(self, ip: str="192.168.5.1") -> None:
        self.ip: str = ip
        self.dashboardPort: int = 29999
        self.feedPort: int = 30004
        self.movePort: int = 30005
        
        self.is_connected: bool = False
        self.exit_signal: bool = False

        # เชื่อมต่อ Robot
        dashboard, move, feed = self.ConnectRobot()
        
        self.dashboard: DobotApiDashboard = dashboard
        self.move: DobotApiMove = move
        self.feed: DobotApi = feed
        
        self.gripper = Gripper1(dashboard)
        
        dashboard.SpeedL(10)
        
        feed_thread = threading.Thread(target=self.GetFeed, args=(feed,))
        feed_thread.daemon = True
        feed_thread.start()
        feed_thread1 = threading.Thread(target=self.ClearRobotError, args=(dashboard,))
        feed_thread1.daemon = True
        feed_thread1.start()
        self.dashboard.EnableRobot()
        print("Staring...")
        print("Starting successed:)")
        print("Looping...")
        self.initial()
        
    def initial(self):
        self.gripper.grip_open()
        init_point = [-145.621, -290.98, 539.75] + r
        self.RunToPoint(init_point, 0)
        
    def ConnectRobot(self):
        try:
            ip = self.ip
            dashboardPort = 29999
            movePort = 30003
            feedPort = 30004
            print("Connecting...")
            dashboard = DobotApiDashboard(ip, dashboardPort)
            move = DobotApiMove(ip, movePort)
            feed = DobotApi(ip, feedPort)
            print(">.<Connecting successed>!<")
            return dashboard, move, feed
        except Exception as e:
            print(":(Connecting fail:(")
            raise e

    def RunToPoint(self, point_list: list[float], coordinateMode: int):
        point_list = point_list + r
        if point_list[2] < 150 or point_list[2] > 600:
            print("*" * 20 + "\tPosition out of bound...!!!\t" + "*" * 20)
            return
        while True:
            p2id = self.RunPoint(self.move, point_list)        
            if p2id[0] == 0: # ไม่มี error (ถ้ามี error จะทำการส่งคำสั่งใหม่เรื่อยๆ)
                self.WaitArrive(point_list) # รอให้แขกลถึงตำแหน่ง
                break

    def RunPoint(self, move: DobotApiMove, point_list: list[float]):
        recv_movemess = move.MovL(point_list[0], point_list[1], point_list[2], point_list[3], point_list[4], point_list[5]) # P = [x, y, z, rx, ry, rz]
        commandArrID = self.parseResultId(recv_movemess)
        return commandArrID

    def parseResultId(self, valueRecv):
        if valueRecv.find("Not Tcp") != -1:
            print("Control Mode Is Not Tcp")
            return [1]
        recvData = re.findall(r'-?\d+', valueRecv)
        recvData = [int(num) for num in recvData] # ! ตำแหน่ง
        if len(recvData) == 0:
            return [2]
        return recvData
    
    def WaitArrive(self, point_list):
        while True:
            is_arrive = True
            globalLockValue.acquire()
            if current_actual is not None:
                for index in range(4):
                    if (abs(current_actual[index] - point_list[index]) > 1):
                        is_arrive = False
                if is_arrive:
                    globalLockValue.release()
                    return
            globalLockValue.release()
            sleep(0.001)
            
    def GetFeed(self, feed: DobotApi):
        global current_actual
        global algorithm_queue
        global enableStatus_robot
        global robotErrorState
        hasRead = 0
        while True:
            data = bytes()
            while hasRead < 1440:
                temp = feed.socket_dobot.recv(1440 - hasRead)
                if len(temp) > 0:
                    hasRead += len(temp)
                    data += temp
            hasRead = 0
            feedInfo = np.frombuffer(data, dtype=MyType)
            if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
                globalLockValue.acquire()
                # Refresh Properties
                current_actual = feedInfo["tool_vector_actual"][0]
                algorithm_queue = feedInfo['run_queued_cmd'][0]
                enableStatus_robot = feedInfo['enable_status'][0]
                robotErrorState = feedInfo['error_status'][0]
                globalLockValue.release()
            sleep(0.001)

    def ClearRobotError(self, dashboard: DobotApiDashboard):
        global robotErrorState
        dataController, dataServo = alarmAlarmJsonFile()    # 读取控制器和伺服告警码
        while True:
            globalLockValue.acquire()
            if robotErrorState:
                numbers = re.findall(r'-?\d+', dashboard.GetErrorID())
                numbers = [int(num) for num in numbers]
                if (numbers[0] == 0):
                    if (len(numbers) > 1):
                        for i in numbers[1:]:
                            alarmState = False
                            if i == -2:
                                print("Machine alarm machine collision! ", i)
                                alarmState = True
                            if alarmState:
                                continue
                            for item in dataController:
                                if i == item["id"]:
                                    print("Machine alarm Controller errorid", i,
                                        item["zh_CN"]["description"])
                                    alarmState = True
                                    break
                            if alarmState:
                                continue
                            for item in dataServo:
                                if i == item["id"]:
                                    print("Machane alarm Servo errorid", i,
                                        item["zh_CN"]["description"])
                                    break

                        choose = input("Enter: 1, clear error, continue machine: ")
                        if int(choose) == 1:
                            dashboard.ClearError()
                            sleep(0.01)
                            dashboard.Continue()

            else:
                if int(enableStatus_robot) == 1 and int(algorithm_queue) == 0:
                    dashboard.Continue()
            globalLockValue.release()
            sleep(5)

    def test_run_to(self):
        point_lists = [
            [70, -300, 500] + r,
            [-200, -300, 500] + r,
        ]
        for i in range(3):
            for p in point_lists:
                self.RunToPoint(p, 1)
        self.dashboard.DisableRobot()