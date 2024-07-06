# from pipe import DataFlow
# from vision.detector import Detector
from time import sleep
from threading import Thread
from typing import Tuple
# from .dobot_api_v4 import DobotApiDashboard, DobotApiFeedBack, DobotApiDashMove
from dobot_api_v3 import DobotApiDashboard, DobotApiMove, DobotApi, alarmAlarmJsonFile, MyType
from threading import Thread
import re
import threading
import numpy as np

current_actual = [-1]
algorithm_queue = -1
enableStatus_robot = -1
robotErrorState = False
globalLockValue = threading.Lock()

r = [-177, 1, -165]

# class RobotControl(Thread):
#     def __init__(self, detector: Detector, data_flow: DataFlow, cam_position: tuple[float, float, float], ip: str = '192.168.5.1') -> None:
#         super().__init__()
#         self.name: str = "Robot Thread"
        
#         self.data_flow: DataFlow = data_flow
#         self.detector: Detector = detector
#         self.exit_signal: bool = False
#         self.cam_position: tuple[float, float, float] = cam_position
        
#         self.class_names = detector.model.names

#         # self.dobot = DobotCR5(ip)
#         # if not self.dobot.is_connected:
#         #     self.exit_signal = True
#         # self.dobot.Enable()
            
#         self.drop_point: dict[str, float] = {
#                                                 "red-cube": [0.0, 0.0, 0.0],
#                                                 "green-cube": [0.0, 0.0, 0.0],
#                                                 "blue-cube": [0.0, 0.0, 0.0],
#                                                 "yellow-cube": [0.0, 0.0, 0.0]
#                                             }
        
#         self.r: list[float] = [0.0, 0.0, 0.0]
#         # self.simple_drop_point = [0.0, 0.0, 200]

#     def stop(self):
#         self.exit_signal = True

#     def run(self):
#         print("-"*20)
#         while not self.exit_signal:
#             if self.data_flow.obj is not None:
#                 obj = self.data_flow.obj
                
#                 self.simulate(obj)
                
#                 # self.run_to_object(obj)
#                 self.hold()
#                 # self.run_to_drop(obj)
#                 self.release()
#             else:
#                 print("Waiting for Objects...")
#                 sleep(5)
                

#     def simulate(self, obj):
#         self.display(obj)
#         sleep(5)
    
#     def display(self, obj):
#         print('-' * 20)
#         string = f"Robot get: {obj.id}\nClass: {self.class_names[obj.raw_label]}\nPosition X: {obj.position[0]}, Y: {obj.position[2]}, Z: {obj.position[1]}"
#         print(string)
#         print('-' * 20)

#     def run_to_object(self, obj):
#         point_list: list[float] = self.cal_pisition_relation_cam_robot(obj) # [x, y, z, rx, ry, rz]
#         self.display(obj)
#         self.dobot.RunToPoint(point_list, 0)

#     def run_to_drop(self, obj):
#         class_name: str | None = self.class_names[obj.raw_label]
#         point_list: list[float] = self.drop_point[class_name] + self.r
#         self.dobot.RunToPoint(point_list, 0)

#     def hold(self):
#         pass
    
#     def release(self):
#         pass
    
#     def cal_pisition_relation_cam_robot(self, obj) -> list[float]:
#         point_list: list[float] = [obj.potition[0] - self.cam_position[0], obj.position[2] - self.cam_position[1], obj.position[1] - self.cam_position[2]] + self.r
#         return point_list

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
        while True:
            p2id = self.RunPoint(self.move, point_list, coordinateMode)        
            if p2id[0] == 0: # ไม่มี error (ถ้ามี error จะทำการส่งคำสั่งใหม่เรื่อยๆ)
                self.WaitArrive(point_list) # รอให้แขกลถึงตำแหน่ง
                break

    def RunPoint(self, move: DobotApiMove, point_list: list[float], coordinateMode: int):
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
            
    def test_run_to(self):
        point_lists = [
            [70, -300, 500] + r,
            [-200, -300, 500] + r,
        ]
        for i in range(3):
            for p in point_lists:
                self.RunToPoint(p, 1)
        self.dashboard.DisableRobot()

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
    
# class DobotCR5ApiV4:
#     def __init__(self, ip: str) -> None:
#         self.ip: str = ip
#         self.dashboardPort: int = 29999
#         self.feedPort: int = 30004
#         self.movePort: int = 30005
        
#         self.is_connected: bool = False
#         self.exit_signal: bool = False

#         # เชื่อมต่อ Robot
#         self.dashboard: DobotApiDashboard | None = None
#         self.feed: DobotApiFeedBack | None = None
#         self.move: DobotApiDashMove | None = None

#         self.t_clearError = Thread(target=self.ClearRobotError)

#     def Connect(self):
#         self.dashboard = DobotApiDashboard(self.ip, self.dashboardPort)
#         self.feed = DobotApiFeedBack(self.ip, self.feedPort)
#         self.move = DobotApiDashMove(self.ip, self.movePort)
#         self.is_connected = True
#         self.t_clearError.start()
#         print("Dobot connection error.")
        
#     def Enable(self):
#         enableState = self.parseResultId(self.dashboard.EnableRobot())
#         if enableState[0] != 0:
#             print("Failed to enable: Check whether port 29999 is occupied.")
#             return
#         print("Enabled successfully!")

#     def RunToPoint(self, point_list: list[float], coordinateMode: int):
#         while True:
#             p2id = self.RunPoint(point_list, coordinateMode)        
#             if p2id[0] == 0: # ไม่มี error (ถ้ามี error จะทำการส่งคำสั่งใหม่เรื่อยๆ)
#                 self.move.WaitArrive(p2id[1]) # รอให้แขกลถึงตำแหน่ง
#                 break

#     def RunPoint(self, point_list: list[float], coordinateMode: int):
#         recv_movemess = self.dashboard.MovJ(point_list[0], point_list[1], point_list[2], point_list[3], point_list[4], point_list[5], coordinateMode) # P = [x, y, z, rx, ry, rz]
#         commandArrID = self.parseResultId(recv_movemess)
#         return commandArrID

#     def ClearRobotError(self):
#         while not self.exit_signal:
#             self.dashboard.ClearError()
#             sleep(5)

#     def parseResultId(self, valueRecv):
#         if valueRecv.find("Not Tcp") != -1:
#             print("Control Mode Is Not Tcp")
#             return [1]
#         recvData = re.findall(r'-?\d+', valueRecv)
#         recvData = [int(num) for num in recvData] # ! ตำแหน่ง
#         if len(recvData) == 0:
#             return [2]
#         return recvData
    
            
class GripperNS21:
    def __init__(self) -> None:
        pass
    
if __name__ == "__main__":
    dobot = DobotCR5ApiV3()
    dobot.test_run_to()