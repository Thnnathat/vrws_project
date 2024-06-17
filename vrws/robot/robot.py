from pipe import DataFlow
from vision.detector import Detector
from time import sleep
from threading import Thread
from typing import Tuple
from .dobot_api_v4 import DobotApiDashboard, DobotApiFeedBack, DobotApiDashMove
from threading import Thread
import re

class RobotControl(Thread):
    def __init__(self, detector: Detector, data_flow: DataFlow, cam_position: tuple[float, float, float], ip: str = '192.168.5.1') -> None:
        super().__init__()
        self.name: str = "Robot Thread"
        
        self.data_flow: DataFlow = data_flow
        self.detector: Detector = detector
        self.exit_signal: bool = False
        self.cam_position: tuple[float, float, float] = cam_position
        
        self.class_names = detector.model.names

        # self.dobot = DobotCR5(ip)
        # if not self.dobot.is_connected:
        #     self.exit_signal = True
        # self.dobot.Enable()
            
        self.drop_point: dict[str, float] = {
                                                "red-cube": [0.0, 0.0, 0.0],
                                                "green-cube": [0.0, 0.0, 0.0],
                                                "blue-cube": [0.0, 0.0, 0.0],
                                                "yellow-cube": [0.0, 0.0, 0.0]
                                            }
        
        self.r: list[float] = [0.0, 0.0, 0.0]
        # self.simple_drop_point = [0.0, 0.0, 200]

    def stop(self):
        self.exit_signal = True

    def run(self):
        print("-"*20)
        while not self.exit_signal:
            if self.data_flow.obj is not None:
                obj = self.data_flow.obj
                
                self.simulate(obj)
                
                # self.run_to_object(obj)
                self.hold()
                # self.run_to_drop(obj)
                self.release()
            else:
                print("Waiting for Objects...")
                sleep(5)
                

    def simulate(self, obj):
        self.display(obj)
        sleep(5)
    
    def display(self, obj):
        print('-' * 20)
        string = f"Robot get: {obj.id}\nClass: {self.class_names[obj.raw_label]}\nPosition X: {obj.position[0]}, Y: {obj.position[2]}, Z: {obj.position[1]}"
        print(string)
        print('-' * 20)

    def run_to_object(self, obj):
        point_list: list[float] = self.cal_pisition_relation_cam_robot(obj) # [x, y, z, rx, ry, rz]
        self.display(obj)
        self.dobot.RunToPoint(point_list, 0)

    def run_to_drop(self, obj):
        class_name: str | None = self.class_names[obj.raw_label]
        point_list: list[float] = self.drop_point[class_name] + self.r
        self.dobot.RunToPoint(point_list, 0)

    def hold(self):
        pass
    
    def release(self):
        pass
    
    def cal_pisition_relation_cam_robot(self, obj) -> list[float]:
        point_list: list[float] = [obj.potition[0] - self.cam_position[0], obj.position[2] - self.cam_position[1], obj.position[1] - self.cam_position[2]] + self.r
        return point_list

class DobotCR5:
    def __init__(self, ip: str) -> None:
        self.ip: str = ip
        self.dashboardPort: int = 29999
        self.feedPort: int = 30004
        self.movePort: int = 30005
        
        self.is_connected: bool = False
        self.exit_signal: bool = False

        # เชื่อมต่อ Robot
        self.dashboard: DobotApiDashboard | None = None
        self.feed: DobotApiFeedBack | None = None
        self.move: DobotApiDashMove | None = None

        self.t_clearError = Thread(target=self.ClearRobotError)

    def Connect(self):
        self.dashboard = DobotApiDashboard(self.ip, self.dashboardPort)
        self.feed = DobotApiFeedBack(self.ip, self.feedPort)
        self.move = DobotApiDashMove(self.ip, self.movePort)
        self.is_connected = True
        self.t_clearError.start()
        print("Dobot connection error.")
        
    def Enable(self):
        enableState = self.parseResultId(self.dashboard.EnableRobot())
        if enableState[0] != 0:
            print("Failed to enable: Check whether port 29999 is occupied.")
            return
        print("Enabled successfully!")

    def RunToPoint(self, point_list: list[float], coordinateMode: int):
        while True:
            p2id = self.RunPoint(point_list, coordinateMode)        
            if p2id[0] == 0: # ไม่มี error (ถ้ามี error จะทำการส่งคำสั่งใหม่เรื่อยๆ)
                self.move.WaitArrive(p2id[1]) # รอให้แขกลถึงตำแหน่ง
                break

    def RunPoint(self, point_list: list[float], coordinateMode: int):
        recv_movemess = self.dashboard.MovJ(point_list[0], point_list[1], point_list[2], point_list[3], point_list[4], point_list[5], coordinateMode) # P = [x, y, z, rx, ry, rz]
        commandArrID = self.parseResultId(recv_movemess)
        return commandArrID

    def ClearRobotError(self):
        while not self.exit_signal:
            self.dashboard.ClearError()
            sleep(5)

    def parseResultId(self, valueRecv):
        if valueRecv.find("Not Tcp") != -1:
            print("Control Mode Is Not Tcp")
            return [1]
        recvData = re.findall(r'-?\d+', valueRecv)
        recvData = [int(num) for num in recvData] # ! ตำแหน่ง
        if len(recvData) == 0:
            return [2]
        return recvData
            
class GripperNS21:
    def __init__(self) -> None:
        pass