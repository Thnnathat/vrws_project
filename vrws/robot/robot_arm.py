from vision.detector import Detector
from time import sleep
from threading import Thread
from typing import Tuple
from dobot_api import DobotApiDashboard, DobotApiFeedBack, DobotApiDashMove
from threading import Thread
class RobotArm(Thread):
    def __init__(self, det: Detector, ip: str):
        super().__init__()
        self.name: str = "Robot Thread"
        
        self.det: Detector = det
        self.exit_signal: bool = False

        self.dobot = DobotCR5(ip)
        if not self.dobot.is_connected:
            self.exit_signal = True

    def run(self):
        print("-"*20)
        while not self.exit_signal:
            if self.det.data_flow.obj is not None:
                obj = self.det.data_flow.obj
                point_list: list[float] = [obj.potition[0], obj.position[1], obj.position[2], 0, 0, 0] # [x, y, z, rx, ry, rz]
                self.dobot.RunToPoint(point_list, 0)
                string = f"Robot get: {obj.id}\nClass: {self.det.model.names[obj.raw_label]}\nPosition X: {obj.position[0]}, Y: {obj.position[1]}, Z: {obj.position[2]}"
                print('-' * 20)
                print(string)
            else:
                print("Waiting for Objects")
            sleep(5)
            
    def cal_pisition_relation_cam_robot(self):
        pass

class DobotCR5:
    def __init__(self, ip: str) -> None:
        self.ip: str = ip
        self.dashboardPort: int = 29999
        self.feedPort: int = 30004

        self.dashboard: DobotApiDashboard | None = None
        self.feed: DobotApiFeedBack | None = None
        self.move: DobotApiDashMove | None = None
        
        self.is_connected: bool = False
        self.exit_signal: bool = False
        self.dashboard, self.feed = self.Connect()

        self.t_chearError = Thread(target=self.ClearRobotError)

    def Connect(self) -> Tuple[DobotApiDashboard, DobotApiFeedBack]:
        try: 
            dashboard = DobotApiDashboard(self.ip, self.dashboardPort)
            feed = DobotApiFeedBack(self.ip, self.feedPort)
            self.is_connected = True
            self.t_chearError.start()
            return dashboard, feed
        except:
            self.exit_signal = True
            print("Dobot connection error.")

    def RunToPoint(self, point_list: list[float], coordinateMode: int):
        while True:
            p2id = self.RunPoint(point_list, coordinateMode)        
            if p2id[0] == 0: # No error
                self.move.WaitArrive(p2id[1]) # Wait for point
                break

    def RunPoint(self, point_list: list[float], coordinateMode: int):
        recv_movemess = self.dashboard.MovJ(point_list[0], point_list[1], point_list[2], point_list[3], point_list[4], point_list[5], coordinateMode) # P = [x, y, z, rx, ry, rz]
        commandArrID = self.dashboard.ParseResultId(recv_movemess)
        return commandArrID

    def ClearRobotError(self):
        while not self.exit_signal:
            self.dashboard.ClearError()
            sleep(5)