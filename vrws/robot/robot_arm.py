from vision.detector import Detector
from time import sleep
from threading import Thread
from typing import Tuple
from robot.dobot_api import DobotApiDashboard, DobotApiFeedBack, DobotApiDashMove
from threading import Thread
class RobotArm(Thread):
    def __init__(self, det: Detector, ip: str) -> None:
        super().__init__()
        self.name: str = "Robot Thread"
        
        self.det: Detector = det
        self.exit_signal: bool = False

        # self.dobot = DobotCR5(ip)
        # if not self.dobot.is_connected:
        #     self.exit_signal = True
        # self.dobot.Enable()
            
        self.drop_point: dict[str, float] = {
                                                "unknow": [0.0, 0.0, 0.0],
                                                "red-cube": [0.0, 0.0, 0.0],
                                                "green-cube": [0.0, 0.0, 0.0],
                                                "blue-cube": [0.0, 0.0, 0.0],
                                                "yellow-cube": [0.0, 0.0, 0.0]
                                            }
        self.simple_drop_point = [0.0, 0.0, 200]

    def run(self):
        print("-"*20)
        while not self.exit_signal:
            if self.det.data_flow.obj is not None:
                obj = self.det.data_flow.obj
                
                self.simulate(obj)
                
                # self.run_to_object(obj)
                # self.run_to_drop(obj)
            else:
                print("Waiting for Objects...")
                sleep(5)
                

    def simulate(self, obj):
        self.display(obj)
        sleep(5)
    
    def display(self, obj):
        print('-' * 20)
        string = f"Robot get: {obj.id}\nClass: {self.det.model.names[obj.raw_label]}\nPosition X: {obj.position[0]}, Y: {obj.position[1]}, Z: {obj.position[2]}"
        print(string)
        print('-' * 20)

    def run_to_object(self, obj):
        point_list: list[float] = [obj.potition[0], obj.position[1], obj.position[2], 0, 0, 0] # [x, y, z, rx, ry, rz]
        self.display(obj)
        self.dobot.RunToPoint(point_list, 0)

    def run_to_drop(self, point_list, obj):
        class_name: str | None = self.det.model.names[obj.raw_label]
        r = [0, 0, 0]
        point_list: list[float] = self.drop_point[class_name] + r
        
        self.dobot.RunToPoint(point_list, 0)
    
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

        self.t_clearError = Thread(target=self.ClearRobotError)

    def Connect(self) -> Tuple[DobotApiDashboard, DobotApiFeedBack]:
        try: 
            dashboard = DobotApiDashboard(self.ip, self.dashboardPort)
            feed = DobotApiFeedBack(self.ip, self.feedPort)
            self.is_connected = True
            self.t_clearError.start()
            return dashboard, feed
        except:
            self.exit_signal = True
            print("Dobot connection error.")
        
    def Enable(self):
        enableState = self.dashboard.ParseResultId(self.dashboard.EnableRobot())
        if enableState[0] != 0:
            print("Failed to enable: Check whether port 29999 is occupied.")
            return
        print("Enabled successfully!")

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