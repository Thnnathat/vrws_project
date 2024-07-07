from pipe import DataFlow
from vision.detector import Detector
from time import sleep
from threading import Thread
from .dobotcr5apiv3 import DobotCR5ApiV3

class RobotControl(Thread):
    def __init__(self, drop_points, detector: Detector, data_flow: DataFlow, cam_position: tuple[float, float, float], ip: str = '192.168.5.1') -> None:
        super().__init__()
        self.name: str = "Robot Thread"
        
        self.data_flow: DataFlow = data_flow
        self.detector: Detector = detector
        self.exit_signal: bool = False
        self.cam_position: tuple[float, float, float] = cam_position
        
        self.class_names = detector.model.names

        self.dobot = DobotCR5ApiV3(ip)
            
        self.drop_points = drop_points

    def stop(self):
        self.exit_signal = True
        self.dobot.dashboard.DisableRobot()

    def run(self):
        print("-"*20)
        while not self.exit_signal:
            if self.data_flow.obj is not None:
                obj = self.data_flow.obj
                if obj is not None:
                    print(obj.raw_label)
                
                    # self.simulate(obj)
                    point = self.cal_pisition_relation_cam_robot(obj)
                    print(point)
                    if point[2] < 150 or point[2] > 600:
                        print("*" * 20 + "\tPosition out of bound...!!!\t" + "*" * 20)
                        sleep(0.1)
                        continue
                    self.run_to_above_object(obj)
                    self.run_to_object(obj)
                    self.hold()
                    self.run_to_common_state(obj)
                    self.run_to_drop(obj)
                    self.release()
                    if self.data_flow.obj is not None:
                        obj = self.data_flow.obj
                    self.run_to_common_state(obj)
                    obj = None
                    sleep(0.1)
            else:
                print("Waiting for Objects...")
                sleep(0.1)
                

    def simulate(self, obj):
        self.display(obj)
        sleep(5)
    
    def display(self, obj):
        print('-' * 20)
        string = f"Robot get: {obj.id}\nClass: {self.class_names[obj.raw_label]}\nPosition X: {obj.position[0]}, Y: {obj.position[2]}, Z: {obj.position[1]}"
        print(string)
        print('-' * 20)

    def run_to_object(self, obj):
        point_list: list[float] = self.cal_pisition_relation_cam_robot(obj) # [x, y, z]
        print(point_list)
        self.display(obj)
        self.dobot.RunToPoint(point_list, 0)
        print("run_to_object")
    
    def run_to_above_object(self, obj):
        point_list: list[float] = self.cal_pisition_relation_cam_robot(obj) # [x, y, z]
        point_list[2] = 300
        self.dobot.RunToPoint(point_list, 0)
        print("run_to_above_object")

    def run_to_drop(self, obj):
        point_list: list[float] = self.find_drop_point_follow_class(obj)
        self.dobot.RunToPoint(point_list, 0)
        print("run_to_drop")
        
    def run_to_common_state(self, obj):
        point_list: list[float] = self.cal_pisition_relation_cam_robot(obj) # [x, y, z]
        point_list[2] = 500
        point_list[1] = -290
        self.dobot.RunToPoint(point_list, 0)
        print("run_to_common_state")

    def hold(self):
        self.dobot.gripper.grip_close()
        sleep(1)
    
    def release(self):
        self.dobot.gripper.grip_open()
        sleep(1)
    
    def cal_pisition_relation_cam_robot(self, obj) -> list[float]:
        point_list: list[float] = [obj.position[0] - self.cam_position[0], (obj.position[2] * -1) - self.cam_position[1], obj.position[1] + self.cam_position[2]] #! 
        print(point_list)
        return point_list
    
    def find_drop_point_follow_class(self, obj):
        return self.drop_points[self.class_names[obj.raw_label]]
            