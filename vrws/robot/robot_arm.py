from vision.detector import Detector
from time import sleep
from threading import Thread

class RobotArm(Thread):
    def __init__(self, det: Detector):
        super().__init__()
        self.name = "Robot Thread"
        
        self.det: Detector = det
        self.exit_signal: bool = False

    def run(self):
        print("-"*20)
        while not self.exit_signal:
            if self.det.data_flow.obj is not None:
                obj = self.det.data_flow.obj
                string = f"Robot get: {obj.id}\nClass: {self.det.model.names[obj.raw_label]}\nPosition X: {obj.position[0]}, Y: {obj.position[1]}, Z: {obj.position[2]}"
                print('-' * 20)
                print(string)
            else:
                print("Waiting for Objects")
            sleep(5)