from detector import Detector
from time import sleep
from threading import Thread

class RobotArm(Thread):
    def __init__(self, det):
        super().__init__()
        self.det: Detector = det
        self.exit_signal: bool = False

    def run(self):
        print("-"*20)
        while not self.exit_signal:
            if self.det.data_flow.obj is not None:
                string = f"Robot get: {self.det.data_flow.obj.id}"
                print('-' * 20)
                print(string)
            else:
                print("Wait Object")
            sleep(5)