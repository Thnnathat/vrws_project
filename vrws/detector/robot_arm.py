from vrws.detector.detector import Zed2i
from time import sleep
from threading import Thread

class RobotArm(Thread):
    def __init__(self, zed):
        super().__init__()
        self.zed: Zed2i = zed
        self.exit_signal: bool = False

    def run(self):
        print("-"*20)
        while not self.exit_signal:
            if self.zed.data_flow.obj is not None:
                string = f"Robot get: {self.zed.data_flow.obj.id}"
                print('-' * 20)
                print(string)
            else:
                print("Wait Object")
            sleep(5)