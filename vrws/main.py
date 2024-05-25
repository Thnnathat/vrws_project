from vision.detector import Detector
from robot.robot_arm import RobotArm
from vision.display import Display
from threading import Thread
from time import sleep

class Main(Thread):
    def __init__(self, det: Detector, disp: Display, robot: RobotArm) -> None:
        super().__init__()
        self.name = "Control Thread"
        
        self.exit_signal = False
        self.det = det
        self.robot = robot
        self.disp = disp

    def run(self):
        while not self.exit_signal:
            if self.det.exit_signal or self.robot.exit_signal or self.disp.exit_signal:
                self.det.exit_signal = self.robot.exit_signal = self.disp.exit_signal  = self.exit_signal = True
            sleep(0.001)

if __name__ == "__main__":
    det = Detector()
    det.start()
    
    disp = Display(det)
    disp.start()

    robot = RobotArm(det)
    robot.start()

    main = Main(det, disp, robot)
    main.start()

    det.t_torch.join()
    det.t_camera.join()
    disp.join()
    robot.join()
    main.join()
    exit()