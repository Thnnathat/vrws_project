from vision.detector import Detector
# from robot.robot_arm import RobotArm
from vision.display import Display
from threading import Thread, Event
from time import sleep

from vision.camera import Camera

# class Main(Thread):
#     def __init__(self, det: Detector, disp: Display, robot: RobotArm) -> None:
#         super().__init__()
#         self.name = "Control Thread"
        
#         self.exit_signal = False
#         self.det = det
#         self.robot = robot
#         self.disp = disp

#     def run(self):
#         while not self.exit_signal:
#             if self.det.exit_signal or self.robot.exit_signal or self.disp.exit_signal:
#                 print("Stopping...")
#                 self.det.exit_signal = self.robot.exit_signal = self.disp.exit_signal  = self.exit_signal = True
#             sleep(0.001)

# class Main1:
#     def __init__(self) -> None:
#         det = Detector()
#         det.start()
    
#         disp = Display(det)
#         disp.start()

#         robot = RobotArm(det, '192.168.5.1')
#         robot.start()

#         main = Main(det, disp, robot)
#         main.start()

#         det.t_torch.join()
#         det.t_camera.join()
#         disp.join()
#         robot.join()
#         main.join()
#         print("Exit")
#         exit()

class Main2:
    def __init__(self) -> None:

        model = 'best'
        model_version = '7'
        self.model = f'E:/Computer Engineering/Computer Engineering Project/vrws_project/vision/runs/detect/train{model_version}/weights/{model}.pt'

    def start(self):
        det_cam_event = Event()
        
        detector = Detector(det_cam_event, self.model)
        t_detector = Thread(name="Thread Detector",target=detector.torch_thread)
        t_detector.start()
        
        camera = Camera(det_cam_event, detector)
        t_camera = Thread(name="Thread Camera",target=camera.camera_thread)
        t_camera.start()
        
        display = Display(detector, camera)
        display.start()

if __name__ == "__main__":
    main = Main2()
    main.start()