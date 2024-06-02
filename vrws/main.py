from vision.detector import Detector
from robot.robot_arm import RobotArm
from vision.display import Display
from threading import Thread, Event
from time import sleep

from vision.camera import Camera
from vision.data_flow import DataFlow
from vision.utils.roi import InterestRegion

class Main:
    def __init__(self) -> None:

        model = 'best'
        model_version = '7'
        self.model = f'E:/Computer Engineering/Computer Engineering Project/vrws_project/vision/runs/detect/train{model_version}/weights/{model}.pt'

    def start(self):
        det_cam_event = Event()
        cam_data_event = Event()
        
        detector = Detector(det_cam_event, self.model)
        t_detector = Thread(name="Thread Detector",target=detector.torch_thread)
        t_detector.start()

        roi = InterestRegion()
        roi.offest_x = 500
        roi.offest_y = 100
        roi.width = 1000
        roi.height = 800
        
        camera = Camera(det_cam_event, cam_data_event, detector, roi)
        t_camera = Thread(name="Thread Camera",target=camera.camera_thread)
        t_camera.start()

        data_flow = DataFlow(cam_data_event, camera)
        data_flow.start()
        
        robot = RobotArm(detector, data_flow, '192.168.5.1')
        robot.start()
        
        display = Display(detector, camera, data_flow, robot, roi)
        display.start()

if __name__ == "__main__":
    main = Main()
    main.start()