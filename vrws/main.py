from vision import Detector
from robot import RobotControl
from vision import Display
from threading import Thread, Event
from time import sleep

from vision import Camera
from pipe import DataFlow
from vision.utils import InterestRegion

class Main:
    def __init__(self) -> None:

        model = 'best'
        model_version = '7'
        self.model = f'E:/Computer Engineering/Computer Engineering Project/vrws_project/vision/color_cube/runs/detect/train{model_version}/weights/{model}.pt'

        self.cam_position: tuple[float, float, float] = [0, 0, 0]
        
        self.roi = InterestRegion()
        self.roi.offest_x = 500
        self.roi.offest_y = 100
        self.roi.width = 1000
        self.roi.height = 1000
        self.roi.set_poly_point((700, 70), (500, 1080), (1700, 1080), (1400, 70)) # (left, top), (left, bottom), (right, bottom), (right, top)

    def start(self):
        det_cam_event = Event()
        cam_data_event = Event()
        
        detector = Detector(det_cam_event, self.model, conf_thres=0.7)
        t_detector = Thread(name="Thread Detector", target=detector.torch_thread)
        t_detector.start()

        camera = Camera(det_cam_event, cam_data_event, detector, self.roi)
        t_camera = Thread(name="Thread Camera",target=camera.camera_thread)
        t_camera.start()

        data_flow = DataFlow(cam_data_event, camera)
        data_flow.start()
        
        robot = RobotControl(detector, data_flow, self.cam_position,'192.168.5.1')
        robot.start()
        
        display = Display(detector, camera, data_flow, robot, self.roi)
        display.start()

if __name__ == "__main__":
    main = Main()
    main.start()