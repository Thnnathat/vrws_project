import cv2
import numpy as np
import pyzed.sl as sl
from .detector import Detector
from pipe import DataFlow
from robot import RobotArm
from .viewer import GuideLine, Viewer
from .camera import Camera
from threading import Thread, Lock
from .utils import InterestRegion

class Display(Thread):
    def __init__(self, det: Detector, camera: Camera, data_flow: DataFlow = None, robot: RobotArm = None, roi: InterestRegion = None) -> None:
        super().__init__()
        self.name = "Display Thread"

        self.image_left: sl.Mat = sl.Mat()

        self.exit_signal: bool = False
        self.detector: Detector = det
        self.camera: Camera = camera
        self.data_flow: DataFlow = data_flow
        self.robot: RobotArm = robot
        self.roi: InterestRegion = roi

        self.guide_line = GuideLine()
        self.viewer = Viewer(det.model)

        # Utilities for 2D display
        self.display_resolution = self.camera.get_display_resolution
        self.image_scale = self.camera.get_image_scale
        self.image_left_ocv = self.camera.get_image_left_ocv

    def run(self):
        while not self.exit_signal:
            objects = self.camera.objects
            enable_tracking = self.camera.obj_param.enable_tracking
    
            self.camera.retrieve_image(self.image_left, sl.VIEW.LEFT, sl.MEM.CPU, self.display_resolution)
            np.copyto(self.image_left_ocv, self.image_left.get_data())

            self.viewer.render_2D(self.image_left_ocv, self.image_scale, objects, enable_tracking)
            # self.guide_line.draw_star_line_center_frame(self.image_left_ocv)
            self.guide_line.draw_roi_rectangle(self.image_left_ocv, self.roi.roi_point, self.image_scale)
            self.guide_line.draw_roi_polygon(self.image_left_ocv, self.roi.poly_point, self.image_scale)

            cv2.imshow("Main Display HD Scale", self.image_left_ocv)
            cv2.imshow("Image Real Scale", self.detector.image_net)

            key = cv2.waitKey(1)
            if key & 0XFF == ord('q'):
                self.stop()

    def stop(self):
        self.exit_signal = True
        if self.detector is not None:
            self.detector.stop()
        if self.camera is not None: 
            self.camera.stop()
        if self.data_flow is not None:
            self.data_flow.stop()
        if self.robot is not None:
            self.robot.stop()