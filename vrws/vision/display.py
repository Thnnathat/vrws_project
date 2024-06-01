import cv2
import numpy as np
import pyzed.sl as sl
from vision.detector import Detector
from vision.viewer.render import GuideLine, Viewer
from vision.camera import Camera
from time import sleep
import time
from threading import Thread, Lock

class Display(Thread):
    def __init__(self, det: Detector, camera: Camera) -> None:
        super().__init__()
        self.name = "Display Thread"

        self.image_left: sl.Mat = sl.Mat()

        self.exit_signal: bool = False
        self.detector: Detector = det
        self.camera: Camera = camera

        self.guide_line = GuideLine()
        self.viewer = Viewer(det.model)

        # Utilities for 2D display
        self.display_resolution = self.camera.get_display_resolution
        self.image_scale = self.camera.get_image_scale
        self.image_left_ocv = self.camera.get_image_left_ocv

    def run(self):
        lock = Lock()
        while not self.exit_signal:
            objects = self.camera.objects
            enable_tracking = self.camera.obj_param.enable_tracking
    
            self.camera.retrieve_image(self.image_left, sl.VIEW.LEFT, sl.MEM.CPU, self.display_resolution)
            np.copyto(self.image_left_ocv, self.image_left.get_data())

            self.viewer.render_2D(self.image_left_ocv, self.image_scale, objects, enable_tracking)
            self.guide_line.draw_star_line_center_frame(self.image_left_ocv)

            # cv2.putText(self.image_left_ocv, fps, (7, 70), cv2.FONT_HERSHEY_SIMPLEX , 3, (100, 255, 0), 3, cv2.LINE_AA) 

            cv2.imshow("Display", self.image_left_ocv)

            key = cv2.waitKey(1)
            if key & 0XFF == ord('q'):
                self.stop()

    def stop(self):
        self.exit_signal = True
        self.detector.stop()
        self.camera.stop()