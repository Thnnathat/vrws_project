import cv2
import numpy as np
import pyzed.sl as sl
from vision.detector import Detector
from vision.viewer.render import GuideLine, Viewer
from time import sleep
import time
from threading import Thread, Lock

class Display(Thread):
    def __init__(self, det: Detector) -> None:
        super().__init__()
        self.name = "Display Thread"

        self.image_left: sl.Mat = sl.Mat()

        self.exit_signal: bool = False
        self.det: Detector = det

        self.guide_line = GuideLine()
        self.viewer = Viewer(det.model)

        # Display
        camera_infos = self.det.zed.get_camera_information()
        camera_res = camera_infos.camera_configuration.resolution

        # Utilities for 2D display
        self.display_resolution = sl.Resolution(min(camera_res.width, 1280), min(camera_res.height, 720))
        self.image_scale = [self.display_resolution.width / camera_res.width, self.display_resolution.height / camera_res.height]
        self.image_left_ocv = np.full((self.display_resolution.height, self.display_resolution.width, 4), [245, 239, 239, 255], np.uint8) 

    def run(self):
        lock = Lock()
        while not self.exit_signal:
            objects = self.det.objects
            enable_tracking = self.det.obj_param.enable_tracking
    
            self.det.zed.retrieve_image(self.image_left, sl.VIEW.LEFT, sl.MEM.CPU, self.display_resolution)
            np.copyto(self.image_left_ocv, self.image_left.get_data())

            self.viewer.render_2D(self.image_left_ocv, self.image_scale, objects, enable_tracking)
            self.guide_line.draw_star_line_center_frame(self.image_left_ocv)

            # cv2.putText(self.image_left_ocv, fps, (7, 70), cv2.FONT_HERSHEY_SIMPLEX , 3, (100, 255, 0), 3, cv2.LINE_AA) 

            cv2.imshow("Display", self.image_left_ocv)

            key = cv2.waitKey(1)
            if key & 0XFF == ord('q'):
                self.exit_signal = True
                self.det.exit_signal = True