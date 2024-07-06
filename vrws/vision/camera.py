from .zed2i import Zed2i
from time import sleep
import pyzed.sl as sl
from threading import Lock, Event
import numpy as np
from .detector import Detector
from .utils import InterestRegion
import cv2

class Camera(Zed2i):
    def __init__(self, det_cam_event: Event, cam_data_event: Event, det: Detector, roi: InterestRegion) -> None:
        super().__init__()
        
        self.image_net: sl.Mat = sl.Mat()
        self.image_left_tmp: sl.Mat = sl.Mat()
        self.objects: sl.Objects = sl.Objects()
        
        # flag
        self.exit_signal = False
        
        self.roi: InterestRegion = roi
        
        self.det: Detector = det
        self.__det_cam_event: Event = det_cam_event
        self.__cam_data_event: Event = cam_data_event
        
        self.lock = Lock()

        # Display
        camera_infos = self.get_camera_information()
        self.camera_res = camera_infos.camera_configuration.resolution
        self.res_width = self.camera_res.width
        self.res_height = self.camera_res.height
        # Utilities for 2D display
        self.__display_resolution = sl.Resolution(min(self.camera_res.width, 1280), min(self.camera_res.height, 720))
        self.__image_scale = [self.__display_resolution.width / self.camera_res.width, self.__display_resolution.height / self.camera_res.height]
        self.__image_left_ocv = np.full((self.__display_resolution.height, self.__display_resolution.width, 4), [245, 239, 239, 255], np.uint8)
        
        self.__roi_image_template = np.full((self.camera_res.height, self.camera_res.width, 4), [245, 239, 239, 255], np.uint8)

        
    def stop(self):
        self.exit_signal = True
    
    def camera_thread(self) -> None:
        while not self.exit_signal:
            if self.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                # -- Get the image
                self.lock.acquire()
                self.retrieve_image(self.image_left_tmp, sl.VIEW.LEFT)
                self.image_net = self.image_left_tmp.get_data()
                image_net = self.roi.crop(self.__roi_image_template, self.image_net, self.res_width, self.res_height)
                self.det.image_net = image_net

                self.lock.release()
                self.__det_cam_event.set()
                
                # -- Detection running on the other thread
                while self.__det_cam_event.is_set():
                    if self.exit_signal: break
                    sleep(0.001)

                # Wait for detections
                self.lock.acquire()
                # -- Ingest detections
                self.ingest_custom_box_objects(self.det.detections)
                self.retrieve_objects(self.objects, self.obj_runtime_param)
                self.lock.release()
                self.__cam_data_event.wait()

                # cv2.imshow("Image Real Scale", image_net)

                # key = cv2.waitKey(1)
                # if key & 0XFF == ord('q'):
                #     self.stop()
                
                # self.real_sence()
                
                # self.data_flow.insert_data_static(self.objects, self.obj_param.enable_tracking)
            else:
                self.stop()
        self.close()
        
    @property
    def get_display_resolution(self):
        return self.__display_resolution

    @property
    def get_image_scale(self):
        return self.__image_scale

    @property
    def get_image_left_ocv(self):
        return self.__image_left_ocv