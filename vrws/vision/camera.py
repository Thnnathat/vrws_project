from .zed2i import Zed2i
from time import sleep
import pyzed.sl as sl
from threading import Lock, Event
import numpy as np
from vision.detector import Detector

class Camera(Zed2i):
    def __init__(self, event: Event, det: Detector) -> None:
        super().__init__()
        
        self.image_net: sl.Mat = sl.Mat()
        self.image_left_tmp: sl.Mat = sl.Mat()
        self.objects: sl.Objects = sl.Objects()
        
        # flag
        self.exit_signal = False
        
        self.det: Detector = det
        self.__event: Event = event
        
        self.lock = Lock()

        # Display
        camera_infos = self.get_camera_information()
        camera_res = camera_infos.camera_configuration.resolution
        # Utilities for 2D display
        self.__display_resolution = sl.Resolution(min(camera_res.width, 1280), min(camera_res.height, 720))
        self.__image_scale = [self.__display_resolution.width / camera_res.width, self.__display_resolution.height / camera_res.height]
        self.__image_left_ocv = np.full((self.__display_resolution.height, self.__display_resolution.width, 4), [245, 239, 239, 255], np.uint8) 

    def stop(self):
        self.exit_signal = True
    
    def camera_thread(self) -> None:
        while not self.exit_signal:
            if self.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                # -- Get the image
                self.lock.acquire()
                self.retrieve_image(self.image_left_tmp, sl.VIEW.LEFT)
                self.image_net = self.image_left_tmp.get_data()
                self.det.image_net = self.image_net # set an image to detector
                self.lock.release()
                self.__event.set()
                
                # -- Detection running on the other thread
                while self.__event.is_set():
                    if self.exit_signal: break
                    sleep(0.001)

                # Wait for detections
                self.lock.acquire()
                # -- Ingest detections
                self.ingest_custom_box_objects(self.det.detections)
                self.retrieve_objects(self.objects, self.obj_runtime_param)
                self.lock.release()
                
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