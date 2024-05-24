from typing import Any, Tuple
import pyzed.sl as sl
from pyzed.sl import Camera, InputType, Mat, Pose, Objects, CustomBoxObjectData
from zed2i_config import Zed2iInitParameters, Zed2iRuntimeParameters, Zed2iObjectDetectionParameters, Zed2iObjectDetectionRuntimeParameters, Zed2iPositionalTrackingParameters
from threading import Thread, Lock
import numpy as np
from viewer import Viewer, GuideLine
from time import sleep
import cv2

class Zed2i(Thread):
    def __init__(self) -> None:
        super().__init__()
        
        # สร้างตัวแปรแบบ global ใน class
        self.run_signal: bool = False
        self.exit_signal: bool = False
        self.image_left_tmp: Mat = Mat()
        self.image_net: Mat = Mat()
        self.detections: CustomBoxObjectData
        self.cam_w_pose: Pose = Pose()
        self.objects: Objects = Objects()
        self.image_left: Mat = Mat()
        input_type: sl.InputType = sl.InputType()
        self.img_in_torch = None

        #สร้าง instance กล้อง
        self.zed = Camera()

        # กำหนดคุณสมบัติเริ่มต้นของกล้อง
        init_params = Zed2iInitParameters()
        self.runtime_params = Zed2iRuntimeParameters()

        # สั่งเปิดกล้อง
        self.open(init_params)

        # กำหนดคุณสมบัติการติดตามวัตถุของกล้อง
        positional_tracking_parameters = Zed2iPositionalTrackingParameters()
        self.zed.enable_positional_tracking(positional_tracking_parameters)

        # กำหนดคุณสมบัติการตรวจจับวัตถุของกล้อง
        self.obj_param = Zed2iObjectDetectionParameters()
        self.zed.enable_object_detection(self.obj_param)

        self.obj_runtime_param = Zed2iObjectDetectionRuntimeParameters()

        # Display
        camera_infos = self.zed.get_camera_information()
        camera_res = camera_infos.camera_configuration.resolution

        # Utilities for 2D display
        self.display_resolution = sl.Resolution(min(camera_res.width, 1280), min(camera_res.height, 720))
        self.image_scale = [self.display_resolution.width / camera_res.width, self.display_resolution.height / camera_res.height]
        self.image_left_ocv = np.full((self.display_resolution.height, self.display_resolution.width, 4), [245, 239, 239, 255], np.uint8)
        
        self.guide_line = GuideLine()

    def open(self, init_parameter) -> str:
        status = self.zed.open(init_parameter)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit()
        print("Camera is openned.")
        return status

    def run(self) -> None:
        lock = Lock()
        while not self.exit_signal:
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                # -- Get the image
                lock.acquire()
                self.zed.retrieve_image(self.image_left_tmp, sl.VIEW.LEFT)
                self.image_net = self.image_left_tmp.get_data()
                lock.release()
                self.run_signal = True
                
                # -- Detection running on the other thread
                while self.run_signal:
                    sleep(0.001)

                # Wait for detections
                lock.acquire()
                # -- Ingest detections
                self.zed.ingest_custom_box_objects(self.detections)
                lock.release()
                self.zed.retrieve_objects(self.objects, self.obj_runtime_param)
                
                self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT, sl.MEM.CPU, self.display_resolution)
                np.copyto(self.image_left_ocv, self.image_left.get_data())
                
                self.viewer.render_2D(self.image_left_ocv, self.image_scale, self.objects, self.obj_param.enable_tracking)
                self.data_flow.insert_data_static(self.objects, self.obj_param.enable_tracking)
                
                cv2.imshow("ZED | 2D View", self.image_left_ocv)
                # cv2.imshow("Image in torch.", self.img_in_torch)
                key = cv2.waitKey(10)
                if key & 0XFF == ord('q'):
                    self.exit_signal = True
            else:
                self.exit_signal = True
        self.zed.close()

if __name__ == "__main__":
    zed2i = Zed2i()
    zed2i.start()