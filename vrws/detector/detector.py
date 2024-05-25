from typing import Any, Tuple
import pyzed.sl as sl
from pyzed.sl import Camera, Mat, Pose, Objects, CustomBoxObjectData
from zed2i_config import Zed2iInitParameters, Zed2iRuntimeParameters, Zed2iObjectDetectionParameters, Zed2iObjectDetectionRuntimeParameters, Zed2iPositionalTrackingParameters
from threading import Thread, Lock
from time import sleep
from ultralytics import YOLO
import cv2
import numpy as np
from viewer.render import Viewer, GuideLine
from data_flow import DataFlow

class Detector():
    def __init__(self) -> None:

        # สร้างตัวแปรแบบ global ใน class
        self.data_flow = DataFlow()
        self.run_signal: bool = False
        self.exit_signal: bool = False
        self.image_left_tmp: Mat = Mat()
        self.image_net: Mat = Mat()
        self.detections: CustomBoxObjectData
        self.cam_w_pose: Pose = Pose()
        self.objects: Objects = Objects()
        self.image_left: Mat = Mat()
        self.img_in_torch = None
        
        # สร้าง, จัดการ Thread
        self.lock: Lock = Lock()
        self.t_torch = Thread(target=self.torch_thread, args=(), kwargs={'weights': 'best.pt', 'img_size': 640, "conf_thres": 0.4})
        self.t_loop = Thread(target=self.camera_thread)

        # เลือก model
        model = 'best'
        model_version = '7'
        self.model = YOLO(f'E:/Computer Engineering/Computer Engineering Project/vrws_project//vision/runs/detect/train{model_version}/weights/{model}.pt')
        # self.model = YOLO('yolov8m.pt')

        #สร้าง instance กล้อง
        self.zed = Camera()

        # สร้าง instance viewer เพื่อไว้แสดงวีดีโอและข้อมูลในรูปแบบ gui
        self.viewer = Viewer(self.model)

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

        self.guide_line = GuideLine()
        
    def start(self) -> None:
        self.exit_signal = False
        self.t_torch.start()
        self.t_loop.start()
        print("Vision start.")
        
    def stop(self) -> None:
        self.exit_signal = True
        print("Vision stop.")
        
    def getCamRes(self):
        cam_res = self.zed.get_camera_information().camera_configuration.resolution
        return cam_res

    def getImgSizeFromCamRes(self) -> Tuple[int, int]:
        cam_res = self.getCamRes()
        img_width = cam_res.width
        img_height = cam_res.height

        return (img_width, img_height)

    def open(self, init_parameter) -> str:
        status = self.zed.open(init_parameter)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit()
        print("Camera is openned.")
        return status

    def camera_thread(self) -> None:
        while not self.exit_signal:
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                # -- Get the image
                self.lock.acquire()
                self.zed.retrieve_image(self.image_left_tmp, sl.VIEW.LEFT)
                self.image_net = self.image_left_tmp.get_data()
                self.lock.release()
                self.run_signal = True
                
                # -- Detection running on the other thread
                while self.run_signal:
                    sleep(0.001)

                # Wait for detections
                self.lock.acquire()
                # -- Ingest detections
                self.zed.ingest_custom_box_objects(self.detections)
                self.lock.release()
                self.zed.retrieve_objects(self.objects, self.obj_runtime_param)
                
                self.data_flow.insert_data_static(self.objects, self.obj_param.enable_tracking)
            else:
                self.exit_signal = True
        self.zed.close()


    def torch_thread(self, weights, img_size, conf_thres=0.2, iou_thres=0.45):
        print("Intializing Network...")

        while not self.exit_signal:
            if self.run_signal:
                self.lock.acquire()
                img = cv2.cvtColor(self.image_net, cv2.COLOR_BGRA2BGR)
                det = self.model.predict(img, save=False, imgsz=img_size, conf=conf_thres, iou=iou_thres, verbose=False)[0].cpu().numpy().boxes
                # ZED CustomBox format (with inverse letterboxing tf applied)
                self.detections = self.detections_to_custom_box(det, self.image_net) # ได้ BBox มา
                self.lock.release()
                self.run_signal = False

            sleep(0.01)
    

    def detections_to_custom_box(self, detections, im0):
        output = []
        for i, det in enumerate(detections):
            xywh = det.xywh[0]

            # Creating ingestable objects for the ZED SDK
            obj = sl.CustomBoxObjectData()
            obj.bounding_box_2d = self.xywh2abcd(xywh, im0.shape)
            obj.label = det.cls
            obj.probability = det.conf
            obj.is_grounded = False
            output.append(obj) # ได้ CustomBox ในแต่ละ Object ใน 1 เฟรม (1 รูป)
        return output


    def xywh2abcd(self, xywh, im_shape):
        output = np.zeros((4, 2))

        # Center / Width / Height -> BBox corners coordinates
        x_min = (xywh[0] - 0.5*xywh[2]) #* im_shape[1]
        x_max = (xywh[0] + 0.5*xywh[2]) #* im_shape[1]
        y_min = (xywh[1] - 0.5*xywh[3]) #* im_shape[0]
        y_max = (xywh[1] + 0.5*xywh[3]) #* im_shape[0]

        # A ------ B
        # | Object |
        # D ------ C

        output[0][0] = x_min
        output[0][1] = y_min

        output[1][0] = x_max
        output[1][1] = y_min

        output[2][0] = x_max
        output[2][1] = y_max

        output[3][0] = x_min
        output[3][1] = y_max
        return output


if __name__ == "__main__":
    det = Detector()
    det.start()