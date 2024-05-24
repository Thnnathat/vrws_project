from ultralytics import YOLO
from threading import Thread, Lock
from time import sleep
import cv2
import pyzed.sl as sl
import numpy as np

class Detector(Thread):
    def __init__(self) -> None:
        super().__init__()
        # สร้างตัวแปรแบบ global ใน class
        
        self.exit_signal: bool = False
        self.run_signal: bool = False
        self.detections: list = []
        self.image_net = sl.Mat()
        
        model = 'best'
        model_version = '7'
        self.model = YOLO(f'E:/Computer Engineering/Computer Engineering Project/vrws_project//vision/runs/detect/train{model_version}/weights/{model}.pt')

    def run(self, img_size=640, conf_thres=0.2, iou_thres=0.45):
        print("Intializing Network...")
        lock = Lock()
        while not self.exit_signal:
            if self.run_signal:
                lock.acquire()
                img = cv2.cvtColor(self.image_net, cv2.COLOR_BGRA2BGR)
                det = self.model.predict(img, save=False, imgsz=img_size, conf=conf_thres, iou=iou_thres, verbose=False)[0].cpu().numpy().boxes
                # ZED CustomBox format (with inverse letterboxing tf applied)
                self.detections = self.detections_to_custom_box(det, self.image_net) # ได้ BBox มา
                lock.release()
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