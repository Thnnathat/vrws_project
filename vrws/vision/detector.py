from ultralytics import YOLO
import numpy as np
import cv2
from time import sleep
import pyzed.sl as sl
from threading import Lock, Event
class Detector:
    def __init__(self, event: Event, model: str, img_size: int = 640, conf_thres: float = 0.2, iou_thres: float = 0.45) -> None:
        
        # Wait from camera
        self.image_net: sl.Mat = sl.Mat()
        
        self.detections = None
        
        self.model = YOLO(model)
        self.lock = Lock()

        # flag
        self.exit_signal = False

        self.__event: Event = event
        self.img_size: int = img_size
        self.conf_thres: float = conf_thres
        self.iou_thres: float = iou_thres
    
    def stop(self):
        self.exit_signal = True

    def torch_thread(self):
        
        print("Intializing Network...")
        while not self.exit_signal:
            if self.__event.is_set():
                self.lock.acquire()
                img = cv2.cvtColor(self.image_net, cv2.COLOR_BGRA2BGR)
                det = self.model.predict(img, save=False, imgsz=self.img_size, conf=self.conf_thres, iou=self.iou_thres, verbose=False)[0].cpu().numpy().boxes
                # ZED CustomBox format (with inverse letterboxing tf applied)
                self.detections = self.detections_to_custom_box(det, self.image_net) # ได้ BBox มา
                self.lock.release()
                self.__event.clear()
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