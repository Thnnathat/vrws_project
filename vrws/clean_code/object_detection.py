from camera import Zed2i
from detector import Detector
from threading import Thread, Lock
from time import sleep
import pyzed.sl as sl
from viewer import Viewer
import cv2

class ObjectDetection(Thread):
    def __init__(self):
        super().__init__()
        self.zed: Zed2i = Zed2i()
        self.detector: Detector = Detector()
        
        self.exit_signal: bool = False
        self.run_signal: bool = False
        
    def run(self):
        self.zed.start()
        self.detector.start()
        while not self.exit_signal:
            if self.zed.run_signal:
                self.detector.run_signal = True
                
            if self.detector.run_signal == False:
                self.zed.run_signal = False