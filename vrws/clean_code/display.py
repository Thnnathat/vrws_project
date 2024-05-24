from viewer import Viewer, GuideLine
from camera_sensors import Zed2i
import cv2
from threading import Thread
class Display:
    def __init__(self, zed) -> None:
        self.zed: Zed2i = zed
        self.t_display = Thread(target=self.display_thred)
        
    def display_thred(self):
        viewer = Viewer(self.zed.model)
        while not self.zed.exit_signal:

            viewer.render_2D(self.zed.image_left_ocv, self.zed.image_scale, self.zed.objects, self.zed.obj_param.enable_tracking)
            cv2.imshow("ZED | 2D View", self.zed.image_left_ocv)

            key = cv2.waitKey(10)
            if key & 0XFF == ord('q'):
                self.zed.exit_signal = True
    
    def display(self):
        self.t_display.start()