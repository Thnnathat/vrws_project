import pyzed.sl as sl
from .zed2i_config import Zed2iInitParameters, Zed2iRuntimeParameters, Zed2iObjectDetectionParameters, Zed2iObjectDetectionRuntimeParameters, Zed2iPositionalTrackingParameters

class Zed2i(sl.Camera):
    def __init__(self):
        # กำหนดคุณสมบัติเริ่มต้นของกล้อง
        init_params = Zed2iInitParameters()
        self.runtime_params = Zed2iRuntimeParameters()

        # สั่งเปิดกล้อง
        self.open_cam(init_params)

        # กำหนดคุณสมบัติการติดตามวัตถุของกล้อง
        positional_tracking_parameters = Zed2iPositionalTrackingParameters()
        self.enable_positional_tracking(positional_tracking_parameters)

        # กำหนดคุณสมบัติการตรวจจับวัตถุของกล้อง
        self.obj_param = Zed2iObjectDetectionParameters()
        self.enable_object_detection(self.obj_param)

        self.obj_runtime_param = Zed2iObjectDetectionRuntimeParameters()

    def open_cam(self, init_parameter) -> str:
        status = self.open(init_parameter)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit()
        print("Camera is openned.")
        return status