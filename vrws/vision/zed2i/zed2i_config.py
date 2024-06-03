from pyzed.sl import InitParameters, RuntimeParameters, PositionalTrackingParameters, ObjectDetectionParameters, ObjectDetectionRuntimeParameters
import pyzed.sl as sl

class Zed2iInitParameters(InitParameters):
    def __init__(self) -> None:
        super().__init__()
        # Create a InitParameters object and set configuration parameters
        self.depth_mode = sl.DEPTH_MODE.NEURAL  # QUALITY
        self.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        # self.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
        self.camera_resolution = sl.RESOLUTION.HD1080
        self.depth_maximum_distance = 2000
        self.depth_minimum_distance = 200
        self.camera_fps = 60
        self.coordinate_units = sl.UNIT.MILLIMETER

        self.display()
    
    def display(self) -> None:
        print("Zed's configuration is ready.")


class Zed2iPositionalTrackingParameters(PositionalTrackingParameters):
    def __init__(self) -> None:
        super().__init__()
            # If the camera is static, uncomment the following line to have better performances and boxes sticked to the ground.
        # ? ใช้เมื่อกกล้องไม่อยู่นิ่ง
        self.set_as_static = True
        self.set_gravity_as_origin = True # ! “horizontal plane” parallel to the ground.
        # positional_tracking_parameters.set_floor_as_origin = True;

class Zed2iObjectDetectionParameters(ObjectDetectionParameters):
    def __init__(self) -> None:
        super().__init__()
        self.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        self.enable_tracking = True

# Runtime ปรับค่าแบบ Realtime
class Zed2iRuntimeParameters(RuntimeParameters):
    def __init__(self) -> None:
        super().__init__()
        self.measure3D_reference_frame = sl.REFERENCE_FRAME.WORLD # ! “horizontal plane” parallel to the ground.

class Zed2iObjectDetectionRuntimeParameters(ObjectDetectionRuntimeParameters):
    def __init__(self) -> None:
        super().__init__()

