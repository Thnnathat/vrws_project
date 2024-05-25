from detector import Detector
from robot_arm import RobotArm
from display import Display

if __name__ == "__main__":
    det = Detector()
    det.start()
    
    disp = Display(det)
    disp.start()

    # robot = RobotArm(det)
    # robot.start()