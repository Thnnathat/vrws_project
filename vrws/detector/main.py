from vrws.detector.detector import Zed2i
from robot_arm import RobotArm

if __name__ == "__main__":
    zed = None
    zed = Zed2i()
    zed.start()

    robot = RobotArm(zed)
    robot.start()