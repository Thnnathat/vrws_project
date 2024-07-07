from time import sleep
from .dobot_api_v3 import DobotApiDashboard
class Gripper1:
    def __init__(self, dashboard) -> None:
        self.dashboard: DobotApiDashboard = dashboard
    
    def grip_close(self):
        self.dashboard.ToolDO(1, 0)
        self.dashboard.ToolDO(2, 1)
        sleep(0.1)

    def grip_open(self):
        self.dashboard.ToolDO(1, 0)
        self.dashboard.ToolDO(2, 0)
        sleep(0.1)
        
if __name__ == "__main__":
    from dobotcr5apiv3 import DobotCR5ApiV3
    from dobot_api_v3 import DobotApiDashboard
    dobot = DobotCR5ApiV3()
    gripper = Gripper1(dobot.dashboard)
    
    for i in range(10):
        gripper.grip_close()
        sleep(1)
        gripper.grip_open()
        sleep(1)

    dobot.dashboard.DisableRobot()