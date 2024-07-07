from .dobot_api_v4 import DobotApiDashboard, DobotApiFeedBack, DobotApiDashMove

# class DobotCR5ApiV4:
#     def __init__(self, ip: str) -> None:
#         self.ip: str = ip
#         self.dashboardPort: int = 29999
#         self.feedPort: int = 30004
#         self.movePort: int = 30005
        
#         self.is_connected: bool = False
#         self.exit_signal: bool = False

#         # เชื่อมต่อ Robot
#         self.dashboard: DobotApiDashboard | None = None
#         self.feed: DobotApiFeedBack | None = None
#         self.move: DobotApiDashMove | None = None

#         self.t_clearError = Thread(target=self.ClearRobotError)

#     def Connect(self):
#         self.dashboard = DobotApiDashboard(self.ip, self.dashboardPort)
#         self.feed = DobotApiFeedBack(self.ip, self.feedPort)
#         self.move = DobotApiDashMove(self.ip, self.movePort)
#         self.is_connected = True
#         self.t_clearError.start()
#         print("Dobot connection error.")
        
#     def Enable(self):
#         enableState = self.parseResultId(self.dashboard.EnableRobot())
#         if enableState[0] != 0:
#             print("Failed to enable: Check whether port 29999 is occupied.")
#             return
#         print("Enabled successfully!")

#     def RunToPoint(self, point_list: list[float], coordinateMode: int):
#         while True:
#             p2id = self.RunPoint(point_list, coordinateMode)        
#             if p2id[0] == 0: # ไม่มี error (ถ้ามี error จะทำการส่งคำสั่งใหม่เรื่อยๆ)
#                 self.move.WaitArrive(p2id[1]) # รอให้แขกลถึงตำแหน่ง
#                 break

#     def RunPoint(self, point_list: list[float], coordinateMode: int):
#         recv_movemess = self.dashboard.MovJ(point_list[0], point_list[1], point_list[2], point_list[3], point_list[4], point_list[5], coordinateMode) # P = [x, y, z, rx, ry, rz]
#         commandArrID = self.parseResultId(recv_movemess)
#         return commandArrID

#     def ClearRobotError(self):
#         while not self.exit_signal:
#             self.dashboard.ClearError()
#             sleep(5)

#     def parseResultId(self, valueRecv):
#         if valueRecv.find("Not Tcp") != -1:
#             print("Control Mode Is Not Tcp")
#             return [1]
#         recvData = re.findall(r'-?\d+', valueRecv)
#         recvData = [int(num) for num in recvData] # ! ตำแหน่ง
#         if len(recvData) == 0:
#             return [2]
#         return recvData