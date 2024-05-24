from pyzed.sl import Objects
class ObjectRecord(Objects):
    def __init__(self) -> None:
        super().__init__()
    def __str__(self) -> str:
        return "{}, {}, {}, {}, {}, {}, {}, {}".format(self.id, self.cls, self.accuracy, self.x, self.y, self.z, self.valocity, self.timestamp)