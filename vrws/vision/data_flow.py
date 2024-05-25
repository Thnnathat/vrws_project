from queue import Queue
from vision.viewer.utils import render_object
import pyzed.sl as sl
class DataFlow:
    def __init__(self):
        self.queue = Queue(100)
        self.item = []
        self.item_weight = []
        self.had = False
        self.obj = None
        
        self.object_weight = {"red-cube": 4.0, "green-cube": 3.0, "blue-cube": 2.0,"yellow-cube": 1.0}

    def insert_data_static(self, objects, is_tracking_on):
        for obj in objects.object_list:
            if render_object(obj, is_tracking_on):
                if obj.action_state == sl.OBJECT_ACTION_STATE.IDLE:
                    self.obj = obj
                for item in self.item:
                    self.had = False
                    if obj.id == item.id:
                        self.had = True
                        break
                if not self.had and obj.action_state == sl.OBJECT_ACTION_STATE.IDLE:
                    self.item.append(obj)
            else:
                self.obj = None
        ids = ""
        for i in self.item:
            ids += f"{i.id},"
        print(ids, end="\r")

    def sorting(self):
        pass

    def provide_boject(self):
        pass
    
