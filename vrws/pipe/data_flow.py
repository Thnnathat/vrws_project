from queue import Queue
import pyzed.sl as sl
from threading import Thread, Lock, Event

from vision.viewer import render_object
from vision.camera import Camera
from time import sleep

class DataFlow(Thread):
    def __init__(self, cam_data_event: Event, camera: Camera):
        super().__init__()
        self.queue = Queue(100)
        self.item = []
        self.item_weight = []
        self.had = False
        self.obj = None

        self.exit_signal: bool = False
        
        self.__lock: Lock = Lock()
        self.__cam_data_event: Event = cam_data_event

        self.camera: Camera = camera
        
        self.object_weight = {"red-cube": 4.0, "green-cube": 3.0, "blue-cube": 2.0,"yellow-cube": 1.0}

    def stop(self):
        self.exit_signal = True

    def run(self):
        while not self.exit_signal:
            self.insert_data_static(self.camera.objects, self.camera.obj_param.enable_tracking)
            sleep(0.5) #! Unlock

    def insert_data_static(self, objects, is_tracking_on):
        self.__cam_data_event.clear()
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
        self.__cam_data_event.set()
        
        ids = ""
        for i in self.item:
            ids += f"{i.id},"
        # print(ids, end="\r")

    def sorting(self):
        pass

    def provide_boject(self):
        pass
    
