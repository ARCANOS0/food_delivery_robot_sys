from time import sleep

import cv2 as cv
import time
import threading
# import torch
# import pandas
# TODO install torch package whenever the internet is available
# TODO 2 install pandas pack
from camera_feed import CameraFeed
# import the Logic to detect objects

class ObjectDetector :
    """
    حتحمل المودل ، حتشغله مع الكاميرا ، وحتوفرلنا UI for the results
    """

    def __init__(self, camera_feed: CameraFeed, model_weight_path: str,
                 conf_threshold: float = 0.25, iuo_threshold: float = 0.45):

        """
        @para 1 -> حياخد الكاميرا
        @para 2 -> الملف اللي فيه افضل النتايج / اسمه best.pt
        @para 3 -> النسبة اللي عندها المودل يقدر يقول ان اللي شافه ده عبارة عن شيء
        @para 4 -> the precentage in which the duplicated boxes are removed
         (the one with the higher % is kept)
        """

        self.camera_feed = camera_feed
        self.model_weight_path = model_weight_path
        self.conf_threshold = conf_threshold
        self.iuo_threshold = iuo_threshold

        # initialization ::
        self.model_version = None # هنا حنحط المودل بتاعنا او اي مودل عايزين نستخدمه
        self.object_name = [] # اسامي الحاجة اللي حنعملها detection
        self.most_recent_detect = [] # حنحط فيها اخر حاجة المودل شافها
        self.latest_frame_with_detections = None
        self.is_running = False # flag بيحدد حالة المودل اذا كان شغال او لا
        self.thread = None # thread for processing
        self.thread_lock = threading.Lock() #معناها ان خيط واحد بس ليه الصلاحية لاستخدام حاجة معينة في وقت معين ، عشان يمنع الdeadlock

        print(f"Loadin the model {self.model_version} from : {self.model_weight_path}")
        try :
            # self.model_version = torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_weights_path')
            # TODO train the custom model and import it
            self.model_conf = self.conf_threshold
            self.model_iuo = self.iuo_threshold
            # نسبة الثقة والدقة اللي حطيناها في الconstructor بنحطها هنا واحنا بنحمل المودل

            # self.object_name = self.model.names
            # TODO :: add the model.names
            print("YOLO was successfully Imported")
            print(f"Detected objects names are : {self.object_name}")

        except Exception as e :
            print(f"error loading the model from the path {self.model_weight_path}")
            self.model_version = None # رجعه زي ما كان حتى لو متغيرش عشان الاخطاء

    # start object detection func
    def start_detection(self) -> None:
        if self.model_version is None :
            print("cannot initialize object detector, please specify a model")
            return

        if not self.is_running :
            self.is_running = True
            self.thread = threading.Thread(target=self.detection_loop) # حيبدأ الخيط هنا في الfunc دي عشان يعالج الصور اللي حتجيله عن طريقها
            self.thread.start()
            print("Object detection thread started")

        else :
            print("thread is already threading")

    def detection_loop(self) :
        """
        takes frames from the camera_feed module
        """
        print("detection loop started")
        time.sleep(2)

        while self.is_running : #طول ما الis_running بــ true
            frame = self.camera_feed.get_latest_frame()

            if frame is None : #لو مفيش فرام وصله، يستنى سيكة
                time.sleep(0.05)
                continue

            try:
                # ندي الفريمات للمودل عشان يعالجها
                result = self.model_version(frame)
                data_frames = result.pandas().xyxy[0] # بتخلي الصور على شكل حاجة اسمها data frame ودي ليها علاقة بمكتبة pandas

                current_dtf = []

                for idx, row in data_frames.iterrow() :
                    if row['confidence'] >= self.conf_threshold :
                        current_dtf.append({
                            'class': row['name'],
                            'confidence': row['confidence'],
                            'box boundries': [ int(row['xmin']), int(row['ymin']), int(row['xmin']), int(row['ymax']) ]
                        })
                # drawing the results on the box to visualize them
                detailed_frame = frame.copy()
                for detected in current_dtf:
                    x1, y1, x2, y2 = detected['box boundries']
                    label = f"{detected['class']} {detected['confidence']} "
                    cv.rectangle(detailed_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    # the last 2 para are the color of the box and the thickness
                    cv.putText(detailed_frame, label, (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    # font scale, color, thickness

                with self.thread_lock :
                    self.most_recent_detect = current_dtf
                    self.latest_frame_with_detections = detailed_frame

            except Exception as e :
                print(f"Error, Not able to detect: {e}")

        print("object detection thread stopped")

    def get_latest_frame(self):
        with self.thread_lock :
            return self.most_recent_detect

    def get_latest_box(self):
        with self.thread_lock :
            if self.latest_frame_with_detections is not None :
                return  self.latest_frame_with_detections.copy()
            return None

    def stop(self):
        if self.is_running :
            self.is_running = False
            if self.thread and self.thread.is_alive() :
                self.thread.join()
                print("Obj detection thread joined")

if __name__ == "__main__" :
    print("-----------< main function has been called >-----------")
    print("\n")

    print("setting Camera Feed..")
    camera_feed = CameraFeed(camera_idx=0, camera_resolution=(640, 480), framerate=30)
    if camera_feed.capture is None or not camera_feed.capture.isOpend() :
        print("failed to initialize the camera feed\nEXITING ..")
        exit()

    camera_feed.start_capture()
    time.sleep(2) #sleep time to correctly set up the camera


# TODO the main function