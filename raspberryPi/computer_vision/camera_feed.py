import threading as thrd
import cv2 as cv
import time

class CameraFeed :
    """
    this is a class that takes the camera input and then we will use it to distribute this input among the
    scripts that needs the camera input
    حنستعمل الكلاس ده عشان ناخد الصور والفيديو input ونوزعها على بقية الفايلات اللي محتاجة البيانات دي عشان تشتغل
    """
    def __init__(self, camera_idx = 1, camera_resolution = (640, 480), framerate = 30):
        """
        اول parameter عبارة عن index of the camera للي الfunction اللي اسمها VideoCapture بتاخدها ، الطبيعي بتاعها ان قيمتها بتبقى صفر
         ، لو حطينا كاميرا تانيو حتبقى 1 وهكذا الرقم ده بيتغير على خسب عدد الكاميرات

         تاني parameter هي جودة الفيديو اللي بيطلع من الكاميرا والتالت هي عدد الفريمات في الثانية الواحدة اللي ممكن يطلع من الكاميرا، غالبا ده أقصى رقم
        """
        self.camera_idx = camera_idx
        self.camera_resolution = camera_resolution
        self.framerate = framerate

        # variables to hold the camera feed
        self.most_recent_frame = None # a variable to hold the frame, None means that now it doesn't have the value
        self.is_running = False # indication flag to say "YES the camera is working bro!"
        self.thread = None # the thread in the processor that processes the data coming from the camera

        # initialize the camera feed
        self.capture = cv.VideoCapture(camera_idx)
        


        # see if the camera is available
        if self.capture.isOpened() :
            # لو الكاميرا تمام شغالة ، حنعمل مربع بالمقاسات اللي حطيناها في الcamera resolution فوق
            self.capture.set(cv.CAP_PROP_FRAME_WIDTH, camera_resolution[0]) # width of the window 640px
            self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, camera_resolution[1]) # height of the window 480px
            self.capture.set(cv.CAP_PROP_FPS, framerate) # set the frame rate to 30fps (frame per second)
            # test
            print(f"Camera resolution set to: {self.capture.get(cv.CAP_PROP_FRAME_WIDTH)}x{self.capture.get(cv.CAP_PROP_FRAME_HEIGHT)}")
        else :
            print(f"cant open the camera at the index {self.camera_idx}")
            self.capture = None

    #  start camera feed function
    def start_capture(self) :
        # حنشوف لو الكاميرا شغالة ولا لأ، ولو في ثريد بيعمل معالجة لبيانات الكاميرا ولا لأ
        if not self.capture.isOpened() or self.capture is None :
            print("the camera is not running or not opened or not available ")
            return # اطلع برا الfunction لو الكاميرا مش شغالة

        # لو الكاميرا مزبوط بس عايزين نشغلها
        if not self.is_running :
            self.is_running = True
            self.thread = thrd.Thread(target=self.capture_loop) # حيعمل خيط معالجة على الfunction اللي اسمها capture loop عشان يفضل ياخد منها بيانات وقراية من الكاميرا
            self.daemon = True
            self.thread.start() # start the thread

            print("camera feed initialized!")

        else :
            print("camera feed is already running!")


# camera feed input 
    def capture_loop(self) :
        print("Capture loop thread started.") # Added print
        while self.is_running and self.capture.isOpened() :
            ret, frame = self.capture.read()
            # print(f"Read result: ret={ret}") # Optional: print ret value every loop

            if not ret :
                print("Error : failed to read the input (ret is False)") # Modified print
                # Optionally, add a tiny sleep here to avoid spamming if it fails immediately
                # time.sleep(0.05)
                # Do NOT set is_running = False or break *yet* - let's see if it recovers
                continue # Skip the rest of the loop iteration if read failed
            else:
                # print("Successfully read frame.") # Optional print
                self.most_recent_frame = frame # store the captured frame as the most recent frame
                # No time.sleep(0.01) needed here usually, read() is often blocking enough
        print("camera stopped (loop condition became false)") # Modified print

    def get_latest_frame(self) :
        if self.most_recent_frame is not None :
            return self.most_recent_frame.copy()
        return None

    def stop(self):
        """
        Stops the capture thread and releases the camera resources.
        """
        if self.is_running:
            self.is_running = False  # Signal the thread to stop
            if self.thread and self.thread.is_alive():
                self.thread.join()  # Wait for the thread to finish gracefully
                print("Camera capture thread joined.")

        if self.capture is not None and self.capture.isOpened():
            self.capture.release()  # Release the camera hardware
            print("Camera resources released.")

    def __del__(self):
        """Ensures resources are released when the object is deleted."""
        self.stop()



if __name__ == "__main__" :
    print("--- Starting Camera Feed Test (Infinite Loop) ---") # Updated start message
    camera_feed = CameraFeed(camera_idx=0, camera_resolution=(640, 480), framerate=30)

    print("CameraFeed object created.")
    if camera_feed.capture is not None and camera_feed.capture.isOpened():
        print("Camera successfully opened by VideoCapture.")
    else:
        print("Failed to open camera via VideoCapture.")
        exit() # Use exit() to stop here if camera not opened

    camera_feed.start_capture()
    print("Camera feed start_capture() called.")

    # Give the capture thread a moment to potentially get the first frame
    time.sleep(1)
    print("Waited 1 second for camera thread.")

    try:
        print("Entering main test loop (Press 'q' to quit)...")
        
        while True:  

            frame = camera_feed.get_latest_frame()

            if frame is not None:
                try:
                    cv.imshow("Camera Feed Test", frame)
                    key = cv.waitKey(1) & 0xFF

                    if key == ord('q'):
                        print("Exiting loop on 'q' press")
                        break # Exit the while True loop
                except Exception as display_error:
                    print(f"Caught an error during imshow/waitKey: {display_error}")
                    break # Exit the loop if display fails

            else:
                 # This warning should only happen if get_latest_frame() returns None
                 # If this repeats, it means capture_loop is failing silently or not running
                 print(f"Warning: Frame is None, cannot display.")
                 # Consider adding a counter here to break if it's always None for too long
                 # if count_none_frames > some_threshold: break

            # Optional: Add a small sleep here if needed, but waitKey(1) usually handles timing
            # time.sleep(0.001) 

        print("Main test loop finished.") # Print after the while loop exits

    except KeyboardInterrupt:
        print("Test interrupted by user (Ctrl+C).")

    except Exception as main_error:
        # This will catch any other unhandled exceptions in the main try block
        print(f"Caught an unexpected error in main execution: {main_error}")

    finally:
        # This block always runs when the try or except is finished/exited
        print("Executing finally block...") 
        camera_feed.stop() 
        cv.destroyAllWindows()
        print("--- CameraFeed test finished ---")

