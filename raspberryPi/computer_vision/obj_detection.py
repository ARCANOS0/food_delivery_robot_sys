# raspberry_pi/computer_vision/object_detector.py

import threading as thrd
import cv2 as cv
import time
import torch  # For loading and running PyTorch-based YOLOv5 models
import numpy as np

from .camera_feed import CameraFeed


class ObjectDetector:
    """
    Handles loading the YOLOv5 model, running inference on camera frames,
    and providing detection results.
    """

    def __init__(self,
                 camera_feed: CameraFeed,  # Takes an instance of CameraFeed
                 model_weights_path: str,  # Path to your trained model weights (e.g., 'weights/best.pt')
                 conf_threshold: float = 0.25,  # Confidence threshold for detections
                 iou_threshold: float = 0.45  # IoU threshold for Non-Maximum Suppression
):
        """
        Initializes the object detector.

        Args:
            camera_feed (CameraFeed): An initialized CameraFeed instance.
            model_weights_path (str): Path to the trained YOLOv5 model file (.pt).
            conf_threshold (float): Minimum confidence to consider a detection.
            iou_threshold (float): IoU threshold for Non-Maximum Suppression.
        """
        self.camera_feed = camera_feed
        self.model_weights_path = model_weights_path
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold

        self.model = None  # Variable to hold the loaded YOLOv5 model
        self.class_names = []  # List to hold the class names (e.g., ['table_A', 'table_B'])

        self.latest_detections = []  # List to store the results of the latest inference
        self.latest_frame_with_detections = None  # Optional: store the frame with boxes drawn

        self._running = False  # Flag to control the detection thread loop
        self._thread = None  # Variable to hold the detection thread object
        self._lock = threading.Lock()  # Lock for safely accessing shared variables (results, frame)

        # --- Load the model ---
        print(f"Attempting to load model from: {self.model_weights_path}")
        try:
            # Load the YOLOv5 model from a local path
            # 'custom' tells torch.hub to load from a local path instead of a repo URL
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_weights_path)
            # Set confidence and IoU thresholds directly on the model if using this load method
            self.model.conf = self.conf_threshold
            self.model.iou = self.iou_threshold

            # Get class names from the loaded model
            self.class_names = self.model.names
            print("YOLOv5 model loaded successfully.")
            print(f"Detected classes: {self.class_names}")

        except Exception as e:
            print(f"Error loading YOLOv5 model from {self.model_weights_path}: {e}")
            self.model = None  # Ensure model is None if loading failed

    def start(self):
        """Starts the separate thread to continuously perform object detection."""
        if self.model is None:
            print("Cannot start object detector: Model not loaded.")
            return

        if not self._running:
            self._running = True
            # Create a thread that runs the _detection_loop method
            self._thread = thrd.Thread(target=self._detection_loop)
            self._thread.daemon = True  # Allow the main program to exit even if this thread is running
            self._thread.start()  # Start the thread
            print("Object detector thread started.")
        else:
            print("Object detector thread is already running.")

    def _detection_loop(self):
        """
        Internal method run by the thread to get frames and run inference.
        """
        print("Object detection thread started.")
        # Give camera feed a moment to start before requesting first frame
        # time.sleep(0.5) # Already handled by delay after camera.start_capture()

        while self._running:
            # 1. Get the latest frame from the camera feed
            # get_latest_frame() should return a COPY, safe to modify
            frame = self.camera_feed.get_latest_frame()

            if frame is None:
                # print("Detector: No frame received from camera feed.") # Avoid spamming if camera is slow to start
                time.sleep(0.05)  # Wait a bit before trying again
                continue  # Skip inference if no frame available

            # 2. Perform object detection inference
            try:
                # Pass the frame to the YOLOv5 model
                # results is a list of detection objects
                results = self.model(frame)

                # 3. Process the results
                # results.pandas().xyxy[0] gives a pandas DataFrame for the first image in the batch
                detections_df = results.pandas().xyxy[0]

                current_detections = []
                # Iterate through detections and store in a structured format
                for index, row in detections_df.iterrows():
                    # Filter by confidence and IoU thresholds (though already set on model, good to be explicit)
                    if row['confidence'] >= self.conf_threshold:  # NMS already applied by model
                        current_detections.append({
                            'class': row['name'],  # Class name (e.g., 'cup', 'table_A')
                            'confidence': row['confidence'],  # Confidence score (0.0-1.0)
                            'box': [int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])]
                            # Bounding box [x1, y1, x2, y2]
                        })

                # Optional: Draw results on the frame for visualization
                frame_with_boxes = frame.copy()  # Draw on a copy
                for det in current_detections:
                    x1, y1, x2, y2 = det['box']
                    label = f"{det['class']} {det['confidence']:.2f}"
                    cv.rectangle(frame_with_boxes, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green box
                    cv.putText(frame_with_boxes, label, (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # 4. Store the processed results and optionally the frame with drawings
                with self._lock:  # Use lock when accessing shared variables
                    self.latest_detections = current_detections
                    self.latest_frame_with_detections = frame_with_boxes  # Store the frame with boxes


            except Exception as e:
                print(f"Error during object detection inference: {e}")
                # Depending on the error, you might set _running = False to stop

            # Add a small sleep if needed to control detection rate,
            # or remove if you want to process as fast as frames are available.
            # time.sleep(0.01)

        print("Object detection thread stopped.")

    def get_latest_detections(self):
        """
        Returns the list of latest detected objects.
        Each object is a dictionary with 'class', 'confidence', and 'box'.
        Returns an empty list if no detections or model not loaded/running.
        """
        with self._lock:  # Use lock when accessing shared shared variables
            return self.latest_detections  # Return the stored list (it's a copy from loop assignment)

    def get_latest_frame_with_detections(self):
        """
        Returns the most recent frame with detection bounding boxes drawn on it.
        Returns None if no frame processed or not enabled.
        """
        with self._lock:  # Use lock
            if self.latest_frame_with_detections is not None:
                return self.latest_frame_with_detections.copy()  # Return a copy
            return None

    def stop(self):
        """
        Stops the detection thread.
        """
        if self._running:
            self._running = False  # Signal the thread to stop
            if self._thread and self._thread.is_alive():
                self._thread.join()  # Wait for the thread to finish
                print("Object detector thread joined.")


# --- Example Usage (for testing this file directly) ---
if __name__ == "__main__":
    print("--- Starting Object Detector Test ---")

    # --- Step 1: Setup Camera Feed (required by ObjectDetector) ---
    # Ensure CameraFeed.py is in the same directory or accessible
    print("Setting up CameraFeed...")
    # Use camera_idx=0 for laptop webcam, or potentially different index/parameters for RPi Cam Module 3
    camera_feed = CameraFeed(camera_idx=0, resolution=(640, 480), framerate=30)
    if camera_feed.capture is None or not camera_feed.capture.isOpened():
        print("Failed to open camera for testing ObjectDetector. Exiting.")
        exit()  # Exit if camera fails

    camera_feed.start_capture()
    print("CameraFeed started.")

    # Give CameraFeed a moment to get initial frames
    time.sleep(1.5)
    print("Waited for CameraFeed.")

    # --- Step 2: Setup Object Detector ---
    # !! IMPORTANT !! Replace 'path/to/your/best.pt' with the actual path to your trained YOLOv5 model file
    # For testing with a default model, you could use 'yolov5s.pt' after downloading it or
    # comment out the path and modify the init to load a pretrained model like:
    # self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    model_path = 'path/to/your/best.pt'  # <-- CHANGE THIS PATH

    # Alternatively, for testing without a custom model, uncomment below and comment above
    # model_path = 'yolov5s' # Loads the 'small' pre-trained YOLOv5 model (requires internet first run)
    # is_pretrained = True # Flag if loading pretrained

    # If using a pretrained model like yolov5s:
    if model_path == 'yolov5s':
        # Need to load pretrained differently if not using a file path
        # This requires internet access for the first run
        detector = ObjectDetector(camera_feed=camera_feed, model_weights_path='yolov5s')  # This will load pretrained
        detector.model = torch.hub.load('ultralytics/yolov5', model_path, pretrained=True)
        # Manually set thresholds if loaded this way, get names
        detector.model.conf = detector.conf_threshold
        detector.model.iou = detector.iou_threshold
        detector.class_names = detector.model.names
        print("Loaded pretrained model.")

    else:
        # Load from a local file path
        detector = ObjectDetector(camera_feed=camera_feed, model_weights_path=model_path)  # This expects a .pt file

    if detector.model is None:
        print("Object detector model failed to load. Exiting test.")
        camera_feed.stop()
        exit()

    detector.start()
    print("Object detector started.")

    # --- Step 3: Run Main Loop to Get & Display Results ---
    try:
        print("Entering main display loop (Press 'q' to quit)...")
        while True:
            # Get the latest frame with detections drawn on it
            frame_to_display = detector.get_latest_frame_with_detections()

            if frame_to_display is not None:
                cv.imshow("Object Detection Result", frame_to_display)

                # Get the latest detection results separately (for use in navigation logic)
                detections = detector.get_latest_detections()
                # print(f"Latest Detections: {detections}") # Uncomment to see the raw detection data

                # Handle user input to quit
                key = cv.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("Exiting loop on 'q' press.")
                    break
            else:
                # print("No frame with detections available yet.")
                time.sleep(0.01)  # Sleep briefly if no frame is ready yet


    except KeyboardInterrupt:
        print("\nTest interrupted by user (Ctrl+C).")

    except Exception as e:
        print(f"An error occurred in the main test loop: {e}")

    finally:
        # --- Step 4: Clean Shutdown ---
        print("Shutting down...")
        detector.stop()  # Stop the detector thread first
        camera_feed.stop()  # Stop the camera feed thread
        cv.destroyAllWindows()  # Close display windows
        print("--- Object Detector Test Finished ---")