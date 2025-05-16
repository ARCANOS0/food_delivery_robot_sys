import threading as thrd 
import cv2 as cv 
import time 
import torch # to run the YOLO model 
import numpy as np 

from .camera_feed import CameraFeed


class ObjectDetection : 
    """
    loads the model, display the results and distrbute the camera feed 
    among the other parts of the system 
    """

    def __init__(self, camera_feed: CameraFeed, model_weights_path: str, iuo_threshold: float, iou_confidence: float) :
        """
        @parameter camera feed :: the camera object 
        @para model wieghts :: the best.pt file path 
        @para confidence :: the minimum confidence 
        @para iuo_confidence :: th e
        """
