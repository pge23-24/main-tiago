from ultralytics import YOLO  # YOLOv8 import
from ultralytics.utils import LOGGER  # LOGGER import
import torch

models_path = "models/"


class YOLOv8Detector:
    type = "YOLOv8"

    def __init__(self):
        try:
            self.model = YOLO(f"{models_path}best.pt")
        except Exception as e:
            LOGGER.error(f"Error loading default YOLOv8 model. {e}")

    def compute(self, image):
        if image is not None:
            results = self.model.track(
                image, conf=0.3, persist=True, tracker="bytetrack.yaml", classes=[0, 80]
            )
            return results


class YOLOv5Detector:
    type = "YOLOv5"

    def __init__(self, model_name=f"{models_path}/yolv5l_custom.pt"):
        # Load YOLOv5 model
        try:
            self.model = torch.hub.load(
                "ultralytics/yolov5",
                "custom",
                path=f"{models_path}yolv5l_custom.pt",
            )
        except Exception as e:
            LOGGER.error(
                f"Error loading custom YOLOv5 model. {e} \nLoading default model."
            )
            try:
                self.model = YOLO(model_name)
            except Exception as e:
                LOGGER.error(f"Error loading default YOLOv5 model. {e}")
                self.model = None

    def compute(self, image):
        """Process a single image"""
        if image is not None:
            results = self.model(image)
            return results
