from ultralytics import YOLO

class yolo:
    def __init__(self):
        self.model = YOLO("yolov8s-pose.pt")
    def compute(self, image):
        # Check if the image was successfully loaded
        if image is not None:
            # Run YOLOv8 tracking on the image
            results = self.model.track(image, persist=True, tracker="bytetrack.yaml")

            return results