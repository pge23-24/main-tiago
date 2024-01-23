# Import the YOLO class from the ultralytics library
from ultralytics import YOLO

# Global variable for the name of the model
# "yolov8s-pose.pt" is a pre-trained model for pose estimation
MODEL = "yolov8s-pose.pt"

# Define a class named 'yolo'
class yolo:
    def __init__(self):
        # Initialize a YOLO model with the specified weights file
        self.model = YOLO(MODEL)

    def compute(self, image):
        # This function processes an image using the YOLO model

        # Check if the image input is valid
        if image is not None:
            # Run YOLOv8 pose estimation on the provided image
            # 'persist=True' enables persistent tracking across frames
            # 'tracker="bytetrack.yaml"' specifies the tracking configuration to use
            results = self.model.track(image, persist=True, tracker="bytetrack.yaml")

            # Return the results of the model
            return results