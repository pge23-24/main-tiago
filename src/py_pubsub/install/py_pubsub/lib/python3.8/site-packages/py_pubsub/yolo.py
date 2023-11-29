import cv2
from ultralytics import YOLO  # YOLOv8 import
from PIL import Image



class yolo:
    def __init__(self):
        self.model = YOLO("yolov8s-pose.pt")
    def compute(self, image):
        # Check if the image was successfully loaded
        if image is not None:
            # Run YOLOv8 tracking on the image
            results = self.model.track(image, persist=True, tracker="bytetrack.yaml")

            return results
        

if __name__ == "__main__":
    instance = yolo()
    image = cv2.imread("test.jpg")
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = instance.compute(image)

    print(results)

    # Visualize the results on the image
    annotated_image = results[0].plot()
    # Display the annotated image
    cv2.imshow("YOLOv8 Tracking", annotated_image)

    # Wait for a key press before closing the window
    cv2.waitKey(0)

    # Close the display window
    cv2.destroyAllWindows()