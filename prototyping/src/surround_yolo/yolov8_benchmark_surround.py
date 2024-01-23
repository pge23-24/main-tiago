import cv2
from ultralytics import YOLO  # YOLOv8 import
from PIL import Image
import numpy as np
import time


class YOLOv8Detector:
    def __init__(self, model_name):
        # Load YOLOv8 model
        self.model = YOLO(model_name)

    def process_and_save_video(self, input_video_path, output_video_path, model_name):
        # Open the input video
        cap = cv2.VideoCapture(input_video_path)
        if not cap.isOpened():
            print("Error opening video file")
            return

        # Get video properties
        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)

        # Define the codec and initialize the video writer
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        out = cv2.VideoWriter(
            output_video_path, fourcc, fps, (frame_width, frame_height)
        )

        frame_times = []

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # Start time
            start = time.time()

            # Process the frame (detection and drawing bounding boxes)
            annotated_frame = self.process_frame(frame)

            # End time
            end = time.time()
            frame_times.append(end - start)

            # Write the frame into the output video file
            out.write(annotated_frame)

        # Release everything when job is finished
        cap.release()
        out.release()

        avg_time_per_frame = sum(frame_times) / len(frame_times)
        print(
            f"Average time per frame for {model_name}: {avg_time_per_frame:.4f} seconds"
        )

    def process_frame(self, frame):
        """Process a single frame for object detection and return annotated frame"""
        frame_pil = Image.fromarray(frame)
        results = self.model.track(
            source=frame_pil,
            persist=True,
            tracker="bytetrack.yaml",
            conf=0.5,
            verbose=False,
        )

        # Convert PIL image back to OpenCV format
        annotated_frame = cv2.cvtColor(np.array(results[0].plot()), cv2.COLOR_RGB2BGR)
        return annotated_frame


if __name__ == "__main__":
    models = ["yolov8n.pt", "yolov8s.pt", "yolov8m.pt", "yolov8l.pt", "yolov8x.pt"]

    # Define input and output video paths
    input_video_path = "assets/VIDEOS/COMBINED_VIDEO_USE_CASE_2.mp4"

    for model in models:
        output_video_path = f"assets/VIDEOS/COMBINED_VIDEO_USE_CASE_2_YOLO_{model}.mp4"
        detector = YOLOv8Detector(model)
        detector.process_and_save_video(input_video_path, output_video_path, model)
