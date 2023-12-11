import cv2
from ultralytics import YOLO  # YOLOv8 import
from PIL import Image
import os
import time
import numpy as np
import pandas as pd


class YOLOv8Detector:
    def __init__(self, model_name):
        # Load YOLOv8 model
        self.model = YOLO(model_name)

    def process_images_from_folder(self, folder_path):
        """Process all images in the specified folder and calculate timing statistics"""
        processing_times = []

        for image_file in os.listdir(folder_path):
            image_path = os.path.join(folder_path, image_file)
            if os.path.isfile(image_path) and image_file.lower().endswith(
                (".png", ".jpg", ".jpeg", ".bmp", ".tiff", ".gif")
            ):
                start_time = time.time()
                self.process_image(image_path)
                end_time = time.time()
                processing_times.append(end_time - start_time)

        return processing_times

    def process_image(self, image_path):
        """Process a single image"""
        frame = cv2.imread(image_path)
        frame_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        results = self.model.predict(source=frame_pil)


if __name__ == "__main__":
    # Define the models to iterate over (assuming YOLOv8 has similar variants)
    models = ["yolov8s.pt", "yolov8m.pt", "yolov8l.pt", "yolov8x.pt"]

    # Define the folders containing images
    folders = [
        "assets/DATA/CAMERA_1",
        "assets/DATA/CAMERA_2",
        "assets/DATA/CAMERA_3",
        "assets/DATA/CAMERA_4",
        "assets/DATA_2/CAMERA_1",
        "assets/DATA_2/CAMERA_2",
        "assets/DATA_2/CAMERA_3",
        "assets/DATA_2/CAMERA_4",
    ]

    # Initialize a DataFrame to store the results
    results_df = pd.DataFrame(
        columns=[
            "Model",
            "Mean Time",
            "Median Time",
            "25th Percentile",
            "75th Percentile",
        ]
    )

    # Iterate over each model and process images
    for model in models:
        all_times = []
        detector = YOLOv8Detector(model)
        for folder in folders:
            all_times.extend(detector.process_images_from_folder(folder))

        if not all_times:
            continue

        mean_time = np.mean(all_times)
        median_time = np.median(all_times)
        quartiles = np.percentile(all_times, [25, 75])

        results_df = results_df.append(
            {
                "Model": model,
                "Mean Time": mean_time,
                "Median Time": median_time,
                "25th Percentile": quartiles[0],
                "75th Percentile": quartiles[1],
            },
            ignore_index=True,
        )

    # Save results to CSV
    results_df.to_csv("yolov8_timing_statistics.csv", index=False)
