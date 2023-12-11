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

        if not processing_times:
            return np.nan, np.nan, [np.nan, np.nan]

        return (
            np.mean(processing_times),
            np.median(processing_times),
            np.percentile(processing_times, [25, 75]),
        )

    def process_image(self, image_path):
        """Process a single image"""
        frame = cv2.imread(image_path)
        frame_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        results = self.model.predict(source=frame_pil)


if __name__ == "__main__":
    # Define the models to iterate over
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

    # Iterate over each model and process images from all folders
    for model in models:
        detector = YOLOv8Detector(model)
        all_times = []
        for folder in folders:
            mean_time, median_time, quartiles = detector.process_images_from_folder(
                folder
            )
            all_times.extend(quartiles)  # Extend the list with quartile times

        # Compute overall statistics across all folders
        overall_mean = np.mean(all_times)
        overall_median = np.median(all_times)
        overall_quartiles = np.percentile(all_times, [25, 75])

        results_df = results_df.append(
            {
                "Model": model,
                "Mean Time": overall_mean,
                "Median Time": overall_median,
                "25th Percentile": overall_quartiles[0],
                "75th Percentile": overall_quartiles[1],
            },
            ignore_index=True,
        )

    # Save results to CSV
    results_df.to_csv("yolov5_timing_statistics_overall.csv", index=False)
