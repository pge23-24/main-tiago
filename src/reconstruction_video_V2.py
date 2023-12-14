import os
import cv2
from natsort import natsorted
from ultralytics import YOLO


class Yolo:

    def __init__(self, model, confidence=0.6):
        # Initialize a YOLO model with the specified weights file
        self.model = YOLO(model)
        self.confidence_threshold = confidence

    def compute(self, image):
        # This function processes an image using the YOLO model

        # Check if the image input is valid
        if image is not None:
            # Run YOLOv8 pose estimation on the provided image
            # 'persist=True' enables persistent tracking across frames
            # 'tracker="bytetrack.yaml"' specifies the tracking configuration to use
            results = self.model.track(image, persist=True, tracker="bytetrack.yaml", conf=self.confidence_threshold)

            # Return the filtered results
            return results


class Reconstruction:

    def video_detection(self, path_folder_image, path_save, frame_width=1226, frame_height=968):
        path_sorted = natsorted(os.listdir(path_folder_image))
        out = cv2.VideoWriter(path_save, cv2.VideoWriter_fourcc(*'mp4v'), 24, (frame_width, frame_height))
        for file in path_sorted:
            path_file = path_file = path_folder_image + "/" + file
            img = cv2.imread(path_file)
            if img is not None and img.shape[:2] == (frame_height, frame_width):
                out.write(img)
            else:
                print(f"Error: Invalid image size or format - {path_file}")
        out.release()

    def video_detection_yolov8(self, nb_use_case, frame_width=1226, frame_height=968, threshold=0.70):
        models = ["yolov8s", "yolov8m", "yolov8l", "yolov8x"]

        for m in models:
            yolov8 = Yolo(m + ".pt", threshold)
            for i in range(1, nb_use_case + 1):
                for c in range(1, 5):
                    path_folder_image = "./USE_CASES/USE_CASE_" + str(i) + "/CAMERA_" + str(c)
                    path_save = "./VIDEO_" + m + "_USE_CASES/VIDEO_" + m + "_USE_CASE_" + str(i) + "/CAMERA_" + str(c)
                    os.system("mkdir " + path_save)
                    path_save = path_save + "/vid.mp4"
                    path_sorted = natsorted(os.listdir(path_folder_image))
                    out = cv2.VideoWriter(path_save, cv2.VideoWriter_fourcc(*'mp4v'), 24, (frame_width, frame_height))
                    for file in path_sorted:
                        path_file = path_folder_image + "/" + file
                        img = cv2.imread(path_file)
                        results = yolov8.compute(image=img)
                        if results:
                            annotated_image = results[0].plot()
                            if img is not None and img.shape[:2] == (frame_height, frame_width):
                                out.write(annotated_image)
                            else:
                                print(f"Error: Invalid image size or format - {path_file}")
                    out.release()
                    print(f"La vidéo du UseCase {i} est enregistrée sous : {path_save}")


if __name__ == '__main__':

    rec = Reconstruction()
    """for i in range(1, nb_use_case + 1):
        prefix = "./USE_CASES/USE_CASE_" + str(i)
        save_prefix = "./VIDEO_USE_CASES/VIDEO_USE_CASE_" + str(i)
        video_detection(prefix + "/CAMERA_1", save_prefix + "/CAMERA_1/vid.mp4")
        video_detection(prefix + "/CAMERA_2", save_prefix + "/CAMERA_2/vid.mp4")
        video_detection(prefix + "/CAMERA_3", save_prefix + "/CAMERA_3/vid.mp4")
        video_detection(prefix + "/CAMERA_4", save_prefix + "/CAMERA_4/vid.mp4")
    print(f"La vidéo du UseCase {i} est enregistrée sous : {save_prefix}")"""

    rec.video_detection_yolov8(1)

