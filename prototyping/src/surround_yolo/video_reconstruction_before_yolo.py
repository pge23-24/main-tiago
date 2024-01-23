import os
import cv2
from natsort import natsorted
from src.surround_yolo.ImageUndistorter import ImageUndistorter
import numpy as np


class Reconstruction:
    def __init__(self):
        self.undistorter = ImageUndistorter(
            np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]]),
            np.array([0.1, -0.25, 0.001, 0.001, 0.0]),
        )

    def video_reconstruction(self, nb_use_case, frame_width=1226, frame_height=968):
        for i in range(1, nb_use_case + 1):
            video_streams = []
            for c in [4, 1, 3, 2]:
                path_folder_image = (
                    "./assets/USE_CASES/USE_CASE_" + str(i) + "/CAMERA_" + str(c)
                )
                path_sorted = natsorted(os.listdir(path_folder_image))
                video_frames = [
                    cv2.imread(path_folder_image + "/" + file)
                    for file in path_sorted
                    if file.endswith(".jpg") or file.endswith(".png")
                ]
                """video_frames = [self.undistorter.undistort(
                    cv2.imread(path_folder_image + "/" + file))
                    for file in path_sorted
                    if file.endswith(".jpg") or file.endswith(".png")
                ]"""
                video_streams.append(video_frames)

            # Ensure all streams have the same number of frames
            min_length = min([len(stream) for stream in video_streams])
            video_streams = [stream[:min_length] for stream in video_streams]

            # Create a VideoWriter for the output video
            path_save = f"./assets/VIDEOS/COMBINED_VIDEO_USE_CASE_{i}.mp4"
            out = cv2.VideoWriter(
                path_save,
                cv2.VideoWriter_fourcc(*"mp4v"),
                24,
                (frame_width * 4, frame_height),
            )

            for frame_idx in range(min_length):
                combined_frame = cv2.hconcat(
                    [stream[frame_idx] for stream in video_streams]
                )
                out.write(combined_frame)

            out.release()
            print(
                f"La vidéo combinée du UseCase {i} est enregistrée sous : {path_save}"
            )


if __name__ == "__main__":
    rec = Reconstruction()
    rec.video_reconstruction(2)
