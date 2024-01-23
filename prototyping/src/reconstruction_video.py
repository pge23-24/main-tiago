import os
import cv2
from natsort import natsorted
import logging

PATH_USE_CASES = "./assets/USE_CASES"
PATH_VIDEO_USE_CASES = "./assets/VIDEO_USE_CASES"

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class VideoReconstruction:
    def __init__(
        self,
        frame_width=1226,
        frame_height=968,
    ):
        self.frame_width = frame_width
        self.frame_height = frame_height

    def video_detection(
        self,
        path_folder_image=PATH_USE_CASES,
        path_save=PATH_VIDEO_USE_CASES,
    ):
        path_sorted = natsorted(os.listdir(path_folder_image))

        out = cv2.VideoWriter(
            path_save,
            cv2.VideoWriter_fourcc(*"mp4v"),
            24,
            (self.frame_width, self.frame_height),
        )
        for file in path_sorted:
            path_file = path_folder_image + "/" + file
            img = cv2.imread(path_file)

            if img is not None and img.shape[:2] == (
                self.frame_height,
                self.frame_width,
            ):
                out.write(img)
            else:
                logger.info(f"Error: Invalid image size or format - {path_file}")
        out.release()

    def use_case_reconstruction(self, nb_use_case: int):
        for i in range(1, nb_use_case + 1):
            prefix = PATH_USE_CASES + "/USE_CASE_" + str(i)
            save_prefix = PATH_VIDEO_USE_CASES + "/VIDEO_USE_CASE_" + str(i)
            self.video_detection(
                prefix + "/CAMERA_1", save_prefix + "/CAMERA_1/vid.mp4"
            )
            self.video_detection(
                prefix + "/CAMERA_2", save_prefix + "/CAMERA_2/vid.mp4"
            )
            self.video_detection(
                prefix + "/CAMERA_3", save_prefix + "/CAMERA_3/vid.mp4"
            )
            self.video_detection(
                prefix + "/CAMERA_4", save_prefix + "/CAMERA_4/vid.mp4"
            )
        print(f"La vidéo du UseCase {i} est enregistrée sous : {save_prefix}")


if __name__ == "__main__":
    use_case = VideoReconstruction()
    nb_use_case = 1
    use_case.use_case_reconstruction(nb_use_case)
