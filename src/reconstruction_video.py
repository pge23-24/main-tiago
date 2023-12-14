import os
import cv2
from natsort import natsorted


def video_detection(path_folder_image, path_save, frame_width=1226, frame_height=968):

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


if __name__ == '__main__':
    nb_use_case = 1
    for i in range(1, nb_use_case + 1):
        prefix = "./USE_CASES/USE_CASE_" + str(i)
        save_prefix = "./VIDEO_USE_CASES/VIDEO_USE_CASE_" + str(i)
        video_detection(prefix + "/CAMERA_1", save_prefix + "/CAMERA_1/vid.mp4")
        video_detection(prefix + "/CAMERA_2", save_prefix + "/CAMERA_2/vid.mp4")
        video_detection(prefix + "/CAMERA_3", save_prefix + "/CAMERA_3/vid.mp4")
        video_detection(prefix + "/CAMERA_4", save_prefix + "/CAMERA_4/vid.mp4")
    print(f"La vidéo du UseCase {i} est enregistrée sous : {save_prefix}")