import time
import cv2 as cv
import os
import prototyping.src.surround_yolo.undistortPolynomial as undistortPolynomial
import prototyping.src.surround_yolo.homography as homography
import numpy as np


class image4to1_V4():

    def undistort_and_stich(self, path_all_use_cases="test_images_damien_stich/USES_CASES",
                            folder_save="test_images_damien_stich/resultats"):
        '''
            Undistort image and stich for already stored images
        '''

        up1 = undistortPolynomial.UndistortPolynomial(1)
        up2 = undistortPolynomial.UndistortPolynomial(2)
        up3 = undistortPolynomial.UndistortPolynomial(3)
        up4 = undistortPolynomial.UndistortPolynomial(4)

        M1 = np.array([[1.33385533e+00, -2.09343873e-01, 7.14895389e+02],
                       [3.13458029e-01, 1.00740937e+00, -6.92486966e+01],
                       [5.57695060e-04, - 2.23578602e-04, 1.00000000e+00]])

        M2 = np.array([[1.61827928e+00, -8.66375625e-02, 5.36091267e+02],
                       [-7.88738898e-03, 9.38739975e-01, 1.01538669e+00],
                       [3.51352043e-04, -1.58623768e-04, 1.00000000e+00]])

        h = homography.Homography(M1, M2)
        path_each_use_cases = [d for d in os.listdir(path_all_use_cases) if
                               os.path.isdir(os.path.join(path_all_use_cases, d))]

        for each_use_case in path_each_use_cases:
            path = os.path.join(path_all_use_cases, each_use_case)
            directories = [d for d in os.listdir(path) if os.path.isdir(os.path.join(path, d))]

            path_cam_1 = os.path.join(path, directories[0])
            list_path_cam_1 = []

            path_cam_2 = os.path.join(path, directories[1])
            list_path_cam_2 = []

            path_cam_3 = os.path.join(path, directories[3])
            list_path_cam_3 = []

            path_cam_4 = os.path.join(path, directories[2])
            list_path_cam_4 = []

            for images in os.listdir(path_cam_1):
                if images.endswith(".png"):
                    list_path_cam_1.append(images)

            for images in os.listdir(path_cam_2):
                if images.endswith(".png"):
                    list_path_cam_2.append(images)

            for images in os.listdir(path_cam_3):
                if images.endswith(".png"):
                    list_path_cam_3.append(images)

            for images in os.listdir(path_cam_4):
                if images.endswith(".png"):
                    list_path_cam_4.append(images)

            nb_img_in_use_case = np.min(
                [len(list_path_cam_1), len(list_path_cam_2), len(list_path_cam_3), len(list_path_cam_4)])
            for i in range(nb_img_in_use_case):
                strImg = f"/image_{i}.png"

                img1 = cv.imread(os.path.join(path, directories[0] + strImg))
                start_time = time.time()
                undistort_img1 = up1.undistort_image(img1)
                end_time = time.time()
                print(end_time - start_time)

                img2 = cv.imread(os.path.join(path, directories[1] + strImg))
                undistort_img2 = up2.undistort_image(img2)

                img3 = cv.imread(os.path.join(path, directories[3] + strImg))
                undistort_img3 = up3.undistort_image(img3)

                img4 = cv.imread(os.path.join(path, directories[2] + strImg))
                undistort_img4 = up4.undistort_image(img4)

                # Stiching
                # Ordre des images pour les cam : 4 1 3 2
                start_time = time.time()
                image_fusionnee = h.apply_homography(undistort_img4, undistort_img1, undistort_img3)
                end_time = time.time()
                print(end_time - start_time)

                if not os.path.exists(os.path.join(folder_save, each_use_case)):
                    os.makedirs(os.path.join(folder_save, each_use_case))
                cv.imwrite(os.path.join(folder_save, each_use_case + strImg), image_fusionnee)


if __name__ == "__main__":
    i = image4to1_V4()
    i.undistort_and_stich()
