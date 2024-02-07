import prototyping.src.surround_yolo.zoom as zoom
import numpy as np
import cv2 as cv
import os


class CvFunction:
    def calibrate(self, folderImages):
        CHECKERBOARD = (6, 9)
        subpix_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        calibration_flags = cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv.fisheye.CALIB_CHECK_COND + cv.fisheye.CALIB_FIX_SKEW
        objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
        _img_shape = None
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.
        for fname in os.listdir(folderImages):
            img = cv.imread(folderImages + "/" + fname)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            # Find the chess board corners
            if _img_shape == None:
                _img_shape = img.shape[:2]
            else:
                assert _img_shape == img.shape[:2], "All images must share the same size."
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            # Chess board corners
            ret, corners = cv.findChessboardCorners(gray, CHECKERBOARD,
                                                    cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK + cv.CALIB_CB_NORMALIZE_IMAGE)
            # Image points (after refinin them)
            if ret == True:
                objpoints.append(objp)
                cv.cornerSubPix(gray, corners, (3, 3), (-1, -1), subpix_criteria)
                imgpoints.append(corners)
        N_OK = len(objpoints)
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        rms, _, _, _, _ = cv.fisheye.calibrate(
            objpoints,
            imgpoints,
            gray.shape[::-1],
            K,
            D,
            rvecs,
            tvecs,
            calibration_flags,
            (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        )
        print("Found " + str(N_OK) + " valid images for calibration")
        print("DIM=" + str(_img_shape[::-1]))
        print("K=np.array(" + str(K.tolist()) + ")")
        print("D=np.array(" + str(D.tolist()) + ")")
        DIM = _img_shape[::-1]

        return K, D, DIM

    def undistort(self, img, K, D, DIM):
        balance = 1
        dim2 = None
        dim3 = None
        dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
        assert dim1[0] / dim1[1] == DIM[0] / DIM[
            1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
        if not dim2:
            dim2 = dim1
        if not dim3:
            dim3 = dim1
        scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
        scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
        # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
        new_K = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
        map1, map2 = cv.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv.CV_16SC2)
        undistorted_img = cv.remap(img, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)

        return undistorted_img


class Fuze:
    def concatener_images_horizontalement(self, listImgCorrected):
        assert listImgCorrected[0].shape[0] == listImgCorrected[1].shape[0] == listImgCorrected[2].shape[0] == \
               listImgCorrected[3].shape[0], "Les hauteurs des images doivent être les mêmes."

        result = np.concatenate((listImgCorrected[0], listImgCorrected[1], listImgCorrected[2], listImgCorrected[3]),
                                axis=1)
        return result


if __name__ == "__main__":

    folderImagesMires = "test_images_damien_stich/selected_img_mires"  # A remplir sur ordi conti
    path_use_cases = "test_images_damien_stich/USES_CASES/USE_CASE_1"  # A remplir sur ordi conti
    folder_save_undistort_and_stich = "test_images_damien_stich/resultats"  # A remplir sur ordi conti
    nb_img_in_use_case = 1  # A remplir

    c = CvFunction()
    f = Fuze()

    # k, d, dim = c.calibrate(folderImagesMires) # A faire pour trouver k, d et DIM
    dim = (1226, 968)
    k = np.array([[347.83050477884086, 0.0, 608.3626306199714], [0.0, 347.0491750226914, 477.76387950465687], [0.0, 0.0, 1.0]])
    d = np.array([[0.03907587018956459], [-0.01029576958403915], [0.002599059284828633], [-0.0007811096479636197]])

    # A faire pour obtenir une photo non distordue pour la calibration du zoom
    # test_img_undistort = "test_images_damien_stich/selected_img_mires/image_1124.png"
    # im = cv.imread(test_img_undistort)
    # undistort_test_pour_calcul_zoom = c.undistort(im, k, d, dim)
    # cv.imwrite(folder_save_undistort_and_stich + "/" + "calibre_zoom.png", undistort_test_pour_calcul_zoom)

    z = zoom.Zoom(folder_save_undistort_and_stich + "/" + "calibre_zoom.png")

    directories = [d for d in os.listdir(path_use_cases) if os.path.isdir(os.path.join(path_use_cases, d))]

    for i in range(nb_img_in_use_case):
        strImg = f"/image_{i}.png"

        img1 = cv.imread(path_use_cases + "/" + directories[0] + strImg)
        undistort_img1 = c.undistort(img1, k, d, dim)
        undistort_zoom_img1 = z.zoom_image(undistort_img1)

        img2 = cv.imread(path_use_cases + "/" + directories[1] + strImg)
        undistort_img2 = c.undistort(img2, k, d, dim)
        undistort_zoom_img2 = z.zoom_image(undistort_img2)

        img3 = cv.imread(path_use_cases + "/" + directories[2] + strImg)
        undistort_img3 = c.undistort(img3, k, d, dim)
        undistort_zoom_img3 = z.zoom_image(undistort_img3)

        img4 = cv.imread(path_use_cases + "/" + directories[3] + strImg)
        undistort_img4 = c.undistort(img4, k, d, dim)
        undistort_zoom_img4 = z.zoom_image(undistort_img4)

        # Ordre des images pour les cam : 4 1 3 2
        list4images = [undistort_zoom_img4, undistort_zoom_img1, undistort_zoom_img3, undistort_zoom_img2]

        concat = f.concatener_images_horizontalement(list4images)
        cv.imwrite(folder_save_undistort_and_stich + f"/concat{i}.png", concat)
