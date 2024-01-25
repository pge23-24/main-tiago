import numpy as np
import cv2 as cv
import os


class CvFunction:
    def calibrate(self, folderImages):
        CHECKERBOARD = (6, 9)
        subpix_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        calibration_flags = (
            cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC
            + cv.fisheye.CALIB_CHECK_COND
            + cv.fisheye.CALIB_FIX_SKEW
        )
        objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp[0, :, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(
            -1, 2
        )
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
                assert (
                    _img_shape == img.shape[:2]
                ), "All images must share the same size."
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            # Chess board corners
            ret, corners = cv.findChessboardCorners(
                gray,
                CHECKERBOARD,
                cv.CALIB_CB_ADAPTIVE_THRESH
                + cv.CALIB_CB_FAST_CHECK
                + cv.CALIB_CB_NORMALIZE_IMAGE,
            )
            # Image points (after refinin them)
            if ret == True:
                objpoints.append(objp)
                cv.cornerSubPix(gray, corners, (3, 3), (-1, -1), subpix_criteria)
                imgpoints.append(corners)
                # Draw and display the corners
                cv.drawChessboardCorners(img, (9, 6), corners, ret)
                cv.imshow("img", img)
                cv.waitKey(0)
                cv.destroyAllWindows()
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
            (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 1e-6),
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
        assert (
            dim1[0] / dim1[1] == DIM[0] / DIM[1]
        ), "Image to undistort needs to have same aspect ratio as the ones used in calibration"
        if not dim2:
            dim2 = dim1
        if not dim3:
            dim3 = dim1
        scaled_K = (
            K * dim1[0] / DIM[0]
        )  # The values of K is to scale with image dimension.
        scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
        # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
        new_K = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(
            scaled_K, D, dim2, np.eye(3), balance=balance
        )
        map1, map2 = cv.fisheye.initUndistortRectifyMap(
            scaled_K, D, np.eye(3), new_K, dim3, cv.CV_16SC2
        )
        undistorted_img = cv.remap(
            img,
            map1,
            map2,
            interpolation=cv.INTER_LINEAR,
            borderMode=cv.BORDER_CONSTANT,
        )

        return undistorted_img


class Fuze:
    def concatener_images_horizontalement(self, listImgCorrected):
        assert (
            listImgCorrected[0].shape[0]
            == listImgCorrected[1].shape[0]
            == listImgCorrected[2].shape[0]
            == listImgCorrected[3].shape[0]
        ), "Les hauteurs des images doivent être les mêmes."

        result = np.concatenate(
            (
                listImgCorrected[0],
                listImgCorrected[2],
                listImgCorrected[1],
                listImgCorrected[3],
            ),
            axis=1,
        )
        return result


# Exemple d'utilisation
if __name__ == "__main__":
    folderImagesMires = "selected_img_mires"  # A remplir sur ordi conti
    test_img_undistort = "selected_img_mires/image_1124.png"  # A remplir sur ordi conti
    path_use_cases = "USE_CASE_1"  # A remplir sur ordi conti
    folder_save_undistort_and_stich = "undistort_and_stich"

    c = CvFunction()
    # On obtient la matrice de la camera et la matrice de la dimension
    k, d, dim = c.calibrate(folderImagesMires)

    f = Fuze()
    nbImg = 2  # A remplir
    directories = [
        d
        for d in os.listdir(path_use_cases)
        if os.path.isdir(os.path.join(path_use_cases, d))
    ]
    print(directories)
    for i in range(nbImg):
        strImg = f"/image_{i}.png"
        img1 = cv.imread(path_use_cases + "/" + directories[0] + strImg)
        undistort_img1 = c.undistort(img1, k, d, dim)
        img2 = cv.imread(path_use_cases + "/" + directories[1] + strImg)
        undistort_img2 = c.undistort(img2, k, d, dim)
        img3 = cv.imread(path_use_cases + "/" + directories[3] + strImg)
        undistort_img3 = c.undistort(img3, k, d, dim)
        img4 = cv.imread(path_use_cases + "/" + directories[2] + strImg)
        undistort_img4 = c.undistort(img4, k, d, dim)
        list4images = [undistort_img1, undistort_img2, undistort_img3, undistort_img4]
        concat = f.concatener_images_horizontalement(list4images)
        cv.imwrite(folder_save_undistort_and_stich + f"/concat{i}.png", concat)
