import cv2
import numpy as np
import cv2 as cv
import os


class CvFunction:
    def calibrate(self, folderImages):
        # termination criteria
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((6 * 7, 3), np.float32)
        objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.
        for fname in os.listdir(folderImages):
            img = cv.imread(fname)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv.findChessboardCorners(gray, (7, 6), None)
            # If found, add object points, image points (after refining them)
            if ret is True:
                objpoints.append(objp)
                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)
                # Draw and display the corners
                cv.drawChessboardCorners(img, (7, 6), corners2, ret)
                cv.waitKey(500)
        cv.destroyAllWindows()
        K = None
        D = None
        ret, mtx, dist, rvecs, tvecs = cv.fisheye.calibrate(
            objpoints, imgpoints, np.shape(fname), K, D
        )

    def undistort(self, image, ret, mtx, dist):
        h, w = image.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        # undistort
        dst = cv.undistort(image, mtx, dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y : y + h, x : x + w]
        cv.imwrite("calibresult.png", dst)


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
    pth1 = "USE_CASES/USE_CASE_1/CAMERA_1/image_NO_0.png"
    pth2 = "USE_CASES/USE_CASE_1/CAMERA_2/image_NO_0.png"
    pth3 = "USE_CASES/USE_CASE_1/CAMERA_3/image_NO_0.png"
    pth4 = "USE_CASES/USE_CASE_1/CAMERA_4/image_NO_0.png"

    folderImagesMires = "vis2-tiago/folder_mire_images"

    cx = None
    cy = None
    fx = None
    fy = None

    c = CvFunction()

    c.calibrate(folderImagesMires)

    f = Fuze()

    list_corrected_image = f.correction_4distorsion()

    resultat_concatene = f.concatener_images_horizontalement(list_corrected_image)
    cv2.imwrite("USE_CASES/USE_CASE_1/stich.png", resultat_concatene)
