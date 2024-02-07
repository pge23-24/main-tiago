import cv2 as cv
import prototyping.src.surround_yolo.undistortPolynomial as undistortPolynomial

class Sticher:
    def __init__(self):
        self.stitch = cv.Stitcher().create(cv.Stitcher_SCANS)

    def stitching(self, imgs_list):
        (status, stitched) = self.stitch.stitch(imgs_list)
        if status == 0:
            print("success")
            cv.imshow("", stitched)
            cv.imwrite("test_images_damien_stich/resultats/stitchingOpenCV.png", stitched)
            cv.waitKey(0)
            cv.destroyAllWindows()
        else:
            print("[INFO] image stitching failed ({})".format(status))

if __name__ == "__main__":
    img1 = cv.imread("test_images_damien_stich/USE_CASE_1/CAMERA_4/image_0.png")
    img2 = cv.imread("test_images_damien_stich/USE_CASE_1/CAMERA_1/image_0.png")
    img3 = cv.imread("test_images_damien_stich/USE_CASE_1/CAMERA_3/image_0.png")
    img4 = cv.imread("test_images_damien_stich/USE_CASE_1/CAMERA_2/image_0.png")

    up1 = undistortPolynomial.UndistortPolynomial(1)
    up2 = undistortPolynomial.UndistortPolynomial(2)
    up3 = undistortPolynomial.UndistortPolynomial(3)
    up4 = undistortPolynomial.UndistortPolynomial(4)

    undistort_img1 = up1.undistort_image(img1)

    undistort_img2 = up2.undistort_image(img2)

    undistort_img3 = up3.undistort_image(img3)

    undistort_img4 = up4.undistort_image(img4)

    list4images = [undistort_img1, undistort_img2, undistort_img3, undistort_img4]
    s = Sticher()
    s.stitching(list4images)

