import cv2 as cv

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
    img1 = cv.imread("test_images_damien_stich/USE_CASE_1/CAMERA_4/reel.png")
    img2 = cv.imread("test_images_damien_stich/USE_CASE_1/CAMERA_1/reel.png")
    img3 = cv.imread("test_images_damien_stich/USE_CASE_1/CAMERA_3/reel.png")
    img4 = cv.imread("test_images_damien_stich/USE_CASE_1/CAMERA_2/reel.png")

    list4images = [img1, img2, img3, img4]
    s = Sticher()
    s.stitching(list4images)

