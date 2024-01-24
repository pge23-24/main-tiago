import numpy as np
import cv2 as cv
import os


class Zoom:
    def __init__(self, path_img):
        self.img = cv.imread(path_img)
        self.gray = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)
        self.old_height, self.old_width = self.img.shape[:2]
        self.ratio = self.old_width / self.old_height
        self.mid_width = int(self.old_width / 2)-1
        min_h = np.inf
        max_h = -np.inf
        for p in range(self.old_height):
            if self.gray[p, self.mid_width] != 0:
                if p < min_h:
                    min_h = p
                elif p > max_h:
                    max_h = p
        self.new_height = max_h - min_h
        self.new_width = self.new_height * self.ratio
        self.start_y = min_h
        self.start_x = self.old_width / 2 - self.new_width / 2

    def zoom_image(self, img):
        return img[self.start_y:self.start_y + self.new_height, int(self.start_x):int(self.start_x + self.new_width), :]


if __name__ == "__main__":
    path_undistort = "undistort_and_stich/calibre_zoom.png"
    z = Zoom(path_undistort)

    zommed = z.zoom_image(cv.imread(path_undistort))
    cv.imshow("Zoom", zommed)
    cv.waitKey(0)
    cv.destroyAllWindows()
