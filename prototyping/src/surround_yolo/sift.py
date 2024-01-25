import cv2 as cv
import numpy as np

class PanoramaFusion:
    def __init__(self):
        self.sift = cv.SIFT_create()
        self.bf = cv.BFMatcher(cv.NORM_L1, crossCheck=True)

    def extraire_points_correspondants(self, img1, img2):
        nb_max_matchs = 50

        k_1, des_1 = self.sift.detectAndCompute(img1, None)
        k_2, des_2 = self.sift.detectAndCompute(img2, None)

        matches = self.bf.match(des_1, des_2)
        matches_filtered = sorted(matches, key=lambda x: x.distance)[:nb_max_matchs]

        img3 = cv.drawMatches(img1, k_1, img2, k_2, matches_filtered, img2, flags=2)

        cv.imshow("Output", img3)
        cv.waitKey(0)
        cv.destroyAllWindows()

        points_correspondants = [(k_1[match.queryIdx].pt, k_2[match.trainIdx].pt) for match in matches_filtered]

        return points_correspondants

    def fusionner_images(self, img1, img2):

        points_correspondants = self.extraire_points_correspondants(img1, img2)

        pts1 = np.float32([pt1 for pt1, _ in points_correspondants])
        pts2 = np.float32([pt2 for _, pt2 in points_correspondants])

        # Calculer la matrice de transformation
        M, _ = cv.findHomography(pts2, pts1, cv.RANSAC, 5.0)

        # Appliquer la transformation à l'image 2 pour aligner les points d'intérêt
        result = cv.warpPerspective(img2, M, (img1.shape[1] + img2.shape[1], img2.shape[0]))
        # Calculer la matrice de transformation
        M, _ = cv.findHomography(pts2, pts1, cv.RANSAC, 5.0)

        # Appliquer la transformation à l'image 2 pour aligner les points d'intérêt
        result = cv.warpPerspective(img2, M, (img1.shape[1] + img2.shape[1], img2.shape[0]))

        # Copier l'image 1 sur l'image résultante
        result[0:img1.shape[0], 0:img1.shape[1]] = img1


        return result
