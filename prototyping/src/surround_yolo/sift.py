import cv2 as cv
import numpy as np


class PanoramaFusion:
    def __init__(self):
        self.sift = cv.SIFT_create()
        self.bf = cv.BFMatcher(cv.NORM_L1, crossCheck=True)

    def extraire_points_correspondants(self, img1, img2):
        nb_max_matchs = 80

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

    def fusionner_images(self, img_gauche, img_centre, img_droite):
        points_correspondants = self.extraire_points_correspondants(img_gauche, img_centre)
        points_correspondants2 = self.extraire_points_correspondants(img_centre, img_droite)

        pts1 = np.float32([pt1 for pt1, _ in points_correspondants])
        pts2 = np.float32([pt2 for _, pt2 in points_correspondants])

        # Calculer la matrice de transformation
        M1, _ = cv.findHomography(pts2, pts1, cv.RANSAC, 5.0)

        result1 = cv.warpPerspective(img_gauche, M1, (img_centre.shape[1] + img_gauche.shape[1], img_gauche.shape[0]))

        pts1 = np.float32([pt1 for pt1, _ in points_correspondants2])
        pts2 = np.float32([pt2 for _, pt2 in points_correspondants2])

        # Calculer la matrice de transformation
        M2, _ = cv.findHomography(pts2, pts1, cv.RANSAC, 5.0)
        print(M2.dtype)
        M2 = np.array([[5.95386881e-02, 2.62926448e-01, 7.28072247e+02],
              [-3.08233310e-01, 1.16498737e+00, -6.54359169e+00],
              [-5.76426116e-04, 2.28260555e-04, 1.00000000e+00]])
        print(M2)

        result2 = cv.warpPerspective(img_droite, M2, (img_centre.shape[1] + img_droite.shape[1], img_droite.shape[0]))

        # Copier l'image 1 sur l'image résultante
        result1[:img_centre.shape[0], img_centre.shape[1]:] = img_centre

        # Copier l'image 1 sur l'image résultante
        result2[0:img_centre.shape[0], 0:img_centre.shape[1]] = img_centre

        result = np.concatenate((result1, result2[:, img_centre.shape[1]:]),
                                axis=1)
        return result
