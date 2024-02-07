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

    def fusionner_images(self, img_gauche, img_centre, img_droite):
        pts1 = np.array([[935, 708],
                         [931, 106],
                         [1189, 571],
                         [1102, 558],
                         [1101, 376],
                         [1087, 330],
                         [1152, 247],
                         [1104, 243],
                         [1086, 162],
                         [1020, 180],
                         [1052, 250],
                         [936, 734],
                         [925, 743],
                         [919, 105],
                         [958, 57],
                         [1109, 270],
                         [1108, 415],
                         [1161, 405],
                         [1162, 434],
                         [1182, 513],
                         [1129, 435],
                         [1127, 526],
                         [1016, 630],
                         [932, 405],
                         [1103, 452],
                         [1121, 327],
                         [1063, 275]])
        pts2 = np.array([[271, 684],
                         [264, 103],
                         [603, 567],
                         [515, 551],
                         [516, 361],
                         [499, 312],
                         [555, 214],
                         [498, 211],
                         [462, 121],
                         [396, 154],
                         [445, 226],
                         [273, 709],
                         [258, 715],
                         [250, 111],
                         [304, 41],
                         [511, 242],
                         [525, 403],
                         [593, 393],
                         [596, 423],
                         [596, 505],
                         [558, 423],
                         [554, 519],
                         [390, 618],
                         [265, 391],
                         [516, 439],
                         [534, 309],
                         [460, 252]])

        # Calculer la matrice de transformation
        M1, _ = cv.findHomography(pts2, pts1, cv.RANSAC, 5.0)

        result1 = cv.warpPerspective(img_gauche, M1, (img_centre.shape[1] + img_gauche.shape[1], img_gauche.shape[0]))

        pts1 = np.array([[1000, 710],
                         [758, 590],
                         [710, 566],
                         [759, 276],
                         [711, 339],
                         [698, 348],
                         [698, 313],
                         [728, 230],
                         [802, 157],
                         [891, 44],
                         [754, 442],
                         [680, 441],
                         [603, 567],
                         [624, 492],
                         [620, 452],
                         [680, 359],
                         [680, 552],
                         [739, 384],
                         [871, 21],
                         [679, 468],
                         [664, 507],
                         [690, 382]])
        pts2 = np.array([[323, 754],
                         [148, 593],
                         [118, 570],
                         [152, 297],
                         [120, 355],
                         [112, 363],
                         [113, 331],
                         [126, 258],
                         [177, 186],
                         [236, 66],
                         [148, 453],
                         [95, 452],
                         [19, 566],
                         [39, 498],
                         [36, 461],
                         [97, 374],
                         [95, 556],
                         [142, 398],
                         [214, 52],
                         [96, 477],
                         [86, 515],
                         [108, 394]])

        # Calculer la matrice de transformation
        M2, _ = cv.findHomography(pts2, pts1, cv.RANSAC, 5.0)

        result2 = cv.warpPerspective(img_droite, M2, (img_centre.shape[1] + img_droite.shape[1], img_droite.shape[0]))

        # Copier l'image 1 sur l'image résultante
        result1[:img_centre.shape[0], img_centre.shape[1]:] = img_centre

        # Copier l'image 1 sur l'image résultante
        result2 = result2[:, img_centre.shape[1]:]

        result = np.concatenate((result1, result2),
                                axis=1)
        return result, M1, M2


if __name__ == '__main__':
    p = PanoramaFusion()
    img1_path = 'test_images_damien_stich/resultats/up4.png'
    img2_path = 'test_images_damien_stich/resultats/up1.png'
    img3_path = 'test_images_damien_stich/resultats/up3.png'
    img_gauche = cv.imread(img1_path)
    img_centre = cv.imread(img2_path)
    img_droite = cv.imread(img3_path)
    res, m1, m2 = p.fusionner_images(img_gauche, img_centre, img_droite)
    print(m1,m2)
    cv.imshow("resultats", res)
    cv.waitKey(0)
    cv.destroyAllWindows()
