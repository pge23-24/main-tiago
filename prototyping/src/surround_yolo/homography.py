import numpy as np
import cv2 as cv


class Homography:
    def __init__(self, M1=None, M2=None):
        if M1 is None or M2 is None:
            self.M1 = np.array(np.eye(3))
            self.M2 = np.array(np.eye(3))
        else:
            self.M1 = M1
            self.M2 = M2

    def apply_homography(self, img_gauche, img_centre, img_droite):

        result1 = cv.warpPerspective(img_gauche, self.M1,
                                     (img_centre.shape[1] + img_gauche.shape[1], img_gauche.shape[0]))

        result2 = cv.warpPerspective(img_droite, self.M2,
                                     (img_centre.shape[1] + img_droite.shape[1], img_droite.shape[0]))

        # Copier l'image 1 sur l'image
        result1[:img_centre.shape[0], img_centre.shape[1]:] = img_centre

        # Crop l'emplacement de l'image 1 sur l'image
        result2 = result2[:, img_centre.shape[1]:]

        result = np.concatenate((result1, result2), axis=1)

        result = result[:, 650:result.shape[1] - 550]

        return result
