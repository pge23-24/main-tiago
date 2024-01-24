import os
from prototyping.src.surround_yolo import sift
from prototyping.src.surround_yolo import image4to1_V2
from prototyping.src.surround_yolo import zoom
import cv2 as cv
import numpy as np

if __name__ == "__main__":
    folderImagesMires = "test_images_damien_stich/selected_img_mires"  # A remplir sur ordi conti
    test_img_undistort = "test_images_damien_stich/selected_img_mires/image_1124.png"  # A remplir sur ordi conti
    path_use_cases = "test_images_damien_stich/USE_CASE_1"  # A remplir sur ordi conti
    folder_save_undistort_and_stich = "test_images_damien_stich/resultats"  # A remplir sur ordi conti
    nb_img_in_use_case = 1  # A remplir

    c = image4to1_V2.CvFunction()
    f = image4to1_V2.Fuze()
    panorama_fusion = sift.PanoramaFusion()

    k, d, dim = c.calibrate(folderImagesMires)
    im = cv.imread(test_img_undistort)
    undistort_test_pour_calcul_zoom = c.undistort(im, k, d, dim)
    cv.imwrite(folder_save_undistort_and_stich + "/" + "calibre_zoom.png", undistort_test_pour_calcul_zoom)

    z = zoom.Zoom(folder_save_undistort_and_stich + "/" + "calibre_zoom.png")

    directories = [d for d in os.listdir(path_use_cases) if os.path.isdir(os.path.join(path_use_cases, d))]

    for i in range(nb_img_in_use_case):
        strImg = (f"/reel"
                  f".png")

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

        list4images = [undistort_zoom_img4, undistort_zoom_img1, undistort_zoom_img3, undistort_zoom_img2]
        concat = f.concatener_images_horizontalement(list4images)
        cv.imwrite(folder_save_undistort_and_stich + f"/concat{i}.png", concat)

    image_fusionnee = panorama_fusion.fusionner_images(img4, img1)
    image_fusionnee = panorama_fusion.fusionner_images(img1, img3)
    image_fusionnee = panorama_fusion.fusionner_images(img3, img2)
    image_fusionnee = panorama_fusion.fusionner_images(img2, img4)

    cv.imwrite("test_images_damien_stich/resultats/test_sift.png", image_fusionnee)
    cv.imshow('Image fusionnée', image_fusionnee)
    cv.waitKey(0)
    cv.destroyAllWindows()