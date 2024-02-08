import cv2
import numpy as np

def click_and_match(img1_path, img2_path):
    # Charger les images
    img1 = cv2.imread(img1_path)
    img2 = cv2.imread(img2_path)

    # Vérifier si les images sont chargées correctement
    if img1 is None or img2 is None:
        print("Erreur lors du chargement des images.")
        return

    # Créer des listes vides pour stocker les points source et destination
    pts_src = []
    pts_dst = []

    # Définir la fonction de rappel pour les clics de souris
    def click_callback(event, x, y, flags, param):
        nonlocal pts_src, pts_dst

        if event == cv2.EVENT_LBUTTONDOWN:
            # Ajouter le point cliqué de l'image 1 à pts_src
            pts_src.append([x, y])

            # Afficher un marqueur sur le point cliqué dans l'image 1
            cv2.circle(img1, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow('Image 1', img1)

            # Attendre la correspondance du point dans l'image 2
            print(f"Cliquez sur le point correspondant dans l'image 2 pour le point ({x}, {y}) de l'image 1.")

        elif event == cv2.EVENT_RBUTTONDOWN:
            # Ajouter le point cliqué de l'image 2 à pts_dst
            pts_dst.append([x, y])

            # Afficher un marqueur sur le point cliqué dans l'image 2
            cv2.circle(img2, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow('Image 2', img2)

            # Afficher la correspondance dans la console
            print(f"Point correspondant dans l'image 2: ({x}, {y})")

    # Afficher les images
    cv2.imshow('Image 1', img1)
    cv2.imshow('Image 2', img2)

    # Définir la fonction de rappel pour les clics de souris sur les deux images
    cv2.setMouseCallback('Image 1', click_callback)
    cv2.setMouseCallback('Image 2', click_callback)

    # Attendre que l'utilisateur appuie sur la touche 'q' pour quitter
    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    # Fermer toutes les fenêtres
    cv2.destroyAllWindows()

    # Convertir les listes de points en tableaux NumPy
    pts_src = np.array(pts_src)
    pts_dst = np.array(pts_dst)

    return pts_src, pts_dst

img1_path = 'test_images_damien_stich/resultats/up4.png'
img2_path = 'test_images_damien_stich/resultats/up1.png'
img3_path = 'test_images_damien_stich/resultats/up3.png'

pts_src, pts_dst = click_and_match(img1_path, img2_path)

# Afficher les points sources et de destination
print("Points sources:\n", pts_src)
print("Points de destination:\n", pts_dst)
