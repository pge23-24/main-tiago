import cv2
import numpy as np


def read_images(image_paths):
    """Read images from the given paths."""
    return [cv2.imread(path) for path in image_paths]


def find_keypoints_and_descriptors(images):
    """Find keypoints and descriptors in all images."""
    orb = cv2.ORB_create()
    keypoints, descriptors = [], []
    for image in images:
        kp, desc = orb.detectAndCompute(image, None)
        keypoints.append(kp)
        descriptors.append(desc)
    return keypoints, descriptors


def match_keypoints(descriptors):
    """Match keypoints between all image pairs."""
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = []
    for i in range(len(descriptors) - 1):
        matches.append(bf.match(descriptors[i], descriptors[i + 1]))
    return matches


def stitch_images(images, keypoints, matches):
    """Stitch images together using the matches."""
    # Start with the first image
    result = images[0]
    result_kp = keypoints[0]

    for i in range(1, len(images)):
        # Retrieve keypoints, descriptors, and matches
        kp1, kp2 = result_kp, keypoints[i]
        good_matches = matches[i - 1]

        # Extract location of good matches
        points1 = np.zeros((len(good_matches), 2), dtype=np.float32)
        points2 = np.zeros((len(good_matches), 2), dtype=np.float32)

        for j, match in enumerate(good_matches):
            points1[j, :] = kp1[match.queryIdx].pt
            points2[j, :] = kp2[match.trainIdx].pt

        # Find homography
        H, _ = cv2.findHomography(points1, points2, cv2.RANSAC)

        # Warp image
        height, width = images[0].shape[:2]
        result = cv2.warpPerspective(result, H, (width, height))

        # Stitch the image with the result
        result = cv2.addWeighted(result, 0.5, images[i], 0.5, 0)

        # Update keypoints for the next iteration
        result_kp = kp2

    return result


def transform_to_birds_eye_view(image, src_points, dest_size=(300, 300)):
    """Transform the stitched image to a bird's eye view.

    Args:
    - image: The input stitched image.
    - src_points: Four source points in the image defining the area to transform.
    - dest_size: Size of the output bird's eye view image.

    Returns:
    - Bird's eye view of the selected portion of the input image.
    """
    # Destination points are the corners of a rectangle with the given size
    dst_points = np.array([
        [0, 0],
        [dest_size[0] - 1, 0],
        [dest_size[0] - 1, dest_size[1] - 1],
        [0, dest_size[1] - 1]
    ], dtype=np.float32)

    # Compute the perspective transform matrix and apply it
    M = cv2.getPerspectiveTransform(src_points, dst_points)
    birds_eye_view = cv2.warpPerspective(image, M, dest_size)

    return birds_eye_view


def visualize_src_points(image, src_points):
    for point in src_points:
        int_point = (int(point[0]), int(point[1]))  # Convert to integer
        cv2.circle(image, int_point, 5, (0, 0, 255), -1)  # Red circles
    cv2.imshow("Source Points on Panorama", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def visualize_matches(img1, kp1, img2, kp2, matches):
    """Visualize matches between two images."""
    match_img = cv2.drawMatches(
        img1, kp1, img2, kp2, matches[:50], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
    )
    cv2.imshow('Matches', match_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# Example usage
image_paths = ['assets/bird_view/front.png', 'assets/bird_view/left.png',
               'assets/bird_view/rear.png', 'assets/bird_view/right.png']
images = read_images(image_paths)
keypoints, descriptors = find_keypoints_and_descriptors(images)
matches = match_keypoints(descriptors)
panorama = stitch_images(images, keypoints, matches)


# Define source points (these should be adjusted based on your specific image)
src_points = np.float32([[100, 100], [400, 100], [400, 400], [100, 400]])

# Visualize src_points on the panorama
visualize_src_points(panorama.copy(), src_points)

# Generate the bird's eye view
birds_eye_view = transform_to_birds_eye_view(panorama, src_points)

cv2.imshow("Bird's Eye View", birds_eye_view)
cv2.waitKey(0)
cv2.destroyAllWindows()
