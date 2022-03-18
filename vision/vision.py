import cv2
import numpy as np


# processes an image into the gameboard vector
def process_image(img):
    # image is initially BGR (confirmed)
    # convert to HSV (H[0,179], S[0,255], V[0,255])
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # ISOLATE BLACK DOTS, EMPTY LOCATIONS
    # threshold the HSV
    low_H = 0
    low_S = 0
    low_V = 0
    high_H = 255
    high_S = 255
    high_V = 50
    
    img_black = cv2.inRange(img, (low_H, low_S, low_V), (high_H, high_S, high_V))
    
    # decay, expand
    kernel = np.ones((5,5), np.uint8)
    img_black = cv2.erode(img_black, kernel, iterations=2)
    img_black = cv2.dilate(img_black, kernel, iterations=4)

    # find blobs
    img_black = cv2.bitwise_not(img_black)
    detector = cv2.SimpleBlobDetector_create()
    keypoints = detector.detect(img_black)
    locations = cv2.KeyPoint_convert(keypoints)

    empty_locations = locations
    print("empty", empty_locations)

    # ISOLATE BLUE X, X LOCATIONS
    # threshold the HSV
    low_H = 70
    low_S = 0
    low_V = 0
    high_H = 255
    high_S = 255
    high_V = 255
    
    img_blue = cv2.inRange(img, (low_H, low_S, low_V), (high_H, high_S, high_V))
    
    # decay, expand
    kernel = np.ones((5,5), np.uint8)
    img_blue = cv2.erode(img_blue, kernel, iterations=2)
    img_blue = cv2.dilate(img_blue, kernel, iterations=4)

    # find blobs
    img_blue = cv2.bitwise_not(img_blue)
    img_blue = cv2.blur(img_blue, ksize=(5,5))
    detector = cv2.SimpleBlobDetector_create()
    keypoints = detector.detect(img_blue)
    locations = cv2.KeyPoint_convert(keypoints)

    x_locations = locations
    print("x", x_locations)

    cv2.imwrite("rgb.png", img_blue)

    cv2.imwrite("base.png", img)

    return []


if __name__ == "__main__":
    img = cv2.imread("vision/test.png")
    process_image(img)