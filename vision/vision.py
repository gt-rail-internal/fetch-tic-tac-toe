from pickletools import unicodestringnl
import cv2
import numpy as np


# processes an image into the gameboard vector
def process_image(img):
    # image is initially BGR (confirmed)
    # convert to HSV (H[0,179], S[0,255], V[0,255])
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # cut away edges
    top_cut = int(img.shape[0]*1/3)
    bottom_cut = int(img.shape[0]*3/4)
    left_cut = int(img.shape[1]*1/4)
    right_cut = int(img.shape[1]*3/4)
    default_pixel = [180,255,255]
    img[:top_cut,:,:] = default_pixel
    img[bottom_cut:,:,:] = default_pixel
    img[:,:left_cut,:] = default_pixel
    img[:,right_cut:,:] = default_pixel

    #cv2.imwrite("cut.png", img)
    
    # ISOLATE BLACK DOTS, EMPTY LOCATIONS
    # threshold the HSV
    
    low_H = 90
    low_S = 60
    low_V = 20
    high_H = 120
    high_S = 255
    high_V = 70
    

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
    
    
    # if not two black blobs, error a bit
    if len(locations) != 2:
        print("Not exactly two black blobs", len(locations))
        cv2.imshow("img_black.png", img_black)
        cv2.waitKey()
        return []

    # sort locations top down left right
    locations = locations[np.lexsort((locations[:,1], locations[:,0]))]

    #print("side coordinate markers", locations)

    # generate the snap locations
    x_left = locations[0][0]
    y_top = locations[0][1]
    x_diff = abs(locations[1][0] - locations[0][0])
    y_diff = abs(locations[0][1] - locations[1][1])
    x_iter = x_diff / 6
    y_iter = y_diff / 6

    snap_locations = np.zeros((9,2))
    idx = 0
    for r in range(0,3):
        for c in range(0,3):
            snap_locations[idx][0] = x_left + (2*c+1) * x_iter
            snap_locations[idx][1] = y_top + (2*r+1) * y_iter
            idx += 1

    #print("snap locations", snap_locations)

    empty_locations = locations
    #print("empty", empty_locations)

    # ISOLATE BLUE X, X LOCATIONS
    # threshold the HSV
    low_H = 70
    low_S = 150
    low_V = 60
    high_H = 140
    high_S = 255
    high_V = 255
    
    img_blue = cv2.inRange(img, (low_H, low_S, low_V), (high_H, high_S, high_V))
    
    # decay, expand
    kernel = np.ones((5,5), np.uint8)
    img_blue = cv2.erode(img_blue, kernel, iterations=2)
    img_blue = cv2.dilate(img_blue, kernel, iterations=4)
    kernel = np.ones((10,10), np.uint8)
    img_blue = cv2.dilate(img_blue, kernel, iterations=1)

    # find blobs
    img_blue = cv2.bitwise_not(img_blue)
    img_blue = cv2.blur(img_blue, ksize=(5,5))

    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = False
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(img_blue)
    locations = cv2.KeyPoint_convert(keypoints)

    x_locations = locations
    #print("x", x_locations)


    # ISOLATE RED O, O LOCATIONS
    # threshold the HSV
    low_H = 140
    low_S = 50
    low_V = 50
    high_H = 255
    high_S = 255
    high_V = 255
    
    img_red = cv2.inRange(img, (low_H, low_S, low_V), (high_H, high_S, high_V))
    
    # decay, expand
    kernel = np.ones((5,5), np.uint8)
    img_red = cv2.erode(img_red, kernel, iterations=2)
    img_red = cv2.dilate(img_red, kernel, iterations=4)
    kernel = np.ones((10,10), np.uint8)
    img_red = cv2.dilate(img_red, kernel, iterations=4)

    # find blobs
    img_red = cv2.bitwise_not(img_red)
    img_red = cv2.blur(img_red, ksize=(5,5))

    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = False
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(img_red)
    locations = cv2.KeyPoint_convert(keypoints)

    o_locations = locations
    #print("o", o_locations)

    markers = determine_coordinates(snap_locations, x_locations, o_locations)
    #print("markers", markers)
    

    cv2.imwrite("base.png", img)

    return markers


def determine_coordinates(snaps, x, o):
    # add marker type to each coord
    snaps = np.hstack((snaps, np.ones((snaps.shape[0],1)) * 0))
    if len(x) != 0:
        x = np.hstack((x, np.ones((x.shape[0],1)) * 1))
    if len(o) != 0:
        o = np.hstack((o, np.ones((o.shape[0],1)) * 2))

    # snap each x and o to closest point, count each at each location
    markers = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    if len(x) != 0:
        for ix in range(x.shape[0]):
            min_dist = float("inf")
            snap = -1
            for i in range(snaps.shape[0]):
                dist = np.sqrt((x[ix][0] - snaps[i][0])**2 + (x[ix][1] - snaps[i][1])**2)
                if dist < min_dist:
                    snap = i
                    min_dist = dist
            markers[snap] = 1
            
    if len(o) != 0:
        for io in range(o.shape[0]):
            min_dist = float("inf")
            snap = -1
            for i in range(snaps.shape[0]):
                dist = np.sqrt((o[io][0] - snaps[i][0])**2 + (o[io][1] - snaps[i][1])**2)
                if dist < min_dist:
                    snap = i
                    min_dist = dist
            markers[snap] = 2

    return markers

if __name__ == "__main__":
    #img = cv2.imread("test2.png")
    #process_image(img)
    pass