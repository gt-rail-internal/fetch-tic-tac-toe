import cv2

# processes an image into the gameboard vector
def process_image(img):
    # image is initially BGR (confirmed)
    # convert to HSV (H[0,179], S[0,255], V[0,255])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
    
    # isolate black dots
    low_H = 0
    low_S = 0
    low_V = 0
    high_H = 255
    high_S = 255
    high_V = 50
    print(cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)[225,460])
    black_dots = cv2.inRange(hsv, (low_H, low_S, low_V), (high_H, high_S, high_V))
    print("black dots", black_dots[225,460])
    cv2.imwrite("rgb.png", black_dots)

    # threshold yellow dots
    threshold_dots = cv2.threshold(img, cv2.rgb())

    cv2.imwrite("base.png", img)

    return []


if __name__ == "__main__":
    img = cv2.imread("vision/test.png")
    process_image(img)