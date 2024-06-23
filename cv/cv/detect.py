import cv2
import numpy as np

root_path = '/home/ammarvora/decaf_ws/src/decaf/cv/'

def cvt_coords(x, y, z, w, h):
    '''
        x is scaled wrt to the width of the image, with the origin at the center.
        z is scaled wrt to the height of the image
    '''

    x_scaled = (x - (w / 2)) / (w / 2)
    y_scaled = (y - (h / 2)) / (h / 2)
    z_scaled = z / h

    return x_scaled, y_scaled, z_scaled


def detect(img):
    h, w, _ = img.shape

    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1, minDist=50, param1=50, param2=10, minRadius=0, maxRadius=h)

    # If circles are detected
    x, y, r = 0, 0, 0
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        idx = np.argmax(circles[:, 2]) # largest circle based on radius

        x, y, r = circles[idx]

        # Draw the circle in the output image
        cv2.circle(img, (x, y), r, (0, 255, 0), 2)
        cv2.circle(img, (x, y), 1, (0, 0, 255), 2)

    return img, cvt_coords(x, y, r, w, h)


if __name__ == '__main__':

    path = 'test.png'

    in_img = cv2.imread(root_path + path)

    out, (x_scaled, y_scaled, z_scaled) = detect(in_img)

    cv2.imshow('out', out)
    cv2.waitKey(0)

    print(x_scaled, y_scaled, z_scaled)