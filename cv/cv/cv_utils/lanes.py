import cv2
import numpy as np

root_path = '/home/ammarvora/decaf_ws/src/decaf/cv/cv_utils'

def find_barrels(img):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the HSV range for orange
    lower_orange = np.array([10, 100, 100])
    upper_orange = np.array([25, 255, 255])

    # Create a mask with the specified range
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    return mask


def find_lanes(img):
    h, w, c = img.shape

    # Crop out the bottom 10% and top 25% of the frame
    cutoff = (0.25, 0.9)
    crop = np.zeros((h, w, c), dtype=np.uint8)
    crop[(int)(cutoff[0] * h) : (int)(cutoff[1] * h), :] = np.array([1, 1, 1])
    img_cropped = img * crop

    lower = 240
    upper = 255
    
    gray = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2GRAY) 
    lane_mask = cv2.inRange(gray, lower, upper)

    # Remove the white lines created by the construction barrels

    barrel_mask = find_barrels(img_cropped) // 255

    # Create a barrel_cols_mask where 0: barrel pixel exists in that column, 1: otherwise
    pixel_intensity = 1 # can be anywhere in range (1, h) to signify how many barrel pixels in that column
    barrel_cols = np.where(np.sum(barrel_mask, axis=0) > pixel_intensity, 0, 1) # shape: (w, )
    barrel_cols = np.expand_dims(barrel_cols, 0) # shape: (1, w)
    barrel_cols_mask = np.repeat(barrel_cols, h, axis=0).astype(np.uint8) # shape: (w, h)

    assert barrel_cols_mask.shape == barrel_mask.shape

    # Final step to remove barrels from lane mask
    clean_lane_mask = lane_mask * barrel_cols_mask

    return clean_lane_mask


if __name__ == '__main__':

    path = 'test.png'

    in_img = cv2.imread(root_path + path)

    out = find_lanes(in_img)

    cv2.imshow('out', out)
    cv2.waitKey(0)
