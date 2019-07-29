import cv2
#import imutils
import numpy as np
import pdb

def cd_color_segmentation(img, show_image=False):

    # convert from rgb to hsv color space (it might be BGR)
    new_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    new_img = new_img[175:, :]
    img = img[225:, :]

    # define lower and upper bound of image values
    red_low_range  = np.array( [ 0, 44, 147] )  
    red_high_range = np.array( [ 8, 254, 225] ) 
    yellow_low_range = np.array( [20, 80, 150] )
    yellow_high_range = np.array( [40, 255, 255])
    blue_low_range = np.array([105,95,130])
    blue_high_range = np.array([115,255,255])
    # create mask for image with overlapping values
    rMask = cv2.inRange(new_img, red_low_range, red_high_range)
    yMask = cv2.inRange(new_img, yellow_low_range, yellow_high_range)
    bMask = cv2.inRange(new_img, blue_low_range, blue_high_range)

    # filter the image with bitwise and
    rfiltered = cv2.bitwise_and(new_img, new_img, mask=rMask)
    yFiltered = cv2.bitwise_and(new_img, new_img, mask=yMask)
    bFiltered = cv2.bitwise_and(new_img, new_img, mask=bMask)
    

    # find the contours in the image
    _, rContours, hierarchy = cv2.findContours(rMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    _, yContours, hierarchy = cv2.findContours(yMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    _, bContours, hierarchy = cv2.findContours(bMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    rx1, ry1, rx2, ry2 = 0, 0, 0, 0
    yx1, yy1, yx2, yy2 = 0, 0, 0, 0
    bx1, by1, bx2, by2 = 0, 0, 0, 0
    for i, color in enumerate([rContours, yContours, bContours]):
        if len(color) != 0:
	    # find contour with max area, which is most likely the cone
            # Solution note: max uses an anonymous function in this case, we can also use a loop...
            contours_max = max(color, key = cv2.contourArea)

	    # Find bounding box coordinates
            x1, y1, x2, y2 = cv2.boundingRect(contours_max)

	    # Draw the bounding rectangle
            if i == 1:
                yx1, yy1, yx2, yy2 = cv2.boundingRect(contours_max)
                cv2.rectangle(img, (yx1, yy1), (yx1 + yx2, yy1 + yy2), (0, 255, 255), 2)
            elif i == 0:
                rx1, ry1, rx2, ry2 = cv2.boundingRect(contours_max)
                cv2.rectangle(img, (rx1, ry1), (rx1 + rx2, ry1 + ry2), (0, 0, 255), 2)
            else: 
                bx1, by1, bx2, by2 = cv2.boundingRect(contours_max)
                cv2.rectangle(img, (bx1, by1), (bx1 + bx2, by1 + by2), (255, 0, 0), 2)

    if show_image:
        cv2.imshow("Color segmentation", img)
        key = cv2.waitKey()
        if key == 'q':
            cv2.destroyAllWindows()

    # Return bounding box
    return[((rx1, ry1), (rx1 + rx2, ry1 + ry2)), ((yx1, yy1), (yx1 + yx2, yy1 + yy2)), ((bx1, by1), (bx1 + bx2, by1 + by2))], img
