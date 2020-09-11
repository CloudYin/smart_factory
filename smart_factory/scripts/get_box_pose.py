#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np


def get_box_pose(calibrated_file_path):
    box_pickX_list = []
    original_image = cv2.imread(calibrated_file_path)
    image_copy = original_image.copy()
    image_hsv = cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)
    lower_b = np.array([0, 0, 254])
    upper_b = np.array([0, 0, 255])
    mask = cv2.inRange(image_hsv, lower_b, upper_b)
    cv2.bitwise_and(original_image, original_image, mask=mask)

    binary, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # Select inner contours
    try:
        hierarchy = hierarchy[0]
        for component in zip(contours, hierarchy):
            currentContour = component[0]
            currentHierarchy = component[1]
            if currentHierarchy[3] == -1:
                # these are the outermost child components
                img_contour = cv2.drawContours(image_copy, currentContour, -1, (0, 0, 0))
                if cv2.moments(currentContour)['m00'] > 16000:
                    m00 = cv2.moments(currentContour)['m00']
                    m01 = cv2.moments(currentContour)['m01']
                    m10 = cv2.moments(currentContour)['m10']
                    centerX = round(m10/m00)
                    centerY = round(m01/m00)
                    rect = cv2.minAreaRect(currentContour)
                    points = cv2.boxPoints(rect)
                    points = np.int0(points)
                    cv2.drawContours(img_contour, [points], -1, (0, 255, 0), 2)
                    pickX = (-255.5 - centerX * 0.1807 ) / 1000
                    box_pickX_list.append(pickX)
                    cv2.circle(img_contour, (int(centerX), int(centerY)), 5, (0, 0, 255))
        # x, y = img_contour.shape[0:2]
        # reshaped_img = cv2.resize(img_contour, (int(y / 3), int(x / 3)))
        # cv2.imshow("img_contour", reshaped_img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        box_pickX_list = sorted(box_pickX_list)
    except:
        pass
    return box_pickX_list


if __name__ == '__main__':
    calibrated_file_path = '/home/pilz/Pictures/smart_factory/cap_calibrated.png'
    box_x = get_box_pose(calibrated_file_path)
    print(box_x)
