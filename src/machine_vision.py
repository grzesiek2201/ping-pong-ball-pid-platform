#!/usr/bin/env python

import numpy as np
import cv2
import time


class MachineVision:

    def __init__(self, show=False):
        print("MachineVision constructor started")
        self.video_capture = cv2.VideoCapture(0)
        print(self.video_capture)
        print("MachineVision constructor in progess")
        self.video_capture.set(3, 640)
        self.video_capture.set(4, 480)
        self.cx = 0
        self.cy = 0
        self.error = 0
        self.show = show
    
        print("MachineVision constructor ended")

    def __del__(self):
        self.video_capture.release()
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def process_frame(self):
        """ Process a frame and return the ball position and error """
        ret, frame = self.video_capture.read()
        platform_frame = crop_image(frame)
        self.cx, self.cy, self.error, rgb_image = detect_ball_in_a_frame(
                                    platform_frame, 
                                    (self.cx, self.cy, self.error), 
                                    show=self.show)

        return self.cx, self.cy, self.error, rgb_image


def read_rgb_image(image_name, show):
    """ Read image and return it """
    rgb_image = cv2.imread(image_name)
    if show:
        cv2.imshow("RGB Image", rgb_image)
    return rgb_image


def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    """ Filter out colors based on lower and upper color ranges, HSV,
        return the mask """
    # convert the image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

    # define a mask using the lower and upper bounds of the yellow color
    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)

    return mask


def getContours(binary_image):
    """ Return contours of an image """
    binary_image = cv2.GaussianBlur(binary_image, (3, 3), 0)
    contours, hierarchy = cv2.findContours(binary_image.copy(),
                                           cv2.RETR_LIST,
                                           cv2.CHAIN_APPROX_SIMPLE)
    #contours = cv2.HoughCircles(binary_image, cv2.HOUGH_GRADIENT, 3, 100)
    return contours


def get_ball(binary_image, rgb_image, contours, prev_coords, show=False):
    """ Return coordinates of the ball """
    black_image = np.zeros([binary_image.shape[0], binary_image.shape[1], 3], 'uint8')
    cx, cy = 0, 0

    if contours is not None:
        for c in contours:
            area = cv2.contourArea(c)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if 3000 < area < 6000:
                cx, cy = get_contour_center(c)
                cv2.drawContours(rgb_image, [c], -1, (150, 250, 150), 1)
                cv2.drawContours(black_image, [c], -1, (150, 250, 150), 1)
                cv2.putText(rgb_image, f"x={cx}; y={cy}", (cx+20, cy-20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), 1)
                cv2.circle(rgb_image, (cx, cy), int(radius), (0, 0, 255), 1)
                cv2.circle(black_image, (cx, cy), int(radius), (0, 0, 255), 1)
                cv2.circle(black_image, (cx, cy), 5, (150, 150, 255), -1)
                
                if show == True:
                    cv2.imshow("RGB Image Contours", rgb_image)
                    cv2.imshow("Binary image mask", binary_image)
                
                return cx, cy, rgb_image
            elif show == True:
                cv2.imshow("RGB Image Contours", rgb_image)
                cv2.imshow("Binary image mask", binary_image)
    else:
        if show == True:
            cv2.imshow("RGB Image Contours", rgb_image)
            cv2.imshow("Binary image mask", binary_image) 
        return prev_coords[0], prev_coords[1], rgb_image

    return False, False, rgb_image


def get_contour_center(contour):
    """ Return the center of the supplied contour """
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if (M['m00'] != 0):
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
    return cx, cy


def detect_ball_in_a_frame(image_frame, prev_coords, show=False):
    """ Supply image and return ball coordinates and error """
    yellowLower = (0, 0 ,0) # day setup (33, 0 ,0), night setup (0, 0, 0) 
    yellowUpper = (179, 153, 255)  # day setup (179, 84, 255), night setup (179, 131, 255)
    
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, yellowLower, yellowUpper)
    
    contours = getContours(binary_image_mask)
    
    cx, cy, rgb_image = get_ball(binary_image_mask, rgb_image, contours, prev_coords, show)
    
    center = (image_frame.shape[1] / 2, image_frame.shape[0] / 2)
    
    if cx is False:
        cx, cy = center[0], center[1]
        
    # error
    error = (-(center[0] - cx), (center[1] - cy))
    return cx, cy, error, rgb_image


def crop_image(img, size=(50, 350, 50, 350)):
    """ Crop the image to desired size """
#    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#    mask = cv2.inRange(hsv_img, (167, 161, 71), (179, 255, 177))
#    edges = cv2.Canny(mask, 50, 150, apertureSize=3)
#    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=10, maxLineGap=130)

#    if lines is not None:
#        x1 = min(lines[:, 0, 0])
#        y1 = min(lines[:, 0, 1])
#        x2 = max(lines[:, 0, 2])
#        y2 = max(lines[:, 0, 3])

        #cv2.rectangle(edges, (x1, y1), (x2, y2), (255, 255, 255), 2)
        #cv2.imshow('edges', edges)

#        if abs(y1-y2) > 100 and abs(x1-x2) > 100:
#            print("returning img[y1:y2, x1:x2]")
#            return img[y1:y2, x1:x2]

#    print("returning img")
#    return img

    return img[size[0]: size[1], size[2]: size[3]]


def main():
    video_capture = cv2.VideoCapture(0)
    video_capture.set(3, 640)
    video_capture.set(4, 480)
    coords = (0, 0, 0)

    while True:
        ret, frame = video_capture.read()
        # crop image to only get the platform in the frame
        platform_frame = crop_image(frame)
        # search for the ball contours and remember the previous ones if not found
        coords = detect_ball_in_a_frame(platform_frame, coords, show=True)
        print(coords[0], coords[1])
        print(f"error = {coords[2]}")
        #time.sleep(0.033)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.waitKey(0)
    cv2.destroyAllWindows()
