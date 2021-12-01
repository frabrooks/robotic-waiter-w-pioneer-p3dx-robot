#! /usr/bin/env python

import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2
from chat import Chat

def decode(im, rosnode) :
    # Find barcodes and QR codes
    decodedObjects = pyzbar.decode(im)

    # Print results
    for obj in decodedObjects:
        rosnode.say(obj.data)

    return decodedObjects


# Display barcode and QR code location
def display(im, decodedObjects):

    # Loop over all decoded objects
    for decodedObject in decodedObjects:
        points = decodedObject.polygon

    # If the points do not form a quad, find convex hull
    if len(points) > 4 :
        hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
        hull = list(map(tuple, np.squeeze(hull)))
    else :
        hull = points;

    # Number of points in the convex hull
    n = len(hull)

    # Draw the convext hull
    for j in range(0,n):
        cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)

        # Display results
        cv2.imshow("web", im);

def cam_img(cam):
    ret_val, img = cam.read()
    return img

# Main
if __name__ == '__main__':

    ## Read image
    # im = cv2.imread('zbar-test.png')

    #decodedObjects = decode(im)
    #display(im, decodedObjects)

    rosnode = Chat()

    cam = cv2.VideoCapture(0)

    while True:
        im = cam_img(cam)
        dec = decode(im, rosnode)

        if dec:
            display(im, dec)
        else:
            cv2.imshow('web', im)

        if cv2.waitKey(1) == 27:
            break
    cv2.destroyAllWindows()
