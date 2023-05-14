#!/usr/bin/env python3

# Use global shutter camera instead of USB camera
# requires that /boot/config.txt includes 'dtoverlay=imx296'
# and does not set camera_auto_detect=1.

import cv2
import numpy as np
from wpimath.geometry import Transform3d
import math
import time
from cscore import CameraServer
from picamera2 import Picamera2
import ntcore # network tables
import logging
import robotpy_apriltag

# This function is called once to initialize the apriltag detector and the pose estimator
def get_apriltag_detector_and_estimator(frame_size):
    detector = robotpy_apriltag.AprilTagDetector()
    # FRC 2023 uses tag16h5 (game manual 5.9.2)
    assert detector.addFamily("tag16h5")
    estimator = robotpy_apriltag.AprilTagPoseEstimator(
    # The raspberry pi global shutter camera uses the Sony IMX296 sensor, whose pixels
    # are 3.45 micrometers square.  They are in a 1440 wide by 1080 high array.
    # I am currently using the pi wide angle lens, with a focal length of 6 millimeters.
    # Assuming there is no space between the pixels, this makes fx and fy
    # 6.00e-3/3.45e-6 = 1739.13 pixels
    robotpy_apriltag.AprilTagPoseEstimator.Config(
            0.1524, # width of tag in meters (6 inches / 39.37 ) (was 0.2)
            1739.13,    # fx: "camera horizontal focal length, in pixels"
            1739.13,    # fy: "camera vertical focal length, in pixels"
            frame_size[0] / 2.0,    # cx: "camera horizontal focal center, in pixels"
            frame_size[1] / 2.0     # cy: "camera vertical focal center, in pixels"
        )
    )
    return detector, estimator
    
# This function is called for every detected tag. It uses the `estimator` to 
# return information about the tag, including its id, pose, and centerpoint.
# (The corners are also available.)
def process_apriltag(estimator, tag):
    tag_id = tag.getId()
    center = tag.getCenter()
    # hamming = tag.getHamming()
    # decision_margin = tag.getDecisionMargin()
    est = estimator.estimateOrthogonalIteration(tag, 50)
    return {'id':tag_id, 'pose':est.pose1, 'center':center}

# This function is called once for every frame captured by the Webcam. For testing, it can simply
# be passed a frame capture loaded from a file.
def detect_and_process_apriltag(frame, detector, estimator):
    assert frame is not None
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect apriltag
    tag_info = detector.detect(gray)
    DETECTION_MARGIN_THRESHOLD = 100
    filter_tags = [tag for tag in tag_info if tag.getDecisionMargin() > DETECTION_MARGIN_THRESHOLD]
    results = [ process_apriltag(estimator, tag) for tag in filter_tags ]
    return results

def setup_network_table_publishers(teamNumber, clientName, apriltag_ids_to_publish):
    global table # needs to exist for entire length of program
    # logging.basicConfig(level=logging.DEBUG)
    instance = ntcore.NetworkTableInstance.getDefault()
    instance.setServerTeam(teamNumber)
    instance.startClient4(clientName)
    table = instance.getTable("/Apriltag")
    publishers = {}
    for id in apriltag_ids_to_publish:
      publisher = table.getDoubleArrayTopic("id_pose_center_" + str(id)).publish()
      publisher.setDefault([])
      # wasSeenLastTime=True is to force a network tables put() at startup
      publishers[id] = { "publisher":publisher, "wasSeenLastTime":True }
    return publishers
    
def publish(publishers, results): # assume 'results' are output of detect_and_process_apriltag
    ids_not_seen = set(list(publishers.keys()))
    # print(".publishing " + str([result['id'] for result in results]))
    for result in results:
        try:
            id = result['id'] # which apriltag
            try:
                ids_not_seen.remove(id) # will throw error if this is not one to publish
            except:
                print("id " + str(id) + " not in " + str(ids_not_seen) + ".  It is not published.")
                continue
            # print("..publishing " + str(id))
            a = [ float(id) ] 
            po = result['pose']
            tr = po.translation()
            a.extend( [tr.x, tr.y, tr.z] ) # in meters
            # rotation() gives counterclockwise rotation angle around x, y, and z axes (in radians?)
            ro = po.rotation()
            a.extend( [ro.x, ro.y, ro.z] )
            ce = result['center']
            a.extend( [ce.x, ce.y] )
            publisher = publishers[id]
            publisher["publisher"].set(a)
            if not publisher["wasSeenLastTime"]:
                print("* Started seeing " + str(id))
                publisher["wasSeenLastTime"] = True
        except:
            continue
    for id in ids_not_seen:
        publisher = publishers[id]
        if publisher["wasSeenLastTime"]:
            print("* Lost sight of " + str(id))
            try:
                publisher["publisher"].set([]) # empty message means this id was not seen in this frame
                publisher["wasSeenLastTime"] = False
            except:
                continue

#######
def main():
    outputImage = True
    apriltag_ids_to_publish = [1, 2, 3, 4, 5, 6, 7, 8]
    CameraServer.enableLogging() # ???

    # Pi Global Shutter camera has 1456*1088 pixels, but
    # the Pi's Broadcom chip can quickly reduce its size.
    # 'frame_size' refers to the reduced size (the 'main'
    # stream).  There is also a low resolution stream ('lores').
    # frame_size = (1456, 1088) # camera's native frame size, in pixels
    # Too big a frame_size slows down processing.
    frame_size = (640, 480)
    frame_rate = 60 # frames/second (this is not used, we just take stills)
    camera = Picamera2()
    camera_config = camera.create_still_configuration( {'size': frame_size} )
    camera.configure(camera_config)
    camera.start()
    time.sleep(1)

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    if (outputImage):
        outputStream = CameraServer.putVideo("Raspberry Pi", frame_size[0], frame_size[1])

    # Making apriltag detector is expensive
    detector, estimator = get_apriltag_detector_and_estimator(frame_size)

    global publishers
    publishers = setup_network_table_publishers(4173, "rPi", apriltag_ids_to_publish)

    prev_grab_time = time.monotonic()
    while True:
        img = camera.capture_array()
        # print("shape=" + str(img.shape))
        if img is None:
            # Send the output the error.
            if (outputImage):
                outputStream.notifyError("Problem")
            # skip the rest of the current iteration
        else:
            global res # for debugging
            results = detect_and_process_apriltag(img, detector, estimator)
        # normalize the within-frame coordinates to the range [-1,1]
        for result in results:
            result['center'].x = (result['center'].x - frame_size[0]/2) / (frame_size[0]/2)
            result['center'].y = (result['center'].y - frame_size[1]/2) / (frame_size[1]/2)
        publish(publishers, results)

        # Give the output stream a new image to display
        if (outputImage):
            outputStream.putFrame(img)
        grab_time = time.monotonic()
        # print("** delta(grab_time)=" + str( (grab_time-prev_grab_time)/1e6 ) + " s., " + str(len(results)) + " tags found")
        prev_grab_time = grab_time
        # If sleep is too long, you won't see the decorated image in img
        # time.sleep(.020)

if __name__ == "__main__":

    # To see messages from networktables, you must setup logging
    import logging

    # logging.basicConfig(level=logging.DEBUG)
    main()
