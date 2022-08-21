import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
import sys

VIDEO_FILE_NAME = "../../Video.mp4"
OUTPUT_FILE_NAME = "../../Motion_Tracking.mp4"
def locateRobot(frame, tracker):

    mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), (34, 19 , 91), (107, 153, 255))
    kernel = np.ones((6,6),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    x = y = -1
    for i in range (0, len(contours), 1):
        (x,y), radius = cv2.minEnclosingCircle(contours[i])

    if x == -1 and y == -1:
        ok = False
        bbox = (0,0,0,0)
    else:
        bbox = (x-radius, y-radius, 2*radius, 2*radius)
        ok = tracker.init(frame, bbox)

    return ok, bbox, tracker

def updateTracker(prev_bbox, bbox, frame, frame_num, tracker):
    if abs(prev_bbox[0] - bbox[0]) > 20 or abs(prev_bbox[1] - bbox[1]) > 20:
        tracker = cv2.TrackerKCF_create()
        ok, bbox, tracker = locateRobot(frame, tracker)

    elif frame_num % 60 == 0:
        tracker = cv2.TrackerKCF_create()
        tracker_type = "KCF"
        ok, bbox ,tracker = locateRobot(frame, tracker) 

    else:
        ok, bbox = tracker.update(frame)

    return ok, bbox, tracker

def readVideo(name):
    video = cv2.VideoCapture(name)

    if not video.isOpened():
        print("Could not open video")
        sys.exit()
    
    return video

def readFrame(video, frame_num):
    ok, frame = video.read()
    if not ok and frame_num == 0:
        print('Cannot read video file')
        sys.exit()
    return ok, frame

def drawTracker(frame, bbox):
    p1 = (int(bbox[0]), int(bbox[1]))
    p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
    cv2.rectangle(frame, p1, p2, (0, 255, 0), 2, 1)
    num_times_lost = 0
    return frame, num_times_lost

def robotTracking():
    frame_num = 0
    num_times_lost = 0
    video = readVideo(VIDEO_FILE_NAME)

    #Uncomment to save tracking video
    w = int(video.get(3))
    h = int(video.get(4))
    out = cv2.VideoWriter(OUTPUT_FILE_NAME, cv2.VideoWriter_fourcc(*'mp4v'), 30, (w, h))
    
    tracker = cv2.TrackerKCF_create()
    tracker_type = "KCF"

    #Read first frame
    ok, frame = readFrame(video, frame_num)

    ok, bbox, tracker = locateRobot(frame, tracker)
    prev_bbox = bbox

    while True:

        ok, frame = readFrame(video, frame_num)
        
        if not ok:
            break

        timer = cv2.getTickCount()
        
        ok, bbox, tracker = updateTracker(prev_bbox, bbox, frame, frame_num, tracker)

        fps = cv2.getTickFrequency()/(cv2.getTickCount() - timer)
        prev_bbox = bbox

        if ok:
            frame, num_times_lost = drawTracker(frame, bbox)
        
        else:
            #Change Tracker
            if (num_times_lost > 2):
                tracker = cv2.TrackerCSRT_create()
                tracker_type = "CSRT"

            else:
                #Relocate robot with the same tracker
                tracker = cv2.TrackerKCF_create()
            ok, bbox, tracker = locateRobot(frame, tracker)
            if ok:
                frame, num_times_lost = drawTracker(frame, bbox)
            num_times_lost += 1
        
        # cv2.imshow("Tracking", frame)
        k = cv2.waitKey(10) & 0xff 
        if k == 27: break
        frame_num += 1

        out.write(frame)

    out.release()
    video.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    robotTracking()