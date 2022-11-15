#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray
import av
import cv2
import numpy as np
import threading
import traceback
import time

import apriltag

class StandaloneVideoStream(object):
    def __init__(self):
        self.cond = threading.Condition()
        self.queue = []
        self.closed = False

    def read(self, size):
        self.cond.acquire()
        try:
            if len(self.queue) == 0 and not self.closed:
                self.cond.wait(2.0)
            data = bytes()
            while 0 < len(self.queue) and len(data) + len(self.queue[0]) < size:
                data = data + self.queue[0]
                del self.queue[0]
        finally:
            self.cond.release()
        return data

    def seek(self, offset, whence):
        return -1

    def close(self):
        self.cond.acquire()
        self.queue = []
        self.closed = True
        self.cond.notifyAll()
        self.cond.release()

    def add_frame(self, buf):
        self.cond.acquire()
        self.queue.append(buf)
        self.cond.notifyAll()
        self.cond.release()


stream = StandaloneVideoStream()


def callback(msg):
    #rospy.loginfo('frame: %d bytes' % len(msg.data))
    stream.add_frame(msg.data)

def find_Mask(img):
    lr0 = np.array([0, 70, 0])
    ur0 = np.array([5, 255, 255])
    lr1 = np.array([175, 70, 0])
    ur1 = np.array([180, 255, 255])
    rm0 = cv2.inRange(img, lr0, ur0)
    rm1 = cv2.inRange(img, lr1, ur1)
    rm = cv2.bitwise_or(rm0, rm1)
    return rm

def find_green_mask(img):
    lr = np.array([36,0,0])
    ur = np.array([86,255,255])
    rm = cv2.inRange(img, lr, ur)
    return rm

def find_blue_mask(img):
    lr = np.array([110,50,50])
    ur = np.array([130,255,255])
    rm = cv2.inRange(img, lr, ur)
    return rm

mask_arr = [find_Mask, find_green_mask, find_blue_mask]

def main():

    fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
    out = cv2.VideoWriter('test.avi', fourcc, 20.0, (1920, 720))
    
    rospy.init_node('h264_listener')
    rospy.Subscriber("/tello/image_raw/h264", CompressedImage, callback)
    point_pub = rospy.Publisher("/target_point", Float64MultiArray, queue_size = 10)

    tag_pub = rospy.Publisher("/tag_value", Float64MultiArray, queue_size = 10)

    container = av.open(stream)
    rospy.loginfo('main: opened')
    
    start_detect = True
    start_detect_tag = False
    start_detect2 = False
    frame_skip = 300
    rec_count = 0

    for frame in container.decode(video=0):
        if 0 < frame_skip:
            frame_skip -= 1
            continue
        start_time = time.time()
        image = cv2.cvtColor(np.array(
            frame.to_image()), cv2.COLOR_RGB2BGR)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        red_mask = mask_arr[rec_count](hsv_img)

        _, c_c, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        max_c = max(c_c, key = cv2.contourArea)
        rect = cv2.minAreaRect(max_c)
        r_w, r_h = rect[1]
        rec_x = rect[0][0]
        rec_y = rect[0][1]
        
        show_image = cv2.cvtColor(np.zeros(image.shape[:2], dtype=np.uint8), cv2.COLOR_GRAY2BGR)
        #cv2.drawContours(show_image, [max_c], -1, (0, 0, 255), -1)
        cv2.rectangle(show_image, (int(rec_x-0.5*r_w), int(rec_y-0.5*r_h)), (int(rec_x+0.5*r_w), int(rec_y+0.5*r_h)), (0,0,255), -1)
        print("minAreaRect x: ", rec_x)
        print("minAreaRect y: ", rec_y)
        cv2.circle(show_image, (int(rec_x), int(rec_y)), 10, (0,50,175), -1)
        cv2.putText(show_image, str((r_w * r_h) / (960*720.)), (10,40), 5, 2, (255,255,0))
        
        if start_detect:
            if r_w * r_h >= 960*720*0.35:
                point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 1, rec_count]))
                start_detect = False
                start_detect_tag = True
                rec_count += 1
            else:
                point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 0, rec_count]))  

        if start_detect_tag:
            print('detect_tag')
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            detector = apriltag.Detector()
            tag_result = detector.detect(gray_image)
            if len(tag_result) != 0:
                tag_pub.publish(Float64MultiArray(data = [tag_result[0].center, tag_result[0].corners]))
                start_detect_tag = False
                start_detect2 = True


        if start_detect2:
            if r_w * r_h >= 960*720*0.35:
                point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 1, rec_count]))
                start_detect2 = False  
            else:
                point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 0, rec_count]))  

        
        
        out.write(np.concatenate((image, show_image), axis = 1))    
        cv2.imshow('result', cv2.resize(np.concatenate((image, show_image), axis = 1), (480, 180) ))
        cv2.waitKey(1)
        if frame.time_base < 1.0/60:
            time_base = 1.0/60
        else:
            time_base = frame.time_base
            frame_skip = int((time.time() - start_time)/time_base)

if __name__ == '__main__':
    try:
        main()
    except BaseException:
        traceback.print_exc()
    finally:
        stream.close()
        cv2.destroyAllWindows()
