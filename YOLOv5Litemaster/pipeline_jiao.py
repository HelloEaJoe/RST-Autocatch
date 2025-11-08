import cv2
import numpy as np
import time
from detectpipeline import *


def findsmallcontour(mask, frame_copy):
    deletelist = []
    center = None
    # top_point = None
    # bottom_point = None
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame_copy, contours, -1, (0, 255, 0), 3)
    dis_center = 320
    for cnt in contours:
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        left_point_x = np.min(box[:, 0])
        right_point_x = np.max(box[:, 0])
        top_point_y = np.min(box[:, 1])
        bottom_point_y = np.max(box[:, 1])
        # box = [left_point_x, right_point_x, top_point_y, bottom_point_y]
        long = int(max(rect[1][0], rect[1][1]))
        width = int(min(rect[1][0], rect[1][1]))
        if width * long > 2500:
            center_test = (int((left_point_x + right_point_x) / 2), int((top_point_y + bottom_point_y) / 2))
            if abs(center_test[0] - 160) < dis_center:
                dis_center = center_test[0]
                center = center_test
                # top_point = (center_test[0], top_point_y)
                # bottom_point = (center_test[0], bottom_point_y - 5)
        else:
            deletelist.append(cnt)
    for cnt in deletelist:
        x, y, w, h = cv2.boundingRect(cnt)  # (x,y)是rectangle的左上角坐标， (w,h)是width和height
        y1 = y
        y2 = y + h
        x1 = x
        x2 = x + w
        mask[int(y1):int(y2), int(x1):int(x2)] = 0
    if center is not None:
        cv2.circle(frame, center, 2, (255, 0, 0), 20)
        # cv2.circle(frame, top_point, 2, (255, 0, 0), 20)
        # cv2.circle(frame, bottom_point, 2, (255, 0, 0), 20)
    return center, mask


def TubeDilate(src, dilate_size, dilate_times):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_size, dilate_size))
    for i in range(dilate_times):
        if dilate_times == 0:
            continue
        src = cv2.dilate(src, kernel)
    return src


def TubeErode(src, erode_size, erode_times):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erode_size, erode_size))
    for i in range(erode_times):
        if erode_times == 0:
            continue
        src = cv2.erode(src, kernel)
    return src


def FollowMode_GetBottomTarget(src):
    dst = src.copy()
    bImg, gImg, rImg = cv2.split(dst)
    color_img1 = cv2.subtract(rImg, bImg)
    ret, bin = cv2.threshold(color_img1, 10, 150, cv2.THRESH_BINARY)
    bin[0:200, 0:320] = 0
    bin[0:320, 0:50] = 0
    bin[0:320, 270:320] = 0
    bin = TubeErode(bin, 4, 1)
    bin = TubeDilate(bin, 4, 3)
    center, bin = findsmallcontour(bin, bin)
    contours, hierarchy = cv2.findContours(bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # cv2.drawContours(src, contours, -1, (0, 255, 0), 20)
    # cv2.imshow('bin1', src)
    return center

def patrol(img, Boolean):
    center = None
    if Boolean:
        center = FollowMode_GetBottomTarget(img)
    return center    


if __name__ == '__main__':
    net = yolov5_lite("pipeline_best.onnx", "pipeline.names", confThreshold=0.25, nmsThreshold=0.3)
    vs = cv2.VideoCapture()
    vs.open("4.mkv")
    count = 0
    # vs.open("/home/wangguocun/文档/QGroundControl/Video/3.mkv")
    while True:
        ret, frame = vs.read()
        if ret:
            time1 = time.time()
            count = (count + 1) % 5
            frame = cv2.resize(frame, (320, 320))
            center = patrol(frame, True)
            turn_now = False
            if count == 0:           
                frame, turn_now = net.detect(frame)
                cv2.imshow("cap1", frame)
                cv2.waitKey(1)
            time2 = time.time()
            if turn_now:
                print(turn_now)
            # print("time: ", time2 - time1)
            cv2.imshow("cap", frame)
            cv2.waitKey(1)
                       
