import time
import queue
import cv2
import numpy as np

def make_print_to_file(path='./'):
    '''
    path， it is a path for save your log about fuction print
    example:
    use  make_print_to_file()   and the   all the information of funtion print , will be write in to a log file
    :return:
    '''
    import sys
    import os
    import sys
    import datetime
 
    class Logger(object):
        def __init__(self, filename="Default.log", path="./"):
            self.terminal = sys.stdout
            self.path= os.path.join(path, filename)
            self.log = open(self.path, "a", encoding='utf8',)
            print("save:", os.path.join(self.path, filename))
 
        def write(self, message):
            self.terminal.write(message)
            self.log.write(message)
 
        def flush(self):
            pass

    fileName = datetime.datetime.now().strftime('day'+'%Y_%m_%d')
    sys.stdout = Logger(fileName + '.log', path=path)
 
    #############################################################
    # 这里输出之后的所有的输出的print 内容即将写入日志
    #############################################################
    print(fileName.center(60,'*'))

q_center=queue.LifoQueue(-1)
q_width=queue.LifoQueue(-1)

def findsmallcontour(mask, frame_copy,frame):
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
        length = int(max(rect[1][0], rect[1][1]))
        width = int(min(rect[1][0], rect[1][1]))
        if width * length > 2500:
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
    bin[0:150, 0:320] = 0
    bin[0:320, 0:20] = 0
    bin[0:320, 300:320] = 0
    # cv2.imshow("img2", bin)
    # means, dev = cv2.meanStdDev(bin_part)
    # print(means)
    bin = TubeErode(bin, 4, 1)
    bin = TubeDilate(bin, 4, 3)
    center, bin = findsmallcontour(bin, bin,src)
    #print(center)
    q_center.put(center)
    bin_part = bin[150:300, 20:300]
    # cv2.imshow("bin_part", bin_part)
    contours, hierarchy = cv2.findContours(bin_part, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    area_max = 0
    count_max = -1
    for i, cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        if area > area_max and area > 1000:
            count_max = i
            area_max = area
    m = None
    means, dev = cv2.meanStdDev(bin_part)
    if count_max != -1:
        rect = cv2.minAreaRect(contours[count_max])
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        # left_point_x = np.min(box[:, 0])
        # right_point_xdef FollowMode_GetBottomTarget(src): = np.max(box[:, 0])
        # top_point_y = np.min(box[:, 1])
        # bottom_point_y = np.max(box[:, 1])
        rect = order_points_new(box)
        # print(rect)
        # print(box)
        # long = int(max(rect[1][0], rect[1][1]))
        # width = int(min(rect[1][0], rect[1][1]))
        if box[1][1] - box[2][1] == 0:
            m = 0
        else:
            m = (rect[1][0] - rect[2][0]) / (rect[1][1] - rect[2][1])
        cv2.drawContours(src, [box], 0, (0, 0, 255), 3)
    # if m is not None:
    #     m = round(m)
    # print("m:", m, "mean:", int(means[0][0]))

    # contours, hierarchy = cv2.findContours(bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # # cv2.drawContours(src, contours, -1, (0, 255, 0), 20)
    # cv2.imshow('bin1', src)
    return m, int(means[0][0])


def order_points_new(pts):
    # sort the points based on their x-coordinates
    xSorted = pts[np.argsort(pts[:, 0]), :]

    # grab the left-most and right-most points from the sorted
    # x-roodinate points
    leftMost = xSorted[:2, :]
    rightMost = xSorted[2:, :]
    if leftMost[0, 1] != leftMost[1, 1]:
        leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    else:
        leftMost = leftMost[np.argsort(leftMost[:, 0])[::-1], :]
    (tl, bl) = leftMost
    if rightMost[0, 1] != rightMost[1, 1]:
        rightMost = rightMost[np.argsort(rightMost[:, 1]), :]
    else:
        rightMost = rightMost[np.argsort(rightMost[:, 0])[::-1], :]
    (tr, br) = rightMost
    # return the coordinates in top-left, top-right,
    # bottom-right, and bottom-left order
    return np.array([tl, tr, br, bl], dtype="float32")


def patrol(img, Boolean):
    if Boolean:
        m, means = FollowMode_GetBottomTarget(img)
        cv2.imshow("cap", img)
        cv2.waitKey(1)
        q_width.put(means)
    return m, means


if __name__ == '__main__':
    make_print_to_file(path='./')
    camera = cv2.VideoCapture(0)
    while True:
        ret, frame = camera.read()
        if ret:
            frame = cv2.resize(frame, (320, 320))
            m, means = patrol(frame, True)
            print("m:", m, "mean:", int(means))
