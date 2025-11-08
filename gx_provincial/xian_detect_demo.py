import cv2
from multiprocessing import Process, Queue
from YOLOv5Litemaster.python_demo.onnxruntime.v5lite import *

# 电脑的模型路径
model_path = r"D:\Luke\water\robocup\xian_auto\gx_provincial\YOLOv5Litemaster\weights\best_1.onnx"
label_path = r"D:\Luke\water\robocup\xian_auto\gx_provincial\YOLOv5Litemaster\data\xian.yaml"
# pi的模型路径
# model_path = "/home/pi/Desktop/xian_auto/gx_provincial/YOLOv5Litemaster/weights/best_1.onnx"
# label_path = "/home/pi/Desktop/xian_auto/gx_provincial/YOLOv5Litemaster/data/xian.yaml"
net = yolov5_lite(model_path, label_path, confThreshold=0.5, nmsThreshold=0.5)


def detect_demo(name):

    frame = cv2.imread(r"D:\Luke\water\robocup\label_process\1039.jpg")
    height, width, _ = frame.shape
    process_frame = net.detect(frame)
    rectangle_coordinate = net.get_rectangle_coordinate()
    owning_classes = net.get_owning_classes()
    center = net.get_center()
    area = net.get_area()
    time_later = time.time()

    for i in range(len(rectangle_coordinate)):
        if owning_classes[i] == name:
            object_exists = True
            object_left, object_top, object_width, object_height = rectangle_coordinate[i]
            center_x, center_y = center[i]
            object_area = area[i]
            cv2.circle(frame, (int(center_x * width), int(center_y * height)), 3, (255, 0, 255), -1)  # 在原图上绘制中点
            break
    cv2.imshow('frame', frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def test():
    while True:
        print(111111)
if __name__ == '__main__':
    door_thread = Process(target=detect_demo, args=('door',))
    test_thread = Process(target=test)
    door_thread.start()
    test_thread.start()
    door_thread.join()

