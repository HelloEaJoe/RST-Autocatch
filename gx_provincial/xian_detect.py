from YOLOv5Litemaster.python_demo.onnxruntime.v5lite import *

# 电脑的模型路径
model_path = r"D:\Luke\water\robocup\xian_auto\gx_provincial\YOLOv5Litemaster\weights\best_1.onnx"
label_path = r"D:\Luke\water\robocup\xian_auto\gx_provincial\YOLOv5Litemaster\data\xian.yaml"
# pi的模型路径
# model_path = "/home/pi/Desktop/xian_auto/gx_provincial/YOLOv5Litemaster/weights/best_1.onnx"
# label_path = "/home/pi/Desktop/xian_auto/gx_provincial/YOLOv5Litemaster/data/xian.yaml"
net = yolov5_lite(model_path, label_path, confThreshold=0.5, nmsThreshold=0.5)

cap = cv2.VideoCapture(0)
ret, frame = cap.read()
while ret:
    ret, frame = cap.read()
    process_frame = net.detect(frame)
    rectangle_coordinate = net.get_rectangle_coordinate()
    owning_classes = net.get_owning_classes()
    center = net.get_center()
    area = net.get_area()
    time_later = time.time()

    for i in range(len(rectangle_coordinate)):
        if owning_classes[i] == 'blueball':
            object_exists = True
            object_left, object_top, object_width, object_height = rectangle_coordinate[i]
            center_x, center_y = center[i]
            object_area = area[i]
            print(center[i])
            break
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

