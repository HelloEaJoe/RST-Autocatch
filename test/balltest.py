import os
os.environ['OMP_NUM_THREADS'] = '1'

import cv2
import time
import numpy as np
import argparse
try:
    import importlib.util
    # check if the openvino.runtime module is available without raising ImportError
    if importlib.util.find_spec("openvino.runtime") is not None:
        openvino_runtime = importlib.import_module("openvino.runtime")
        Core = getattr(openvino_runtime, "Core")
        openvino_available = True
    else:
        openvino_available = False
except Exception:
    openvino_available = False
import onnxruntime as ort
import threading
import queue

class yolov5_lite():
    def __init__(self, model_pb_path, label_path, confThreshold=0.5, nmsThreshold=0.5, objThreshold=0.5, intra_op_num_threads=4, inter_op_num_threads=1):
        self.use_openvino = False
        self.openvino_input_name = None
        self.openvino_output_name = None
        if openvino_available:
            try:
                self.core = Core()
                self.ov_model = self.core.read_model(model_pb_path)
                self.compiled_model = self.core.compile_model(self.ov_model, "CPU")
                self.openvino_input_name = self.compiled_model.inputs[0].get_any_name()
                self.openvino_output_name = self.compiled_model.outputs[0].get_any_name()
                self.use_openvino = True
                print("使用OpenVINO runtime 加速推理")
            except Exception as e:
                print("OpenVINO runtime 加载失败，回退到 ONNXRuntime CPU", e)
        if not self.use_openvino:
            so = ort.SessionOptions()
            so.log_severity_level = 3
            so.intra_op_num_threads = intra_op_num_threads
            so.inter_op_num_threads = inter_op_num_threads
            self.net = ort.InferenceSession(model_pb_path, so, providers=['CPUExecutionProvider'])
            print("使用 ONNXRuntime CPU 推理")
        self.classes = list(map(lambda x: x.strip(), open(label_path, 'r').readlines()))
        self.num_classes = len(self.classes)
        anchors = [[10, 13, 16, 30, 33, 23], [30, 61, 62, 45, 59, 119], [116, 90, 156, 198, 373, 326]]
        self.nl = len(anchors)
        self.na = len(anchors[0]) // 2
        self.no = self.num_classes + 5
        self.grid = [np.zeros(1)] * self.nl
        self.stride = np.array([8., 16., 32.])
        self.anchor_grid = np.asarray(anchors, dtype=np.float32).reshape(self.nl, -1, 2)

        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.objThreshold = objThreshold
        # input_shape 取决于推理后端
        if self.use_openvino:
            shape = self.compiled_model.input(0).shape
            self.input_shape = (shape[2], shape[3])
        else:
            self.input_shape = (self.net.get_inputs()[0].shape[2], self.net.get_inputs()[0].shape[3])
        self.net = None  # 默认初始化为 None，避免未定义错误
        print("OpenVINO 可用性:", openvino_available)
        if self.use_openvino:
            print("OpenVINO 初始化成功")
            self.net = self.compiled_model  # 确保 OpenVINO 模式下 self.net 被正确赋值
        else:
            print("使用 ONNXRuntime 初始化")

    def resize_image(self, srcimg, keep_ratio=True):
        top, left, newh, neww = 0, 0, self.input_shape[0], self.input_shape[1]
        if keep_ratio and srcimg.shape[0] != srcimg.shape[1]:
            hw_scale = srcimg.shape[0] / srcimg.shape[1]
            if hw_scale > 1:
                newh, neww = self.input_shape[0], int(self.input_shape[1] / hw_scale)
                img = cv2.resize(srcimg, (neww, newh), interpolation=cv2.INTER_AREA)
                left = int((self.input_shape[1] - neww) * 0.5)
                img = cv2.copyMakeBorder(img, 0, 0, left, self.input_shape[1] - neww - left, cv2.BORDER_CONSTANT,
                                         value=0)  # add border
            else:
                newh, neww = int(self.input_shape[0] * hw_scale), self.input_shape[1]
                img = cv2.resize(srcimg, (neww, newh), interpolation=cv2.INTER_AREA)
                top = int((self.input_shape[0] - newh) * 0.5)
                img = cv2.copyMakeBorder(img, top, self.input_shape[0] - newh - top, 0, 0, cv2.BORDER_CONSTANT, value=0)
        else:
            img = cv2.resize(srcimg, self.input_shape, interpolation=cv2.INTER_AREA)
        return img, newh, neww, top, left

    def _make_grid(self, nx=20, ny=20):
        xv, yv = np.meshgrid(np.arange(ny), np.arange(nx))
        return np.stack((xv, yv), 2).reshape((-1, 2)).astype(np.float32)

    def postprocess(self, frame, outs, pad_hw):
        newh, neww, padh, padw = pad_hw
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]
        ratioh, ratiow = frameHeight / newh, frameWidth / neww
        # Scan through all the bounding boxes output from the network and keep only the
        # ones with high confidence scores. Assign the box's class label as the class with the highest score.
        classIds = []
        confidences = []
        box_index = []
        boxes = []
        for detection in outs:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > self.confThreshold and detection[4] > self.objThreshold:
                center_x = int((detection[0] - padw) * ratiow)
                center_y = int((detection[1] - padh) * ratioh)
                width = int(detection[2] * ratiow)
                height = int(detection[3] * ratioh)
                left = int(center_x - width / 2)
                top = int(center_y - height / 2)
                classIds.append(classId)
                confidences.append(float(confidence))
                boxes.append([left, top, width, height])

        # Perform non maximum suppression to eliminate redundant overlapping boxes with
        # lower confidences.
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confThreshold, self.nmsThreshold)

        for i in indices:
            box_index.append(i)


        for i in box_index:
            box = boxes[i]
            left = box[0]
            top = box[1]
            width = box[2]
            height = box[3]
            frame = self.drawPred(frame, classIds[i], confidences[i], left, top, left + width, top + height)
        return frame

    def drawPred(self, frame, classId, conf, left, top, right, bottom):
        # Draw a bounding box.
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), thickness=2)

        label = '%.2f' % conf
        label = '%s:%s' % (self.classes[classId], label)

        # Display the label at the top of the bounding box
        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, labelSize[1])
        # cv.rectangle(frame, (left, top - round(1.5 * labelSize[1])), (left + round(1.5 * labelSize[0]), top + baseLine), (255,255,255), cv.FILLED)
        cv2.putText(frame, label, (left, top - 10), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), thickness=1)
        return frame

    def detect(self, srcimg):
        img, newh, neww, top, left = self.resize_image(srcimg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        blob = np.expand_dims(np.transpose(img, (2, 0, 1)), axis=0)

        t1 = time.time()
        if self.use_openvino:
            result = self.compiled_model([blob])[self.openvino_output_name]
            outs = result.squeeze(axis=0)
        else:
            outs = self.net.run(None, {self.net.get_inputs()[0].name: blob})[0].squeeze(axis=0)
        cost_time = time.time() - t1

        row_ind = 0
        for i in range(self.nl):
            h, w = int(self.input_shape[0] / self.stride[i]), int(self.input_shape[1] / self.stride[i])
            length = int(self.na * h * w)
            if self.grid[i].shape[2:4] != (h, w):
                self.grid[i] = self._make_grid(w, h)

            outs[row_ind:row_ind + length, 0:2] = (outs[row_ind:row_ind + length, 0:2] * 2. - 0.5 + np.tile(
                self.grid[i], (self.na, 1))) * int(self.stride[i])
            outs[row_ind:row_ind + length, 2:4] = (outs[row_ind:row_ind + length, 2:4] * 2) ** 2 * np.repeat(
                self.anchor_grid[i], h * w, axis=0)
            row_ind += length

        # 获取检测结果
        detected_boxes = []
        newh, neww, padh, padw = newh, neww, top, left
        frameHeight = srcimg.shape[0]
        frameWidth = srcimg.shape[1]
        ratioh, ratiow = frameHeight / newh, frameWidth / neww

        for detection in outs:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > self.confThreshold and detection[4] > self.objThreshold:
                center_x = int((detection[0] - padw) * ratiow)
                center_y = int((detection[1] - padh) * ratioh)
                width = int(detection[2] * ratiow)
                height = int(detection[3] * ratioh)
                detected_boxes.append({
                    'class_id': classId,
                    'class_name': self.classes[classId],
                    'confidence': float(confidence),
                    'x': center_x,
                    'y': center_y,
                    'width': width,
                    'height': height
                })

        # 绘制检测框
        for box in detected_boxes:
            srcimg = self.drawPred(srcimg, box['class_id'], box['confidence'], 
                                   box['x'] - box['width'] // 2, box['y'] - box['height'] // 2,
                                   box['x'] + box['width'] // 2, box['y'] + box['height'] // 2)

        infer_time = 'Inference Time: ' + str(int(cost_time * 1000)) + 'ms'
        cv2.putText(srcimg, infer_time, (5, 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 0, 0), thickness=1)

        return srcimg, detected_boxes


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--modelpath', type=str, default='/home/pi/autocatch/model/bestball320.onnx', help="onnx filepath")
    parser.add_argument('--classfile', type=str, default='/home/pi/autocatch/YOLOv5Litemaster/cube.names', help="classname filepath")
    parser.add_argument('--confThreshold', default=0.5, type=float, help='class confidence')
    parser.add_argument('--nmsThreshold', default=0.6, type=float, help='nms iou thresh')
    args = parser.parse_args()

    # 优化onnxruntime线程
    so = ort.SessionOptions()
    so.log_severity_level = 3
    so.intra_op_num_threads = 4
    so.inter_op_num_threads = 1
    net = yolov5_lite(args.modelpath, args.classfile, confThreshold=args.confThreshold, nmsThreshold=args.nmsThreshold)
    net.net = ort.InferenceSession(args.modelpath, so, providers=['OpenVINOExecutionProvider', 'CPUExecutionProvider'])

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    winName = 'ONNX Object Detection'
    cv2.namedWindow(winName, cv2.WINDOW_AUTOSIZE)

    frame_count = 0
    start_time = time.time()
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame_count += 1
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0

        if frame_count % 2 == 0:
            result, boxes = net.detect(frame)  # ✅ 拆包两个返回值
            if isinstance(result, np.ndarray):
                cv2.putText(result, f'FPS: {fps:.2f}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow(winName, result)
            else:
                print("Warning: detect() did not return an image.")
        else:
            cv2.putText(frame, f'FPS: {fps:.2f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow(winName, frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    cap.release()
    cv2.destroyAllWindows()
    avg_fps = frame_count / (time.time() - start_time)
    print(f'平均帧率: {avg_fps:.2f} FPS')
