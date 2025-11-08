import cv2
import time
import numpy as np
import argparse
import onnxruntime as ort

class yolov5_lite():
    def __init__(self, model_pb_path, label_path, confThreshold=0.5, nmsThreshold=0.5, objThreshold=0.5):
        so = ort.SessionOptions()
        so.log_severity_level = 3
        self.net = ort.InferenceSession(model_pb_path, so)
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
        self.input_shape = (self.net.get_inputs()[0].shape[2], self.net.get_inputs()[0].shape[3])

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

    def postprocess(self, frame, outs, pad_hw, angle_num, cube_num):
        
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
        cube = False
        angle = False
        cube_name = None
        angle_point = None
        top_num = 0
        # list1= ["pipeline_heng", "pipeline_shu","angle",
        #      "cube","circle","triangle","circular"]
        list1 = ["cube","circle","triangle","circular"]
        # list1 = ["cube","circle"]
        for i in box_index:
            #print(i)
            i = i#[0]
            box = boxes[i]
            left = box[0]
            top = box[1]
            width = box[2]
            height = box[3]
            area = width*height
            if (self.classes[classIds[i]] in list1) and area > 500 and 290 > (top + height/2) > cube_num and abs(left + width / 2 - 160) < 110:
                cube = True
                if top > top_num:              
                    cube_name = self.classes[classIds[i]]
                
            elif self.classes[classIds[i]] == "angle" and 280 > (top + height/2) > angle_num and abs(left + width / 2 - 160) < 100:
                angle = True
                angle_point = (int(left + width / 2), int(top + height/2))
            frame = self.drawPred(frame, classIds[i], confidences[i], left, top, left + width, top + height)
            
            print( left, top, left + width, top + height)
        return frame, angle, angle_point, cube, cube_name

    def drawPred(self, frame, classId, conf, left, top, right, bottom):
   
       
        # Draw a bounding box.
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), thickness=2)
        # print("draw")

       
        label = '%.2f' % conf
        label = '%s:%s' % (self.classes[classId], label)

       
        # Display the label at the top of the bounding box
        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, labelSize[1])
        # cv.rectangle(frame, (left, top - round(1.5 * labelSize[1])), (left + round(1.5 * labelSize[0]), top + baseLine), (255,255,255), cv.FILLED)
        cv2.putText(frame, label, (left, top - 10), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0), thickness=1)
        return frame

    def detect(self, srcimg, angle_num, cube_num):
        """这段代码是目标检测模型（尤其可能是类 YOLO 结构的模型）后处理阶段的核心逻辑，主要作用是将模型输出的
        原始预测结果（如边界框偏移量、宽高缩放因子）转换为图像上的实际坐标，
        并为后续的后处理（如角度计算、立方体检测）做准备。
        """
        img, newh, neww, top, left = self.resize_image(srcimg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        blob = np.expand_dims(np.transpose(img, (2, 0, 1)), axis=0)

        t1 = time.time()
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
        srcimg, angle, angle_point, cube, cube_name = self.postprocess(srcimg, outs, (newh, neww, top, left), angle_num, cube_num)
        infer_time = 'Inference Time: ' + str(int(cost_time * 1000)) + 'ms'
        cv2.putText(srcimg, infer_time, (5, 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 0, 0), thickness=1)
        return srcimg, angle, angle_point, cube, cube_name


if __name__ == '__main__':
    srcimg = cv2.imread('/home/wangguocun/pipeline/picture2/1664.jpg')
    srcimg = cv2.resize(srcimg, (320, 320))
    net = yolov5_lite("best-4.21.onnx", "cube.names", confThreshold=0.1, nmsThreshold=0.3)
    srcimg, turn_now = net.detect(srcimg)
    winName = 'Deep learning object detection in onnxruntime'
    # cv2.namedWindow(winName, cv2.WINDOW_NORMAL)
    cv2.imshow(winName, srcimg)
    cv2.waitKey(0)
    # cv2.imwrite('save.jpg', srcimg )
    cv2.destroyAllWindows()
