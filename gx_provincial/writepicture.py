import cv2

# 打开摄像头
cap = cv2.VideoCapture(0)

# 创建窗口并显示图像
cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)

# 截图计数器
count = 0

while True:
    # 读取摄像头帧
    ret, frame = cap.read()

    # 检查是否成功读取帧
    if not ret:
        print("Failed to read frame from camera!")
        break

    # 显示图像
    cv2.imshow("Camera", frame)

    # 检测按键
    key = cv2.waitKey(1)
    if key == ord('q'):  # 如果按下'q'键，退出程序
        break
    elif key == ord('s'):  # 如果按下's'键，保存当前帧
        count += 1
        filename = "./cap/{:04d}.png".format(count)
        cv2.imwrite(filename, frame)


# 释放摄像头并关闭窗口
cap.release()
cv2.destroyAllWindows()
