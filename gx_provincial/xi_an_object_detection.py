import cv2
import numpy as np

src = cv2.imread(r"C:\Users\Luke\Desktop\demo.png")
cv2.namedWindow("input", cv2.WINDOW_AUTOSIZE)
cv2.imshow("input", src)
"""
提取图中的红色部分
"""
# 获取图像的高度和宽度
height, width, _ = src.shape

# 计算高度的一半附近的行范围
row_start = height // 2 - 10  # 取中间行的上方10行
row_end = height // 2 + 10  # 取中间行的下方10行

col_start = width // 2 - 200
col_end = width // 2 + 200

# middle_region = src[row_start:row_end, :, :]
middle_region = src[row_start:row_end, col_start:col_end, :]
#opencv里读图是BGR！！！！！！！！！！！！！！！！
low_bgr = np.array([115, 120, 120])
high_bgr = np.array([140, 150, 135])
mask = cv2.inRange(middle_region, lowerb=low_bgr, upperb=high_bgr)

coords = np.column_stack(np.where(mask > 0))
# 如果找到保留的区域，计算中点
if coords.size > 0:
    center = np.mean(coords, axis=0).astype(int)
    # 在原图上绘制中点
    cv2.circle(src, (center[1]+200, center[0] + row_start), 5, (0, 255, 0), -1)  # 在原图上绘制中点

cv2.imshow("test", mask)
cv2.imshow("Result", src)
cv2.waitKey(0)
cv2.destroyAllWindows()