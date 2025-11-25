# BGR与YUV
BGR，即三色图。cv读取jpg等图片文件(BGR的压缩编码)会自动转
```
cv2.imread("xxx.jpg") → BGR
cv2.imwrite("xxx.jpg")
```
YUV，亮度和色度,可以节省带宽,没有存储的文件格式,直接是原始像素。需要转化
```
import numpy as np
import cv2
W, H = 1280, 720
yuv = np.fromfile("frame_i420.yuv", dtype=np.uint8)
yuv = yuv.reshape((H * 3 // 2, W))
bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
cv2.imwrite("frame_from_i420.jpg", bgr)
```
# H.265与YUY2
H.265是视频压缩格式,因为视频有连续性,连续帧之间区别不大
YUY2是YUV的一种，不同任务需求不同
| 名字                    | 适用     | 分辨率关系                 |
| --------------------- | ------ | --------------------- |
| **YUV444**            | 最完整    | Y:W×H / U:W×H / V:W×H |
| **YUV422（YUY2）**      | 摄像头常用  | 色度水平减半                |
| **YUV420（NV12/I420）** | 视频硬件常用 | 色度水平、垂直都减半            |
