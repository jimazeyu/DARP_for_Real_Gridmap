import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
from multiRobotPathPlanner import *
from PIL import Image

sys.setrecursionlimit(10000)  # 将默认的递归深度修改为10000

#裁切白边；分布棋盘格；求出在图形内部的点；遍历图形内部的点画圆，形成新的图像；判断有没有把原图覆盖；

# main
if __name__ == '__main__':
    # 读取地图
    occupancy_map = read_img("../maps//bridge//test.pgm")
    

   