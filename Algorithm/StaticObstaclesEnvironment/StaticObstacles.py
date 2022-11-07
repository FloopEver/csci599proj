# static obstacle environment.py

import numpy as np
class Obstacle1:
    def __init__(self):
        self.obstacle = np.array([[7, 2, 1],
                                  [7, 4, 2],
                                  [5, 4, 1],
                                  [6, 2, 1]], dtype=float)  # 球障碍物坐标
        self.Robstacle = np.array([1, 2, 2, 1], dtype=float)  # 球半径
        self.cylinderPos = np.array([[6, 2]], dtype=float)  # 圆柱体障碍物坐标（圆形的x,y，无顶盖）
        self.cylinderR = np.array([1], dtype=float)  # 圆柱体障碍物半径
        self.cylinderH = np.array([5], dtype=float)  # 圆柱体高度
        self.conePos = np.array([[3, 0]], dtype=float)  # 圆锥底面中心坐标
        self.coneR = np.array([1], dtype=float)  # 圆锥底面圆半径
        self.coneH = np.array([3], dtype=float)  # 圆锥高度
        self.x0 = np.array([0, 0, 2], dtype=float)  # 起始点
        self.qgoal = np.array([8, 6, 1.2], dtype=float)  # 目标点


Obstacle = {"Obstacle1":Obstacle1()}  # 我感觉我这个注释写的不要太详细，非常简单易懂
