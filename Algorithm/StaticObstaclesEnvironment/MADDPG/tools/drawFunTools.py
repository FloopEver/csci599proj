#!/usr/bin/python
# -*- coding: utf-8 -*-
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

def drawEnv(apf):  # 绘制环境
    fig = plt.figure()
    apf.ax = Axes3D(fig)
    plt.grid(True)  # 添加网格
    apf.ax.scatter3D(apf.qgoal[0], apf.qgoal[1], apf.qgoal[2], marker='o', color='red', s=100, label='Goal')
    apf.ax.scatter3D(apf.x0[0], apf.x0[1], apf.x0[2], marker='o', color='blue', s=100, label='Start')
    for i in range(apf.Robstacle.shape[0]):  # 绘制球
        drawSphere(apf, apf.obstacle[i, :], apf.Robstacle[i])
    for i in range(apf.cylinder.shape[0]):  # 绘制圆柱体
        drawCylinder(apf, apf.cylinder[i, :], apf.cylinderR[i], apf.cylinderH[i])
    plt.legend(loc='best')  # 设置 图例所在的位置 使用推荐位置
    plt.grid()
    apf.ax.set_xlim3d(left=0, right=10)
    apf.ax.set_ylim3d(bottom=0, top=10)
    apf.ax.set_zlim3d(bottom=0, top=10)


def drawSphere(apf, center, radius):  # 绘制球函数
    u = np.linspace(0, 2 * np.pi, 40)
    v = np.linspace(0, np.pi, 40)
    x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
    y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
    h = apf.ax.plot_wireframe(x, y, z, cstride=4, color='b')
    return h


def drawCylinder(apf, center, radius, height):  # 绘制圆柱体函数
    u = np.linspace(0, 2 * np.pi, 30)  # 把圆分按角度为50等分
    h = np.linspace(0, height, 20)  # 把高度均分为20份
    x = np.outer(center[0] + radius * np.sin(u), np.ones(len(h)))  # x值重复20次
    y = np.outer(center[1] + radius * np.cos(u), np.ones(len(h)))  # y值重复20次
    z = np.outer(np.ones(len(u)), h)  # x，y 对应的高度
    h = apf.ax.plot_surface(x, y, z)
    return h
