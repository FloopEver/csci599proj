#!/usr/bin/python
# -*- coding: utf-8 -*-

# APF
import numpy as np
import matplotlib.pyplot as plt

from Algorithm.StaticObstaclesEnvironment.MADDPG.tools.calculateFunTools import distanceCost, getUnitVec, angleVec
from Algorithm.StaticObstaclesEnvironment.MADDPG.tools.drawFunTools import drawEnv
from Algorithm.StaticObstaclesEnvironment.MADDPG.environmentSetup.StaticObstacles import Obstacle


class APF:
    def __init__(self):
        #-------------------obstacles-------------------#
        env = 'Obstacle1'  #虽然我这里写的障碍物1，好像有很多障碍物环境的样子，但其实就这一个，这就是你要实现的地方啦
        self.obstacle = Obstacle[env].obstacle      # 球坐标
        self.Robstacle = Obstacle[env].Robstacle    # 球半径
        self.cylinder = Obstacle[env].cylinderPos      # 圆柱体坐标
        self.cylinderR = Obstacle[env].cylinderR    # 圆柱体半径
        self.cylinderH  = Obstacle[env].cylinderH   # 圆柱体高度
        self.cone = Obstacle[env].conePos              # 圆锥底面中心坐标
        self.coneR = Obstacle[env].coneR            # 圆锥底面圆半径
        self.coneH = Obstacle[env].coneH            # 圆锥高度
        self.numberOfSphere = self.obstacle.shape[0]       # 球形障碍物的数量
        self.numberOfCylinder = self.cylinder.shape[0]     # 圆柱体障碍物的数量
        self.numberOfCone = self.cone.shape[0]             # 圆锥障碍物的数量

        self.x0 = Obstacle[env].x0  # 起始点
        self.qgoal = Obstacle[env].qgoal          # 目标点
        self.stepSize = 0.2                               # 物体移动的固定步长
        self.dgoal = 5                                    # 当q与qgoal距离超过它时将衰减一部分引力
        self.r0 = 5                                       # 斥力超过这个范围后将不复存在
        self.threshold = 0.2                              # q与qgoal距离小于它时终止训练或者仿真
        #------------运动学约束------------#
        self.xmax = 10/180 * np.pi
        self.gammax = 10/180 * np.pi
        self.maximumClimbingAngle = 100/180 * np.pi
        self.maximumSubductionAngle = - 75 / 180 * np.pi

        #-------------路径---------#
        self.path = self.x0.copy()
        self.path = self.path[np.newaxis, :]              # 增加一个维度

        #-------------参考参数-------------#
        self.epsilon0 = 0.8
        self.eta0 = 0.5

    def reset(self):        # reset environment
        self.path = self.x0.copy()
        self.path = self.path[np.newaxis, :]


    def calculateDynamicState(self, q):
        dic = {'sphere':[], 'cylinder':[], 'cone':[]}
        sAll = self.qgoal - q
        for i in range(self.numberOfSphere):
            s1 = self.obstacle[i,:] - q
            dic['sphere'].append(np.hstack((s1,sAll)))
        for i in range(self.numberOfCylinder):
            s1 = np.hstack((self.cylinder[i,:],q[2])) - q
            dic['cylinder'].append(np.hstack((s1, sAll)))
        for i in range(self.numberOfCone):
            s1 = np.hstack((self.cone[i,:],self.coneH[i]/2)) - q
            dic['cone'].append(np.hstack((s1,sAll)))
        return dic

    def inRepulsionArea(self, q):  # 计算一个点位r0半径范围内的障碍物索引, 返回dic{'sphere':[1,2,..],'cylinder':[0,1,..]}
        dic = {'sphere':[], 'cylinder':[], 'cone':[]}
        for i in range(self.numberOfSphere):
            if distanceCost(q, self.obstacle[i,:]) < self.r0:
                dic['sphere'].append(i)
        for i in range(self.numberOfCylinder):
            if distanceCost(q[0:2], self.cylinder[i,:]) < self.r0:
                dic['cylinder'].append(i)
        for i in range(self.numberOfCone):
            if distanceCost(q[0:2], np.hstack((self.cone[i,:],self.coneH[i]/2))) <self.r0:
                dic['cone'].append(i)
        return dic


    def attraction(self, q, epsilon):  # 计算引力，反正公式都是抄论文上的，显然我只是个微不足道的搬运工
        r = distanceCost(q, self.qgoal)
        if r <= self.dgoal:
            fx = epsilon * (self.qgoal[0] - q[0])
            fy = epsilon * (self.qgoal[1] - q[1])
            fz = epsilon * (self.qgoal[2] - q[2])
        else:
            fx = self.dgoal * epsilon * (self.qgoal[0] - q[0]) / r
            fy = self.dgoal * epsilon * (self.qgoal[1] - q[1]) / r
            fz = self.dgoal * epsilon * (self.qgoal[2] - q[2]) / r
        return np.array([fx, fy, fz])


    def repulsionForOneObstacle(self, q, eta, qobs): #斥力计算，当然也是抄的，说好听一点，我实现的
        f0 = np.array([0, 0, 0])
        Rq2qgoal = distanceCost(q, self.qgoal)
        r = distanceCost(q, qobs)
        if r <= self.r0:
            tempfvec = eta * (1 / r - 1 / self.r0) * Rq2qgoal ** 2 / r ** 2 * self.differential(q, qobs) \
                       + eta * (1 / r - 1 / self.r0) ** 2 * Rq2qgoal * self.differential(q, self.qgoal)
            f0 = f0 + tempfvec
        else:
            tempfvec = np.array([0, 0, 0])
            f0 = f0 + tempfvec
        return f0

    def differential(self, q, other):   #向量微分
        output1 = (q[0] - other[0]) / distanceCost(q, other)
        output2 = (q[1] - other[1]) / distanceCost(q, other)
        output3 = (q[2] - other[2]) / distanceCost(q, other)
        return np.array([output1, output2, output3])

    def getqNext(self, epsilon, eta1List, eta2List, eta3List, q, qBefore):
        qBefore = np.array(qBefore)
        if qBefore[0] is None:
            unitCompositeForce = self.getUnitCompositeForce(q, eta1List, eta2List, eta3List, epsilon)
            qNext = q + self.stepSize * unitCompositeForce  # 计算下一位置
        else:
            unitCompositeForce = self.getUnitCompositeForce(q, eta1List, eta2List, eta3List, epsilon)
            qNext = q + self.stepSize * unitCompositeForce  # 计算下一位置
            _, _, _, _, qNext = self.kinematicConstrant(q, qBefore, qNext)
        self.path = np.vstack((self.path, qNext))  # 记录轨迹
        return qNext

    def getUnitCompositeForce(self,q,eta1List, eta2List, eta3List, epsilon):
        Attraction = self.attraction(q, epsilon)  # 计算引力，我真的感觉就是做这玩意害得学好物理力的分析
        Repulsion = np.array([0,0,0])
        for i in range(len(eta1List)): #对每个球形障碍物分别计算斥力并相加
            Repulsion = Repulsion + self.repulsionForOneObstacle(q, eta1List[i], self.obstacle[i,:])
        for i in range(len(eta2List)):
            Repulsion = Repulsion + self.repulsionForOneObstacle(q, eta2List[i], np.hstack((self.cylinder[i,:],q[2])))
        for i in range(len(eta3List)):
            Repulsion = Repulsion + self.repulsionForOneObstacle(q, eta3List[i], np.hstack((self.cone[i,:],self.coneH[i]/2)))
        compositeForce = Attraction + Repulsion  # 合力 = 引力 + 斥力
        unitCompositeForce = getUnitVec(compositeForce)  # 力单位化，apf中力只用来指示移动方向
        return unitCompositeForce

    def kinematicConstrant(self, q, qBefore, qNext):    #运动学约束函数 返回(上一时刻航迹角，上一时刻爬升角，约束后航迹角，约束后爬升角，约束后下一位置qNext)
        # 计算qBefore到q航迹角x1,gam1
        qBefore2q = q - qBefore
        if qBefore2q[0] != 0 or qBefore2q[1] != 0:
            x1 = np.arcsin(np.abs(qBefore2q[1] / np.sqrt(qBefore2q[0] ** 2 + qBefore2q[1] ** 2)))  # 这里计算的角限定在了第一象限的角 0-pi/2
            gam1 = np.arcsin(qBefore2q[2] / np.sqrt(np.sum(qBefore2q ** 2)))
        else:
            return None, None, None, None, qNext
        # 计算q到qNext航迹角x2,gam2
        q2qNext = qNext - q
        x2 = np.arcsin(np.abs(q2qNext[1] / np.sqrt(q2qNext[0] ** 2 + q2qNext[1] ** 2)))  # 这里同理计算第一象限的角度
        gam2 = np.arcsin(q2qNext[2] / np.sqrt(np.sum(q2qNext ** 2)))

        # 根据不同象限计算矢量相对于x正半轴的角度 0-2 * pi
        if qBefore2q[0] > 0 and qBefore2q[1] > 0:
            x1 = x1
        if qBefore2q[0] < 0 and qBefore2q[1] > 0:
            x1 = np.pi - x1
        if qBefore2q[0] < 0 and qBefore2q[1] < 0:
            x1 = np.pi + x1
        if qBefore2q[0] > 0 and qBefore2q[1] < 0:
            x1 = 2 * np.pi - x1
        if qBefore2q[0] > 0 and qBefore2q[1] == 0:
            x1 = 0
        if qBefore2q[0] == 0 and qBefore2q[1] > 0:
            x1 = np.pi / 2
        if qBefore2q[0] < 0 and qBefore2q[1] == 0:
            x1 = np.pi
        if qBefore2q[0] == 0 and qBefore2q[1] < 0:
            x1 = np.pi * 3 / 2

        # 根据不同象限计算与x正半轴的角度
        if q2qNext[0] > 0 and q2qNext[1] > 0:
            x2 = x2
        if q2qNext[0] < 0 and q2qNext[1] > 0:
            x2 = np.pi - x2
        if q2qNext[0] < 0 and q2qNext[1] < 0:
            x2 = np.pi + x2
        if q2qNext[0] > 0 and q2qNext[1] < 0:
            x2 = 2 * np.pi - x2
        if q2qNext[0] > 0 and q2qNext[1] == 0:
            x2 = 0
        if q2qNext[0] == 0 and q2qNext[1] > 0:
            x2 = np.pi / 2
        if q2qNext[0] < 0 and q2qNext[1] == 0:
            x2 = np.pi
        if q2qNext[0] == 0 and q2qNext[1] < 0:
            x2 = np.pi * 3 / 2

        # 约束航迹角x   xres为约束后的航迹角
        deltax1x2 = angleVec(q2qNext[0:2], qBefore2q[0:2])  # 利用点乘除以模长乘积求xoy平面投影的夹角
        if deltax1x2 < self.xmax:
            xres = x2
        elif x1 - x2 > 0 and x1 - x2 < np.pi:
            xres = x1 - self.xmax
        elif x1 - x2 > 0 and x1 - x2 > np.pi:
            xres = x1 + self.xmax
        elif x1 - x2 < 0 and x2 - x1 < np.pi:
            xres = x1 + self.xmax
        else:
            xres = x1 - self.xmax

        # 约束爬升角gam
        if np.abs(gam1 - gam2) <= self.gammax:
            gamres = gam2
        elif gam2 > gam1:
            gamres = gam1 + self.gammax
        else:
            gamres = gam1 - self.gammax
        if gamres > self.maximumClimbingAngle:
            gamres = self.maximumClimbingAngle
        if gamres < self.maximumSubductionAngle:
            gamres = self.maximumSubductionAngle

        # 计算约束过后下一个点qNext的坐标
        Rq2qNext = distanceCost(q, qNext)
        deltax = Rq2qNext * np.cos(gamres) * np.cos(xres)
        deltay = Rq2qNext * np.cos(gamres) * np.sin(xres)
        deltaz = Rq2qNext * np.sin(gamres)

        qNext = q + np.array([deltax, deltay, deltaz])
        return x1, gam1, xres, gamres, qNext

    def checkCollision(self, q):     #碰撞返回[0,障碍物类型index, 碰撞障碍物index]，没有碰撞返回[1,-1, -1]
        for i in range(self.numberOfSphere):#球碰撞
            if distanceCost(q, self.obstacle[i, :]) <= self.Robstacle[i]:
                return np.array([0,0,i])
        for i in range(self.numberOfCylinder): #圆柱体碰撞
            if 0 <= q[2] <= self.cylinderH[i] and distanceCost(q[0:2], self.cylinder[i, :]) <= self.cylinderR[i]:
                return np.array([0,1,i])
        for i in range(self.numberOfCone):
            if q[2] >= 0 and distanceCost(q[0:2], self.cone[i, :]) <= self.coneR[i] - q[2] * self.coneR[i] / self.coneH[i]:
                return np.array([0,2,i])
        return np.array([1, -1, -1])   #不撞

    def calculateLength(self):
        sum = 0
        for i in range(self.path.shape[0] - 1):
            sum += distanceCost(self.path[i, :], self.path[i + 1, :])
        return sum

    def loop(self):
        q = self.x0.copy()
        qBefore = [None, None, None]
        eta1List = [0.2 for i in range(self.obstacle.shape[0])]
        eta2List = [0.2 for i in range(self.cylinder.shape[0])]
        eta3List = [0.2 for i in range(self.cone.shape[0])]
        for i in range(500):
            qNext = self.getqNext(self.epsilon0,eta1List,eta2List,eta3List,q,qBefore)  # 功能如名字，就是算下一个点呗
            qBefore = q  # 一整个连续的大动作
            print(apf.checkCollision(q))  #我看看输出对不对
            self.ax.plot3D([q[0], qNext[0]], [q[1], qNext[1]], [q[2], qNext[2]], color="k",linewidth=2)  # 绘制上一位置和这一位置
            q = qNext
            if distanceCost(qNext,self.qgoal) < self.threshold: # 咱就是说到家了诶兄弟们
                self.path = np.vstack((self.path,self.qgoal))
                self.ax.plot3D([qNext[0], self.qgoal[0]], [qNext[1], self.qgoal[1]], [qNext[2], self.qgoal[2]], color="k",linewidth=2)  # 绘制上一位置和这一位置
                break
            plt.pause(0.001)

if __name__ == "__main__":
    apf = APF()
    drawEnv(apf)
    apf.loop()
    print('path length：', apf.calculateLength())
    plt.legend(loc='best')
    plt.show()