

import cv2
import numpy as np
import random
import copy
import math


class State(object):
    def __init__(self, pos):
        self.pos = pos
        self.b = None
        self.t = 'NEW'
        self.h = float('inf')
        self.k = float('inf')
        self.is_obs = False
        return


class D_STAR(object):
    def __init__(self, map_path, qstart, qgoal, grid_size):
        self.directions = [[-1, -1], [-1, 0], [-1, 1],
                           [0, -1], [0, 1],
                           [1, -1], [1, 0], [1, 1]]

        self.MapGridding(map_path, qstart, qgoal, grid_size)
        return

    def MapGridding(self, map_path, qstart, qgoal, grid_size, color=(0, 0, 0)):
        '''
        grid map 网格化地图
        '''

        self.src_map = cv2.imread(map_path)
        self.map = cv2.cvtColor(self.src_map, cv2.COLOR_BGR2GRAY)
        _, self.map = cv2.threshold(
            self.map, 0, 255, cv2.THRESH_BINARY_INV)

        self.map_shape = np.shape(self.map)
        self.grid_size = grid_size
        self.rows = int(self.map_shape[0] / grid_size)
        self.cols = int(self.map_shape[1] / grid_size)

        # 地图栅格, 初始化地图所有路径点
        self.S = []
        for row in range(self.rows):
            items_rows = []
            for col in range(self.cols):
                items_rows.append(State([row, col]))
            self.S.append(items_rows)

        for row in range(self.rows):
            for col in range(self.cols):
                if self.IsObstacle(self.S[row][col]):
                    self.S[row][col].is_obs = True

        self.qstart = self.S[int(qstart[0] / grid_size)][int(qstart[1] / grid_size)]
        self.qgoal = self.S[int(qgoal[0] / grid_size)][int(qgoal[1] / grid_size)]

        # 绘制地图栅格
        for row in range(self.rows + 1):
            pt1 = (0, int(row * self.grid_size))
            pt2 = (int(self.cols * self.grid_size), int(row * self.grid_size))
            cv2.line(self.src_map, pt1, pt2, color)
            cv2.imshow('D_STAR', self.src_map)
            cv2.waitKey(2)

        for col in range(self.cols + 1):
            pt1 = (int(col * self.grid_size), 0)
            pt2 = (int(col * self.grid_size), int(self.rows * self.grid_size))
            cv2.line(self.src_map, pt1, pt2, color)
            cv2.imshow('D_STAR', self.src_map)
            cv2.waitKey(2)

        cv2.imshow('D_STAR', self.src_map)
        cv2.waitKey(5)
        return

    def DrawObstacle(self, obs_set, color):
        '''
        画出添加障碍的节点
        '''
        for obs in obs_set:
            # 原图和二值化后的图片尺寸反的
            # 这里将边界缩小1个像素，防止出现障碍扩充到隔壁格子
            pt1 = (int(obs[1] * self.grid_size + 1),
                   int(obs[0] * self.grid_size + 1))
            pt2 = (int((obs[1] + 1) * self.grid_size - 1),
                   int((obs[0] + 1) * self.grid_size - 1))

            cv2.rectangle(self.map, pt1, pt2, 255, cv2.FILLED)
            cv2.rectangle(self.src_map, pt1, pt2, color, cv2.FILLED)

            cv2.imshow('D_STAR', self.src_map)
            cv2.waitKey(5)
        return

    def AddObstacle(self, num):
        '''
        产生 num 个随机障碍
        '''
        obs_pos = []
        obstacles = []
        color = (0, 0, 0)

        for i in range(num):

            # 排除起始点和终止点
            pos = [random.randint(0, self.rows - 1),
                   random.randint(0, self.cols - 1)]
            if pos == self.qstart.pos or pos == self.qgoal.pos:
                i -= 1
                continue

            obs_pos.append(pos)

            obs = self.S[pos[0]][pos[1]]
            obs.is_obs = True
            obstacles.append(obs)

        self.DrawObstacle(obs_pos, color)

        return obstacles

    def IsObstacle(self, x):
        '''
        检查以x点为中心的方格是否有障碍物
        '''

        # 反二值化图片，所以判断是否有障碍，就对这个区域求和即可
        row_start = x.pos[0] * self.grid_size
        col_start = x.pos[1] * self.grid_size
        area = self.map[row_start: row_start +
                        self.grid_size, col_start: col_start + self.grid_size]
        if np.sum(area):
            return True

        return False

    def MinState(self):
        if not self.open_list:
            return None

        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def GetKMin(self):
        if not self.open_list:
            return -1

        k_min = min([X.k for X in self.open_list])
        return k_min

    def Delete(self, x):
        if x in self.open_list:
            self.open_list.remove(x)
        x.t = 'CLOSED'

    def Neighbors(self, x):
        '''
        获取接邻的8个节点
        '''
        neighbors = []
        for direction in self.directions:
            row = x.pos[0] + direction[0]
            if row < 0 or row >= self.rows:
                continue

            col = x.pos[1] + direction[1]
            if col < 0 or col >= self.cols:
                continue

            neighbors.append(self.S[row][col])

        return neighbors

    def CollisionFree(self, s1, s2):
        '''
        检查以s1->s2是否可行，必要时增加s1,s2之间的栅格检查
        '''

        # half_grid_size = int(self.grid_size / 2)
        # row_start = int((s1.pos[0] + s2.pos[0]) * self.grid_size / 2)
        # col_start = int((s1.pos[1] + s2.pos[1]) * self.grid_size / 2)
        # area = self.map[row_start: row_start +
        #                 self.grid_size, col_start: col_start + self.grid_size]

        # if np.sum(area):
        #     return False

        return s1.is_obs == False and s2.is_obs == False

    def ChebyshevDistance(self, p1, p2):
        return self.grid_size * max(abs(p1[0] - p2[0]), abs(p1[1] - p2[1]))

    def EuclideanDistance(self, p1, p2):
        return self.grid_size * int(math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2))

    def DiagonalDistance(self, p1, p2):
        dx = abs(p1[0] - p2[0])
        dy = abs(p1[1] - p2[1])
        return self.grid_size * (dx + dy) + int((math.sqrt(2) - 2) * self.grid_size) * min(dx, dy)

    def Cost(self, s1, s2):
        '''
        arc cost function
        '''
        if self.CollisionFree(s1, s2):
            return self.DiagonalDistance(s1.pos, s2.pos)
        else:
            return float('inf')

    def Insert(self, x, h_new):
        if x.t == 'NEW':
            x.k = h_new
        elif x.t == 'OPEN':
            x.k = min(x.k, h_new)
        elif x.t == 'CLOSED':
            x.k = min(x.h, h_new)

        x.h = h_new
        x.t = 'OPEN'
        self.open_list.add(x)
        return

    def ProcessState(self):
        x = self.MinState()
        if x is None:
            return -1
        k_old = self.GetKMin()
        self.Delete(x)

        if k_old < x.h:
            for y in self.Neighbors(x):
                if y.h <= k_old and x.h > y.h + self.Cost(y, x):
                    x.b = y
                    x.h = y.h + self.Cost(y, x)

        if k_old == x.h:
            for y in self.Neighbors(x):
                if y.t == 'NEW' or \
                    (y.b is x and y.h != x.h + self.Cost(x, y)) or \
                        (y.b is not x and y.h > x.h + self.Cost(x, y)):
                    y.b = x
                    self.Insert(y, x.h + self.Cost(x, y))
        else:
            for y in self.Neighbors(x):
                if y.t == 'NEW' or \
                        (y.b is x and y.h != x.h + self.Cost(x, y)):
                    y.b = x
                    self.Insert(y, x.h + self.Cost(x, y))
                else:
                    if y.b is not x and y.h > x.h + self.Cost(x, y):
                        self.Insert(x, x.h)
                    else:
                        if y.b is not x and x.h > y.h + self.Cost(y, x) and \
                                y.t == 'CLOSED' and y.h > k_old:
                            self.Insert(y, y.h)

        return self.GetKMin(), x

    def ModifyCost(self, x, y, cval):
        if x.t == 'CLOSED':
            # self.Insert(x, x.b.h + self.Cost(x, x.b))
            self.Insert(x, y.h + self.Cost(x, y))
        return self.GetKMin()

    def DrawPath(self, color=None):
        '''
        路径结果， 这里坐标和cv的图片坐标相反的
        '''

        if color is None:
            color = (random.randint(0, 255),
                     random.randint(0, 255),
                     random.randint(0, 255))

        mid_node = self.qstart
        while mid_node != self.qgoal:

            # 画出网格连线
            pt1 = (int(mid_node.pos[1] * self.grid_size + self.grid_size / 2),
                   int(mid_node.pos[0] * self.grid_size + self.grid_size / 2))
            pt2 = (int(mid_node.b.pos[1] * self.grid_size + self.grid_size / 2),
                   int(mid_node.b.pos[0] * self.grid_size + self.grid_size / 2))

            cv2.line(self.src_map, pt1, pt2, color, 2)
            cv2.imshow('D_STAR', self.src_map)
            cv2.waitKey(50)

            mid_node = mid_node.b
        return

    def Planning(self):
        num = input("input obstacle numbers: ")
        try:
            input_num = eval(num)
            if type(input_num) == int:
                self.AddObstacle(input_num)
        except:
            return

        self.open_list = set()
        self.qgoal.h = 0
        self.Insert(self.qgoal, 0)
        k_min, _ = self.ProcessState()
        while k_min != -1 and self.qstart.t != 'CLOSED':
            k_min, _ = self.ProcessState()

        if k_min == -1:
            print("Not Found")
            return
        else:
            print("Found")
            self.DrawPath()

        while True:
            num = input("add obstacle numbers: ")
            try:
                input_num = eval(num)
                if type(input_num) == int:
                    self.AddObstacle(input_num)
            except:
                return

            x = self.qstart
            while x != self.qgoal:
                if x.b.is_obs:

                    # approach 1. http://www.cs.cmu.edu/~motionplanning/
                    '''
                    self.Insert(x.b, float('inf'))
                    self.ModifyCost(x, x.b, self.Cost(x, x.b))
                    for yy in self.Neighbors(x.b):
                        self.Insert(yy, yy.h)

                    k_min, tmp = self.ProcessState()
                    while x.b.k < x.b.h and k_min != -1:
                        k_min, tmp = self.ProcessState()
                    '''

                    # approach 2. paper: Optimal and Efficient Path Planning for Partially-Known Environments

                    self.Insert(x.b, float('inf'))
                    self.ModifyCost(x, x.b, self.Cost(x, x.b))
                    k_min, tmp = self.ProcessState()
                    while x.b.k < x.b.h and k_min != -1:
                        k_min, tmp = self.ProcessState()

                    if x.b.is_obs == False:
                        x = x.b
                    elif k_min == -1:
                        break
                else:
                    x = x.b

            if x == self.qgoal:
                print("Found.")
                self.DrawPath()
            else:
                print("Not Found")
                return
        return


if __name__ == "__main__":

    map_path = 'map/map500-500.png'
    qstart = [10, 10]
    qgoal = [490, 490]
    max_steps = 1000
    grid_size = 20

    d_star = D_STAR(map_path, qstart, qgoal, grid_size)
    input('press any key to start planning:')
    d_star.Planning()
    input('press any key to quit:')
