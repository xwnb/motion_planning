
import cv2
import copy
import numpy as np
import random
import math


class Node(object):
    def __init__(self, pos):
        self.pos = pos
        self.g = float('inf')
        self.rhs = float('inf')
        self.k = [float('inf'), float('inf')]
        self.p = None
        self.is_obs = False


class D_STAR_LITE(object):
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
                items_rows.append(Node([row, col]))
            self.S.append(items_rows)

        self.qstart = self.S[int(qstart[0] / grid_size)][int(qstart[1] / grid_size)]
        self.qgoal = self.S[int(qgoal[0] / grid_size)][int(qgoal[1] / grid_size)]
        for row in range(self.rows):
            for col in range(self.cols):
                if self.IsObstacle(self.S[row][col]):
                    self.S[row][col].is_obs = True

        # 绘制地图栅格
        for row in range(self.rows + 1):
            pt1 = (0, int(row * self.grid_size))
            pt2 = (int(self.cols * self.grid_size), int(row * self.grid_size))
            cv2.line(self.src_map, pt1, pt2, color)
            cv2.imshow('D_STAR_LITE', self.src_map)
            cv2.waitKey(2)

        for col in range(self.cols + 1):
            pt1 = (int(col * self.grid_size), 0)
            pt2 = (int(col * self.grid_size), int(self.rows * self.grid_size))
            cv2.line(self.src_map, pt1, pt2, color)
            cv2.imshow('D_STAR_LITE', self.src_map)
            cv2.waitKey(2)

        cv2.imshow('D_STAR_LITE', self.src_map)
        cv2.waitKey(50)
        return

    def IsObstacle(self, s):
        '''
        检查以s点为中心的方格是否有障碍物
        '''

        # 反二值化图片，所以判断是否有障碍，就对这个区域求和即可
        row_start = s.pos[0] * self.grid_size
        col_start = s.pos[1] * self.grid_size
        area = self.map[row_start: row_start +
                        self.grid_size, col_start: col_start + self.grid_size]
        if np.sum(area):
            return True

        return False

    def CollisionFree(self, s1, s2):
        '''
        检查以s1->s2是否可行，必要时增加s1,s2之间的栅格检查
        '''

        half_grid_size = int(self.grid_size / 2)
        row_start = int((s1.pos[0] + s2.pos[0]) * self.grid_size / 2)
        col_start = int((s1.pos[1] + s2.pos[1]) * self.grid_size / 2)
        area = self.map[row_start: row_start +
                        self.grid_size, col_start: col_start + self.grid_size]

        if np.sum(area):
            return False

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
        节点 s1 和 s2 估计距离
        '''
        if self.CollisionFree(s1, s2):
            return self.DiagonalDistance(s1.pos, s2.pos)
        else:
            return float('inf')

    def HScore(self, s):
        '''
        节点 s 到起始点的估计距离
        '''
        return self.DiagonalDistance(self.qstart.pos, s.pos)

    def CalculateKey(self, s):
        '''
        计算节点 Key值
        '''
        key2 = min(s.g, s.rhs)
        return [key2 + self.HScore(s), key2]

    def TopKey(self):
        '''
        返回有限队列 topkey
        '''
        return min([X.k for X in self.U])

    def Successors(self, s):
        '''
        8 邻接节点：与 s 可以联通（s和succ均无障碍物）
        '''
        succ_list = []
        for direction in self.directions:
            row = s.pos[0] + direction[0]
            if row < 0 or row >= self.rows:
                continue

            col = s.pos[1] + direction[1]
            if col < 0 or col >= self.cols:
                continue

            neighbor = self.S[row][col]

            if neighbor.is_obs == False:
                succ_list.append(neighbor)

        return succ_list

    def Predessors(self, s):
        return self.Successors(s)

    def UInsert(self, u, key):
        '''
        将节点u 插入到优先队列中
        '''
        u.k = key
        self.U.add(u)
        return

    def UpdateVertex(self, u):
        if u != self.qgoal:
            new_rhs = float('inf')
            for succ in self.Successors(u):
                tmp_rhs = self.Cost(u, succ) + succ.g
                if tmp_rhs < new_rhs:
                    new_rhs = tmp_rhs
                    u.p = succ
            u.rhs = new_rhs

        if u in self.U:
            self.U.remove(u)

        if u.g != u.rhs:
            self.UInsert(u, self.CalculateKey(u))

        return

    def ComputeShortestPath(self):
        '''
        计算最短路径
        '''

        while len(self.U) != 0 and ((self.TopKey() < self.CalculateKey(self.qstart)) or
                                    (self.qstart.rhs != self.qstart.g)):

            k_old = self.TopKey()
            u = min(self.U, key=lambda x: x.k)
            self.U.remove(u)

            k_u = self.CalculateKey(u)
            if k_old < k_u:
                self.UInsert(u, k_u)
            elif u.g > u.rhs:
                u.g = u.rhs
                for succ in self.Successors(u):
                    self.UpdateVertex(succ)
            else:
                u.g = float('inf')
                self.UpdateVertex(u)
                for pred in self.Predessors(u):
                    self.UpdateVertex(pred)

        return self.qstart.rhs != float('inf')

    def DrawGrid(self, color=(0, 0, 0)):
        '''
        画出网格
        '''

        for row in range(self.rows + 1):
            pt1 = (0, int(row * self.grid_size))
            pt2 = (int(self.cols * self.grid_size), int(row * self.grid_size))
            cv2.line(self.src_map, pt1, pt2, color)
            cv2.imshow('D_STAR_LITE', self.src_map)
            cv2.waitKey(50)

        for col in range(self.cols + 1):
            pt1 = (int(col * self.grid_size), 0)
            pt2 = (int(col * self.grid_size), int(self.rows * self.grid_size))
            cv2.line(self.src_map, pt1, pt2, color)
            cv2.imshow('D_STAR_LITE', self.src_map)
            cv2.waitKey(50)

        cv2.imshow('D_STAR_LITE', self.src_map)
        cv2.waitKey(50)
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

            cv2.imshow('D_STAR_LITE', self.src_map)
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
            pt2 = (int(mid_node.p.pos[1] * self.grid_size + self.grid_size / 2),
                   int(mid_node.p.pos[0] * self.grid_size + self.grid_size / 2))

            cv2.line(self.src_map, pt1, pt2, color, 2)
            cv2.imshow('D_STAR_LITE', self.src_map)
            cv2.waitKey(50)

            mid_node = mid_node.p

        return

    def Planning(self):

        self.U = set()
        self.qgoal.rhs = 0
        self.UInsert(self.qgoal, [self.HScore(self.qgoal), 0])

        num = input("input obstacle numbers: ")
        try:
            input_num = eval(num)
            if type(input_num) == int:
                obstacle_set = self.AddObstacle(input_num)
            else:
                return
        except:
            return

        if self.ComputeShortestPath():
            print('Found')
            self.DrawPath()
        else:
            print('Not Found')
            return

        while True:
            num = input("input obstacle numbers: ")
            try:
                input_num = eval(num)
                if type(input_num) == int:
                    obstacle_set = self.AddObstacle(input_num)
                    for obs in obstacle_set:
                        successors = self.Successors(obs)
                        for succ in successors:
                            self.UpdateVertex(succ)
                        for succ in successors:
                            if succ.p is obs:
                                succ.p = None

                    if self.ComputeShortestPath():
                        print('Found')
                        self.DrawPath()
                    else:
                        print('Not Found')
                        return
                else:
                    return
            except:
                return


if __name__ == "__main__":

    map_path = 'map/map500-500.png'
    qstart = [10, 10]
    qgoal = [490, 490]
    max_steps = 1000
    grid_size = 20

    d_star_lite = D_STAR_LITE(map_path, qstart, qgoal, grid_size)
    input('press any key to start planning:')
    d_star_lite.Planning()
    input('press any key to quit:')
