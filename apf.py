import cv2
import numpy as np
import random
import math
import copy


class Node(object):
    def __init__(self, pos):
        self.pos = pos
        self.obs = False
        self.p = None


class APF(object):
    def __init__(self, img, img_binary, pstart, pgoal, pdelta, pstep, maxtimes, xi, eta, rho0):
        self.img = img
        self.img_binary = img_binary

        self.delta = pdelta
        self.step = pstep
        self.world = img
        self.rows = int(np.shape(img)[0] / pdelta)
        self.cols = int(np.shape(img)[1] / pdelta)

        self.maxtimes = maxtimes

        self.pstart = [int(pstart[0] / pdelta), int(pstart[1] / pdelta)]
        self.pgoal = [int(pgoal[0] / pdelta), int(pgoal[1] / pdelta)]

        # 地图栅格
        self.S = []
        # 初始化地图所有路径点
        for row in range(self.rows):
            items_rows = []
            for col in range(self.cols):
                items_rows.append(Node([row, col]))

            # 按行列对齐摆放节点S
            self.S.append(items_rows)

        # 障碍物节点
        self.Obs = []
        for row in range(self.rows):
            for col in range(self.cols):
                if self.IsObstacle(self.S[row][col]):
                    self.Obs.append(self.S[row][col])

                # 始末节点为表示为
        self.start = self.S[self.pstart[0]][self.pstart[1]]
        self.goal = self.S[self.pgoal[0]][self.pgoal[1]]

        self.xi = xi
        self.rho0 = rho0
        self.eta = eta

        self.DrawGrid()
        return

    def IsObstacle(self, q):
        '''
        检查以s点为中心的方格是否有障碍物
        '''

        # 反二值化图片，所以判断是否有障碍，就对这个区域求和即可
        row_start = q.pos[0] * self.delta
        col_start = q.pos[1] * self.delta
        area = self.img_binary[row_start: row_start + self.delta,
                               col_start: col_start + self.delta]
        if np.sum(area):
            return True

        return False

    def DrawGrid(self, color=(0, 0, 0)):
        '''
        画出网格
        '''

        for row in range(self.rows + 1):
            pt1 = (0, int(row * self.delta))
            pt2 = (int(self.cols * self.delta), int(row * self.delta))
            cv2.line(self.img, pt1, pt2, color)
            cv2.imshow('APF', self.img)
            cv2.waitKey(5)

        for col in range(self.cols + 1):
            pt1 = (int(col * self.delta), 0)
            pt2 = (int(col * self.delta), int(self.rows * self.delta))
            cv2.line(self.img, pt1, pt2, color)
            cv2.imshow('APF', self.img)
            cv2.waitKey(5)

        cv2.imshow('APF', self.img)
        cv2.waitKey(5)
        return

    def DrawSingleGrid(self, grid_set, color=(0, 0, 0)):
        '''
        画出过程方格
        '''
        for obs in grid_set:
            # 原图和二值化后的图片尺寸反的
            # 这里将边界缩小1个像素，防止出现障碍扩充到隔壁格子
            pt1 = (int(obs.pos[1] * self.delta + 1),
                   int(obs.pos[0] * self.delta + 1))
            pt2 = (int((obs.pos[1] + 1) * self.delta - 1),
                   int((obs.pos[0] + 1) * self.delta - 1))

            cv2.rectangle(self.img, pt1, pt2, color, cv2.FILLED)

            cv2.imshow('APF', self.img)
            cv2.waitKey(5)
        return

    def DrawObstacle(self, obs_set, color):
        '''
        画出添加障碍的节点
        '''
        for obs in obs_set:
            # 原图和二值化后的图片尺寸反的
            # 这里将边界缩小1个像素，防止出现障碍扩充到隔壁格子
            pt1 = (int(obs[1] * self.delta + 1),
                   int(obs[0] * self.delta + 1))
            pt2 = (int((obs[1] + 1) * self.delta - 1),
                   int((obs[0] + 1) * self.delta - 1))

            cv2.rectangle(self.img_binary, pt1, pt2, 255, cv2.FILLED)
            cv2.rectangle(self.img, pt1, pt2, color, cv2.FILLED)

            cv2.imshow('APF', self.img)
            cv2.waitKey(5)
        return

    def AddObstacle(self, num):
        '''
        产生 num 个随机障碍
        '''
        obs_pos = []
        obstacles = []
        color = (random.randint(0, 255),
                 random.randint(0, 255),
                 random.randint(0, 255))

        for i in range(num):

            # 排除起始点和终止点
            pos = [random.randint(1, self.rows - 2),
                   random.randint(1, self.cols - 2)]
            obs_pos.append(pos)

            obs = self.S[pos[0]][pos[1]]
            obstacles.append(obs)

        self.DrawObstacle(obs_pos, color)

        return obstacles

    def Distance(self, q1, q2):
        return math.sqrt((q1[0] - q2[0]) ** 2 + (q1[1] - q2[1]) ** 2)

    def ComputeAttract(self, q, qgoal, xi):
        '''
        计算引力: xi-引力因子， rho-q到qgoal的距离
        引力函数： U_att = 1/2 * xi * rho(q, qgoal)^2
        引力势场： F_att = - xi * rho(q, qgoal)
        '''
        fatt1 = xi * (qgoal.pos[0] - q.pos[0])
        fatt2 = xi * (qgoal.pos[1] - q.pos[1])
        return [fatt1, fatt2]

    def ComputeRepulsion(self, q, qobs, eta, rho0):
        '''
        计算斥力： eta-斥力因子， rho-q到qobs的距离， rho0-障碍物的影响半径
        斥力函数： U_rep = 1/2 * eta * (1/rho - 1/rho0)^2 --- rho <= rho0
                        = 0 --- rho > rho0
        '''
        rho_square = ((q.pos[0] - qobs.pos[0]) ** 2 +
                      (q.pos[1] - qobs.pos[1]) ** 2)
        rho = math.sqrt(rho_square)

        if rho > rho0:
            return [0, 0]
        else:
            print('check obs in: ', qobs.pos)
            k = eta * (1 / rho - 1 / rho0) / rho_square
            frep1 = k * (q.pos[0] - qobs.pos[0])
            frep2 = k * (q.pos[1] - qobs.pos[1])
            return [frep1, frep2]

    def Planning(self):

        num = input("input obstacle numbers: ")
        try:
            input_num = eval(num)
            if type(input_num) == int:
                obstacle_set = self.AddObstacle(input_num)
                for obs in obstacle_set:
                    obs.obs = True
                    self.Obs.append(obs)
        except:
            return

        '''
        obs_set = [[3, 2], [3, 3], [5, 7], [5, 6], [6, 6],
                   [2, 4], [3, 8], [4, 7], [8, 9]]  # 障碍物坐标

        for obs_pos in obs_set:
            self.DrawObstacle([[50 - obs_pos[1] * 5,
                                obs_pos[0] * 5]], (0, 0, 0))
            obs = self.S[50 - obs_pos[1] * 5][obs_pos[0] * 5]
            obs.obs = True
            self.Obs.append(obs)
        '''

        mid_q = self.start
        tmp_q = mid_q
        k = -1
        while k < self.maxtimes:
            k += 1
            fatt = self.ComputeAttract(mid_q, self.goal, self.xi)

            sum_freq = [0, 0]
            for qobs in self.Obs:
                freq = self.ComputeRepulsion(mid_q, qobs, self.eta, self.rho0)
                sum_freq[0] += freq[0]
                sum_freq[1] += freq[1]

            sum_fpot = [0, 0]
            inc_pos = [0, 0]
            sum_fpot[0] = fatt[0] + sum_freq[0]
            sum_fpot[1] = fatt[1] + sum_freq[1]
            base_inc = max(abs(sum_fpot[0]), abs(sum_fpot[1]))
            inc_pos[0] = int(sum_fpot[0] / base_inc * self.step)
            inc_pos[1] = int(sum_fpot[1] / base_inc * self.step)
            row = mid_q.pos[0] + inc_pos[0]
            col = mid_q.pos[1] + inc_pos[1]
            if row < 0:
                row = 0
            elif row >= self.rows:
                row = self.rows - 1

            if col < 0:
                col = 0
            elif col >= self.cols:
                col = self.cols - 1

            tmp_q = self.S[row][col]
            self.DrawSingleGrid([tmp_q], (255, 0, 0))
            if tmp_q == self.goal:
                print('Found')
                return
            mid_q = tmp_q

        print('Not Found')
        return


if __name__ == "__main__":
    img = cv2.imread('./map/area6.png')
    cv2.imshow('APF', img)
    cv2.waitKey(5)
    img_copy = copy.deepcopy(img)
    img_gray = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)
    ret, img_binary = cv2.threshold(img_gray, 0, 255, cv2.THRESH_BINARY_INV)

    map_size = np.shape(img_binary)

    # 网格大小
    sdelta = 10

    # 起点
    sstart = [int(sdelta / 2), int(sdelta / 2)]

    # 终点
    sgoal = [int(map_size[0] - sdelta / 2), int(map_size[1] - sdelta / 2)]

    # 最大迭代次数
    maxtimes = 1000

    # 引力系数
    xi = 1

    # 斥力系数（取始末点距离的一半的平方）
    eta = 200

    # 斥力影响距离
    rho0 = 8

    # 步长
    step = 1

    a_star = APF(img_copy, img_binary, sstart, sgoal,
                 sdelta, step, maxtimes, xi, eta, rho0)
    a_star.Planning()
    cv2.waitKey(0)
