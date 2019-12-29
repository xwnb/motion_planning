import cv2
import numpy as np
import copy
import random
import math


class Node(object):
    def __init__(self, pos):
        self.pos = pos          # position: (x, y)
        self.g = float('inf')   # g_score: g(n)
        self.h = float('inf')   # h_score: h(n)
        self.f = float('inf')   # f_score: f(n)
        self.p = None           # parent node
        self.is_obs = False


class BFS(object):
    def __init__(self, map_path, qstart, qgoal, grid_size, max_steps):

        self.max_steps = max_steps
        self.directions = [[-1, -1], [-1, 0], [-1, 1],
                           [0, -1], [0, 1],
                           [1, -1], [1, 0], [1, 1]]

        self.MapGridding(map_path, qstart, qgoal, grid_size)
        return

    def MapGridding(self, map_path, qstart, qgoal, grid_size, color=(0, 0, 0)):
        '''
        grid map
        '''

        self.src_map = cv2.imread(map_path)
        self.map = cv2.cvtColor(self.src_map, cv2.COLOR_BGR2GRAY)
        _, self.map = cv2.threshold(
            self.map, 0, 255, cv2.THRESH_BINARY_INV)

        self.map_shape = np.shape(self.map)
        self.grid_size = grid_size
        self.rows = int(self.map_shape[0] / grid_size)
        self.cols = int(self.map_shape[1] / grid_size)

        # map gridding
        self.S = []
        for row in range(self.rows):
            items_rows = []
            for col in range(self.cols):
                items_rows.append(Node([row, col]))
            self.S.append(items_rows)

        # if the grid can not reach, make a flag
        for row in range(self.rows):
            for col in range(self.cols):
                if self.IsObstacle(self.S[row][col]):
                    self.S[row][col].is_obs = True

        self.qstart = self.S[int(qstart[0] / grid_size)][int(qstart[1] / grid_size)]
        self.qgoal = self.S[int(qgoal[0] / grid_size)][int(qgoal[1] / grid_size)]

        for row in range(self.rows + 1):
            pt1 = (0, int(row * self.grid_size))
            pt2 = (int(self.cols * self.grid_size), int(row * self.grid_size))
            cv2.line(self.src_map, pt1, pt2, color)
            cv2.imshow('BFS', self.src_map)
            cv2.waitKey(2)

        for col in range(self.cols + 1):
            pt1 = (int(col * self.grid_size), 0)
            pt2 = (int(col * self.grid_size), int(self.rows * self.grid_size))
            cv2.line(self.src_map, pt1, pt2, color)
            cv2.imshow('BFS', self.src_map)
            cv2.waitKey(2)

        cv2.imshow('BFS', self.src_map)
        cv2.waitKey(50)
        return

    def ManhattanDistance(self, p1, p2):
        return self.grid_size * (abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]))

    def ChebyshevDistance(self, p1, p2):
        return self.grid_size * max(abs(p1[0] - p2[0]), abs(p1[1] - p2[1]))

    def EuclideanDistance(self, p1, p2):
        return self.grid_size * int(math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2))

    def DiagonalDistance(self, p1, p2):
        dx = abs(p1[0] - p2[0])
        dy = abs(p1[1] - p2[1])
        return self.grid_size * (dx + dy) + int((math.sqrt(2) - 2) * self.grid_size) * min(dx, dy)

    def HScore(self, s):
        '''
        the heuristic function that estimates the cost of path from s to goal node
        '''
        # 启发函数修改为欧式距离 0 < Euclidean < 实际距离
        # H = 实际距离; 最佳路径
        # H = 0， G 有效， 退化为BFS算法
        # H >> G, H 有效， 退化为BFS,  best first search
        return self.EuclideanDistance(self.qgoal.pos, s.pos)

    def GScore(self, current, s):
        '''
        the cost of the path from the start node to s node
        '''
        # return current.g + self.DistanceCost(s, current)
        return 0

    def FScore(self, s):
        '''
        cost function of the path
        '''
        return s.g + s.h

    def DistanceCost(self, s1, s2):
        '''
        distance cost from s1 to s2 node
        '''
        return self.DiagonalDistance(s1.pos, s2.pos)

    def Neighbors(self, s):
        '''
        the 8 neighbors of s
        '''
        neighbors = []
        for direction in self.directions:
            row = s.pos[0] + direction[0]
            if row < 0 or row >= self.rows:
                continue

            col = s.pos[1] + direction[1]
            if col < 0 or col >= self.cols:
                continue

            neighbors.append(self.S[row][col])

        return neighbors

    def IsObstacle(self, s):
        '''
        check the grid(node) s is obstacle or not
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
        check the path from s1 to s2 is collision free or not
        '''
        row_start = int((s1.pos[0] + s2.pos[0]) * self.grid_size / 2)
        col_start = int((s1.pos[1] + s2.pos[1]) * self.grid_size / 2)
        area = self.map[row_start: row_start +
                        self.grid_size, col_start: col_start + self.grid_size]

        if np.sum(area):
            return False

        return s1.is_obs == False and s2.is_obs == False

    def DrawGrid(self, grid, color=(0, 0, 0)):
        '''
        draw the grid(node)
        '''

        # 原图和二值化后的图片尺寸反的
        # 这里将边界缩小1个像素，防止出现障碍扩充到隔壁格子
        pt1 = (int(grid[1] * self.grid_size + 1),
               int(grid[0] * self.grid_size + 1))
        pt2 = (int((grid[1] + 1) * self.grid_size - 1),
               int((grid[0] + 1) * self.grid_size - 1))

        cv2.rectangle(self.src_map, pt1, pt2, color, cv2.FILLED)

        cv2.imshow('BFS', self.src_map)
        cv2.waitKey(50)
        return

    def DrawObstacle(self, obs_set, color):
        '''
        draw the obstacle grid(node)
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
            cv2.imshow('BFS', self.src_map)
            cv2.waitKey(50)
        return

    def AddObstacle(self, num):
        '''
        generate num obstacles
        '''
        obs_pos = []
        obstacles = []
        color = (random.randint(0, 255),
                 random.randint(0, 255),
                 random.randint(0, 255))

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

    def FindPath(self, color=None):
        '''
        reconstruct the path from start to goal node
        '''
        if color is None:
            color = (random.randint(0, 255),
                     random.randint(0, 255),
                     random.randint(0, 255))

        path = [self.qgoal]
        mid_node = self.qgoal
        while mid_node != self.qstart:

            # 画出网格连线
            pt1 = (int(mid_node.pos[1] * self.grid_size + self.grid_size / 2),
                   int(mid_node.pos[0] * self.grid_size + self.grid_size / 2))
            pt2 = (int(mid_node.p.pos[1] * self.grid_size + self.grid_size / 2),
                   int(mid_node.p.pos[0] * self.grid_size + self.grid_size / 2))

            cv2.line(self.src_map, pt1, pt2, color, 2)
            cv2.imshow('BFS', self.src_map)
            cv2.waitKey(50)

            mid_node = mid_node.p
            path.append(mid_node)

        path.reverse()
        return path

    def Planning(self):

        color_close = (random.randint(0, 255),
                       random.randint(0, 255),
                       random.randint(0, 255))

        color_open = (random.randint(0, 255),
                      random.randint(0, 255),
                      random.randint(0, 255))

        # input numbers of obstacle grid if necessary
        num = input("input obstacle numbers: ")
        try:
            input_num = eval(num)
            if type(input_num) == int:
                obstacle_set = self.AddObstacle(input_num)
            else:
                return
        except:
            return

        # init qstart node
        self.qstart.g = 0
        self.qstart.h = self.HScore(self.qstart)
        self.qstart.f = self.FScore(self.qstart)

        # init open_table and close_table
        self.open_table = set()
        self.close_table = set()

        # add start node into open_table
        self.open_table.add(self.qstart)

        k = -1
        while k < self.max_steps or len(self.open_table) != 0:

            # get the node in open_table having the lowest f_score value
            best_node = min(self.open_table, key=lambda node: node.f)

            # reach the goal node
            if best_node.pos == self.qgoal.pos:
                print("Found")
                self.FindPath()
                return

            # remove the best node in open table and add it into close table
            self.open_table.remove(best_node)
            self.close_table.add(best_node)
            self.DrawGrid(best_node.pos, color_close)

            # traversal the neighbors of best node
            for neighbor in self.Neighbors(best_node):

                # if neighbor is in close table or is collision, skip
                if neighbor in self.close_table or self.CollisionFree(best_node, neighbor) == False:
                    continue

                # calculate the tentative_g which is the distance from start to the neighbor through best_node
                tentative_g = self.GScore(best_node, neighbor)

                # check the neighbor is in open table or not
                if neighbor not in self.open_table:
                    # if neighbor not in open table,
                    # then calculate the g, h, f values and add it into open table
                    neighbor.g = tentative_g
                    neighbor.h = self.HScore(neighbor)
                    neighbor.f = self.FScore(neighbor)
                    neighbor.p = best_node
                    self.open_table.add(neighbor)
                    self.DrawGrid(neighbor.pos, color_open)
                else:
                    # if neighbor in open table,
                    # then compare the tentative_g and neighbor.g
                    if tentative_g < neighbor.g:
                        # the path from best_node to neighbor is better than previous
                        neighbor.g = tentative_g
                        neighbor.f = self.FScore(neighbor)
                        neighbor.p = best_node

        print("Not Found")


if __name__ == "__main__":

    map_path = 'map/area6.png'
    qstart = [10, 10]
    qgoal = [490, 490]
    max_steps = 1000
    grid_size = 20

    bfs = BFS(map_path, qstart, qgoal, grid_size, max_steps)
    input('press any key to start planning:')
    bfs.Planning()
    input('press any key to quit:')
