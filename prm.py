import cv2
import numpy as np
import random
import math
import copy


class Node(object):
    def __init__(self, pos):
        self.pos = pos
        self.neighbors = []
        self.g = float('inf')
        self.h = float('inf')
        self.f = float('inf')
        self.p = None


class PRM(object):
    def __init__(self, map_path, qstart, qgoal, grid_size, max_steps,
                 num_nodes, max_distance, max_num_neighbors):

        self.max_steps = max_steps
        self.num_nodes = num_nodes
        self.max_distance = max_distance
        self.max_num_neighbors = max_num_neighbors

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
        self.rows = np.shape(self.map)[0]
        self.cols = np.shape(self.map)[1]

        # 始末节点为表示为
        self.qstart = Node(qstart)
        self.qgoal = Node(qgoal)

        cv2.imshow('PRM', self.src_map)
        cv2.waitKey(50)
        return

    def DrawGrid(self, grid, grid_size, color=(0, 0, 0)):
        '''
        画出方格
        '''
        # 原图和二值化后的图片尺寸反的
        # 这里将边界缩小1个像素，防止出现障碍扩充到隔壁格子
        half_grid_size = int(grid_size / 2)
        pt1 = (int(grid[1] - half_grid_size + 1),
               int(grid[0] - half_grid_size + 1))
        pt2 = (int(grid[1] + half_grid_size - 1),
               int(grid[0] + half_grid_size - 1))

        cv2.rectangle(self.src_map, pt1, pt2, color, cv2.FILLED)

        cv2.imshow('PRM', self.src_map)
        cv2.waitKey(50)
        return

    def RandomConfigNodes(self, nums, row_range, col_range, grid_size, vertex):
        '''
        随机产生nums非障碍节点
        '''
        k = 0

        color_vertex = (random.randint(0, 255),
                        random.randint(0, 255),
                        random.randint(0, 255))
        while k < nums:
            # 边缘的点就不采集了
            pos = [random.randrange(row_range[0], row_range[1]),
                   random.randrange(col_range[0], col_range[1])]
            node = Node(pos)
            if self.IsObstacle(node.pos, grid_size) == False:
                self.DrawGrid(node.pos, self.grid_size, color_vertex)
                vertex.append(node)
                k += 1

        return vertex

    def Neighbors(self, s):
        '''
        s邻接节点：
        '''
        return s.neighbors

    def FindNeighbors(self, node, vertexs, max_num, max_dist):
        '''
        在节点空间vertexs中寻找node的在max_dist距离内的最多max_num个邻居
        '''
        k = 0
        neighbors = []
        dict_neighbors = []
        for v in vertexs:
            dist = self.DistanceCost(node, v)
            if dist < max_dist:
                dict_neighbors.append({'node': v, 'dist': dist})
                k += 1
        dict_neighbors.sort(key=lambda node: node['dist'])

        for neighbor in dict_neighbors:
            neighbors.append(neighbor['node'])

        num = min(max_num, k)
        return neighbors[:num]

    def ChebyshevDistance(self, p1, p2):
        return max(abs(p1[0] - p2[0]), abs(p1[1] - p2[1]))

    def EuclideanDistance(self, p1, p2):
        return int(math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2))

    def DiagonalDistance(self, p1, p2):
        dx = abs(p1[0] - p2[0])
        dy = abs(p1[1] - p2[1])
        return (dx + dy) + int((math.sqrt(2) - 2) * self.grid_size) * min(dx, dy)

    def HScore(self, s):
        '''
        节点 s 到目标点的估计距离
        '''
        # 启发函数修改为欧式距离 0 < Euclidean < 实际距离
        # H = 实际距离; 最佳路径
        # H = 0， G 有效， 退化为Dijkstra算法
        # H >> G, H 有效， 退化为BFS,  best first search
        return self.EuclideanDistance(self.qgoal.pos, s.pos)

    def GScore(self, s):
        '''
        节点s 距离起始点的预估最短距离（切比雪夫距离表征）
        '''
        # 起点的预估最短距离取对角线距离
        return self.EuclideanDistance(s.pos, self.qstart.pos)

    def FScore(self, s):
        '''
        节点s 估计距离
        '''
        return self.GScore(s) + self.HScore(s)

    def DistanceCost(self, s1, s2):
        '''
        节点 s1 和 s2 估计距离
        '''
        return self.EuclideanDistance(s1.pos, s2.pos)

    def IsObstacle(self, p, grid_size):
        '''
        检查以s点为中心的方格是否有障碍物
        '''
        half_grid_size = int(grid_size / 2)
        area = self.map[p[0] - half_grid_size: p[0] + half_grid_size,
                        p[1] - half_grid_size: p[1] + half_grid_size]
        if np.sum(area):
            return True

        return False

    def CollisionFree(self, s1, s2, grid_size):
        '''
        检查以s1->s2是否可行，中间增加一个过度格子
        '''
        rows = s1.pos[0] - s2.pos[0]
        cols = s1.pos[1] - s2.pos[1]
        length = max(abs(rows), abs(cols))

        for i in range(0, length, int(grid_size / 2)):
            row = int(s2.pos[0] + i * rows / length)
            col = int(s2.pos[1] + i * cols / length)
            if self.IsObstacle([row, col], grid_size):
                return False

        # the s1 may not be contained above, confirm s1 check
        return not self.IsObstacle(s1.pos, grid_size)

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
            cv2.imshow('PRM', self.src_map)
            cv2.waitKey(50)
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

    def DrawPath(self, color=None, thickness=2):
        '''
        draw the complete path
        '''
        if color is None:
            color = (random.randint(0, 255),
                     random.randint(0, 255),
                     random.randint(0, 255))

        mid_node = self.qgoal
        while mid_node != self.qstart:
            cv2.line(self.src_map, (mid_node.pos[1], mid_node.pos[0]),
                     (mid_node.p.pos[1], mid_node.p.pos[0]),
                     color, thickness)
            cv2.imshow('PRM', self.src_map)
            cv2.waitKey(50)
            mid_node = mid_node.p
        return

    def Planning(self):
        self.V = []

        self.V.append(self.qstart)
        self.V.append(self.qgoal)

        self.RandomConfigNodes(self.num_nodes, [0, self.map_shape[0]],
                               [0, self.map_shape[1]], self.grid_size, self.V)

        for node in self.V:
            neighbors = self.FindNeighbors(node, self.V, self.max_num_neighbors, self.max_distance)
            if len(neighbors) == 0:
                print("current node has no neighbor")
                return False

            node.neighbors = neighbors

        # PRM
        # 初始节点放入 OPEN 表中
        self.open_table = set()
        self.close_table = set()
        self.open_table.add(self.qstart)

        color_close = (random.randint(0, 255),
                       random.randint(0, 255),
                       random.randint(0, 255))

        color_open = (random.randint(0, 255),
                      random.randint(0, 255),
                      random.randint(0, 255))
        k = -1
        while k < self.max_steps:
            k += 1

            # OPEN 表为空，失败返回
            if len(self.open_table) == 0:
                print("NotFound")
                return False

            # 把最佳节点 qnode 从 OPEN 表中移到 CLOSED 表
            best_node = min(self.open_table, key=lambda node: node.f)
            self.open_table.remove(best_node)
            self.close_table.add(best_node)
            self.DrawGrid(best_node.pos, self.grid_size, color_close)

            # best_node[0]: 当前节点坐标， best_node[1]: 父节点坐标
            if best_node.pos == self.qgoal.pos:
                print("Found")
                self.DrawPath()
                return True

            # 对当前节点的周围八个节点检查： 是否可达或者在 CLOSE 表中
            for neighbor in self.Neighbors(best_node):

                # 如果邻接节点不可达或者在CLOSE表中，则忽略
                if neighbor in self.close_table or \
                        self.CollisionFree(best_node, neighbor, self.grid_size) == False:
                    continue

                # 检查邻接节点是否在 OPEN 表中
                # 如果邻接节点不在 OPEN 表中，则放入，并且父节点为当前节点
                if neighbor not in self.open_table:
                    neighbor.g = self.GScore(neighbor)
                    neighbor.h = self.HScore(neighbor)
                    neighbor.f = self.FScore(neighbor)
                    neighbor.p = best_node
                    self.open_table.add(neighbor)
                    self.DrawGrid(neighbor.pos, self.grid_size, color_open)
                    continue

                # 如果邻接节点在 OPEN 表中，则
                if neighbor in self.open_table:
                    # 如果存在在 OPEN 表中，检查经过当前节点 best_node['pos'] 到该邻接节点 neighbor 是否更加近(G值)
                    new_g = best_node.g + self.DistanceCost(best_node, neighbor)

                    # 如果更加近(G值更小)，该邻接节点的父节点为当前节点， 并重新计算 fscore 和 gscore
                    if new_g < neighbor.g:
                        neighbor.g = new_g
                        neighbor.p = best_node

        print("NotFound")
        return False


if __name__ == "__main__":
    map_path = 'map/area6.png'
    qstart = [20, 20]
    qgoal = [480, 480]
    max_steps = 10000
    grid_size = 10

    # 节点数
    num_nodes = 100

    # 邻域节点距离
    max_distance = 200

    # 邻域节点数目
    max_num_neighbors = 10

    prm = PRM(map_path, qstart, qgoal, grid_size, max_steps,
              num_nodes, max_distance, max_num_neighbors)
    input('press any key to start planning:')
    prm.Planning()
    input('press any key to quit:')
