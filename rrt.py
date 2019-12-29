import cv2
import numpy as np
import copy
import random
import math


class Node(object):
    def __init__(self, pos=[0, 0]):
        self.pos = pos
        self.parent = None


class RRT(object):
    def __init__(self, map_path, qstart, qgoal, grid_size, step_size,
                 max_steps=1000, goal_prob=0.0):
        '''
        initialize RRT
        '''
        self.vertices = []  # 树的节点
        self.edges = []  # 树的边
        self.path = []  # 路径
        self.qstart = Node(qstart)  # 起点
        self.qgoal = Node(qgoal)  # 终点
        self.step_size = step_size  # 步长
        self.max_steps = max_steps  # 最大迭代次数
        self.goal_prob = goal_prob  # 随机趋向终点概率
        self.grid_size = grid_size  # 网格边长

        self.MapPreProcess(map_path)  # 初始化地图

    def MapPreProcess(self, map_path):
        '''
        convert map image to binary image
        '''
        self.src_map = cv2.imread(map_path)
        self.map = cv2.cvtColor(self.src_map, cv2.COLOR_BGR2GRAY)
        _, self.map = cv2.threshold(
            self.map, 0, 255, cv2.THRESH_BINARY_INV)

        self.map_shape = np.shape(self.map)
        cv2.imshow('RRT', self.src_map)
        cv2.waitKey(50)

    def GenerateRandomNode(self):
        '''
        qrand = a randomly chosen free configuration
        '''
        if random.random() > goal_prob:
            row = random.randrange(0, self.map_shape[0])
            col = random.randrange(0, self.map_shape[1])
            return Node([row, col])

        return self.qgoal

    def Distance(self, p1, p2):
        '''
        calculate the distance between p1 and p2
        '''
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def FindNearestNode(self, q):
        '''
        qnear = closest neighbor of q in T
        '''

        min_distance = float('inf')
        for node in self.vertices:
            distance = self.Distance(node.pos, q.pos)
            if distance < min_distance:
                min_distance = distance
                qnear = node

        return min_distance, qnear

    def ExtendTree(self, qnear, qrand):
        '''
        progress qnear by step_size along the straight line in Q(map) between qnear and qrand
        if qrand is close to qnear, ignore qrand and return None
        '''

        vec = [qrand.pos[0] - qnear.pos[0], qrand.pos[1] - qnear.pos[1]]
        norm_vec = math.sqrt(vec[0] ** 2 + vec[1] ** 2)
        if norm_vec < 0.0001:
            return None

        vec = [vec[0] / norm_vec, vec[1] / norm_vec]

        qnew_pos = [int(qnear.pos[0] + self.step_size * vec[0]),
                    int(qnear.pos[1] + self.step_size * vec[1])]

        if qnew_pos[0] < 0 or qnew_pos[0] >= self.map_shape[0]:
            return None

        if qnew_pos[1] < 0 or qnew_pos[1] >= self.map_shape[1]:
            return None

        return Node(qnew_pos)

    def IsObstacle(self, p):
        '''
        check the grid of pos p in map is obstacle or not

        '''
        half_grid_size = int(grid_size / 2)
        area = self.map[p[1] - half_grid_size: p[1] + half_grid_size,
                        p[0] - half_grid_size: p[0] + half_grid_size]
        if np.sum(area):
            return True

        return False

    def CollsionFree(self, qnear, qnew):
        '''
        check qnear to qnew is collsion-free 
        '''
        rows = qnew.pos[0] - qnear.pos[0]
        cols = qnew.pos[1] - qnear.pos[1]
        length = max(abs(rows), abs(cols))

        for i in range(0, length, int(self.grid_size / 2)):
            row = int(qnear.pos[0] + i * rows / length)
            col = int(qnear.pos[1] + i * cols / length)
            if self.IsObstacle([row, col]):
                return False

        # the qnew may not be contained above, confirm qnew check
        return not self.IsObstacle(qnew.pos)

    def AddVertices(self, qnew):
        '''
        add qnew to vertices
        '''
        self.vertices.append(qnew)
        return

    def AddEdges(self, qnear, qnew):
        '''
        here we use a pointer to point to qnear as qnew's parent
        '''
        qnew.parent = qnear
        return

    def DrawEdges(self, qnear, qnew, color=(0, 0, 255), thickness=1):
        '''
        draw the new edge
        '''
        cv2.line(self.src_map, tuple(qnear.pos), tuple(qnew.pos),
                 color, thickness)
        cv2.imshow('RRT', self.src_map)
        cv2.waitKey(50)

    def IsArrival(self, qnew):
        '''
        if the distance between qnew and qgoal less than threshold, 
        and the path of qnew to qgoal is collsion-free, 
        the next vertices is qgoal, obviously
        '''
        if self.Distance(qnew.pos, self.qgoal.pos) > self.step_size:
            return False

        if self.CollsionFree(qnew, self.qgoal):
            return True

        return False

    def FindPath(self):
        '''
        find the complete path with node in vertices propagate with parent pointer
        '''
        node = self.vertices[-1]
        self.path.append(node)
        while node.parent:
            self.path.append(node.parent)
            node = node.parent

        self.path.reverse()
        return self.path

    def DrawPath(self, path, color=None, thickness=3):
        '''
        draw the complete path
        '''
        if color is None:
            color = (random.randint(0, 255),
                     random.randint(0, 255),
                     random.randint(0, 255))

        node = path[0]
        for next_node in path:
            cv2.line(self.src_map, tuple(node.pos), tuple(next_node.pos),
                     color, thickness)
            cv2.imshow('RRT', self.src_map)
            cv2.waitKey(50)
            node = next_node
        return

    def SmoothPath(self, path):
        '''
        smooth path
        '''
        smooth_path = [path[0]]
        pre_node = path[0]
        cur_node = []
        next_node = []
        # pre_node(0) ...... cur_node(0), next_node(0)
        # if pre_node -> next_node is collsion, then the new edge is pre_node -> cur_node.
        # the next iterator is:
        # pre_node(cur_node(0)) ...... cur_node(next_node(0)), next_node
        for next_node in path:
            if self.CollsionFree(pre_node, next_node) == False:
                smooth_path.append(cur_node)
                pre_node = cur_node
            cur_node = next_node

        # the last node
        smooth_path.append(cur_node)
        return smooth_path

    def Planning(self):
        '''
        rrt planning
        '''
        self.AddVertices(self.qstart)
        self.AddEdges(None, self.qstart)

        k = -1
        while k < self.max_steps:
            k += 1
            qrand = self.GenerateRandomNode()
            _, qnear = self.FindNearestNode(qrand)
            qnew = self.ExtendTree(qnear, qrand)
            if qnew and self.CollsionFree(qnear, qnew):
                self.AddVertices(qnew)
                self.AddEdges(qnear, qnew)

                self.DrawEdges(qnear, qnew)

                if self.IsArrival(qnew):
                    print("Found")
                    self.AddVertices(self.qgoal)
                    self.AddEdges(qnew, self.qgoal)

                    path = self.FindPath()
                    self.DrawPath(path)
                    smooth_path = self.SmoothPath(path)
                    self.DrawPath(smooth_path)

                    return True
        print('Not Found')
        return False


if __name__ == "__main__":

    map_path = 'map/area6.png'
    qstart = [20, 20]
    qgoal = [480, 480]
    max_steps = 1000
    step_size = 20
    goal_prob = 0.01
    grid_size = 10

    rrt = RRT(map_path, qstart, qgoal, grid_size,
              step_size, max_steps, goal_prob)
    input('press any key to start planning:')
    rrt.Planning()
    input('press any key to quit:')
