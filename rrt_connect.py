import cv2
import numpy as np
import copy
import random
import math


class Node(object):
    def __init__(self, pos=[0, 0]):
        self.pos = pos
        self.parent = None


class RRT_CONNECT(object):
    def __init__(self, map_path, qstart, qgoal, grid_size, step_size,
                 max_steps=1000, goal_prob=0.0):
        '''
        initialize RRT_CONNECT
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
        cv2.imshow('RRT_CONNECT', self.src_map)
        cv2.waitKey(50)

    def GenerateRandomNode(self, row_range, col_range, qgoal, goal_prob):
        '''
        qrand = a randomly chosen free configuration
        '''
        if random.random() > goal_prob:
            row = random.randrange(row_range[0], row_range[1])
            col = random.randrange(col_range[0], col_range[1])
            return Node([row, col])

        return qgoal

    def Distance(self, p1, p2):
        '''
        calculate the distance between p1 and p2
        '''
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def FindNearestNode(self, q, vertices):
        '''
        qnear = closest neighbor of q in T
        '''

        min_distance = float('inf')
        for node in vertices:
            distance = self.Distance(node.pos, q.pos)
            if distance < min_distance:
                min_distance = distance
                qnear = node

        return min_distance, qnear

    def ExtendTree(self, qnear, qrand, step_size):
        '''
        progress qnear by step_size along the straight line in Q(map) between qnear and qrand
        if qrand is close to qnear, ignore qrand and return None
        '''

        vec = [qrand.pos[0] - qnear.pos[0], qrand.pos[1] - qnear.pos[1]]
        norm_vec = math.sqrt(vec[0] ** 2 + vec[1] ** 2)
        if norm_vec < 0.0001:
            return None

        vec = [vec[0] / norm_vec, vec[1] / norm_vec]

        qnew_pos = [int(qnear.pos[0] + step_size * vec[0]),
                    int(qnear.pos[1] + step_size * vec[1])]

        if qnew_pos[0] < 0 or qnew_pos[0] >= self.map_shape[0]:
            return None

        if qnew_pos[1] < 0 or qnew_pos[1] >= self.map_shape[1]:
            return None

        return Node(qnew_pos)

    def IsObstacle(self, p, grid_size):
        '''
        check the grid of pos p in map is obstacle or not

        '''
        half_grid_size = int(grid_size / 2)
        area = self.map[p[1] - half_grid_size: p[1] + half_grid_size,
                        p[0] - half_grid_size: p[0] + half_grid_size]
        if np.sum(area):
            return True

        return False

    def CollsionFree(self, qnear, qnew, grid_size):
        '''
        check qnear to qnew is collsion-free 
        '''
        rows = qnew.pos[0] - qnear.pos[0]
        cols = qnew.pos[1] - qnear.pos[1]
        length = max(abs(rows), abs(cols))

        for i in range(0, length, int(grid_size / 2)):
            row = int(qnear.pos[0] + i * rows / length)
            col = int(qnear.pos[1] + i * cols / length)
            if self.IsObstacle([row, col], grid_size):
                return False

        # the qnew may not be contained above, confirm qnew check
        return not self.IsObstacle(qnew.pos, grid_size)

    def AddVertices(self, qnew, vertices):
        '''
        add qnew to vertices
        '''
        vertices.append(qnew)
        return

    def AddEdges(self, qnear, qnew, edges=None):
        '''
        here we use a pointer to point to qnear as qnew's parent
        '''
        if qnew:
            qnew.parent = qnear
        return

    def DrawEdges(self, src_map, qnear, qnew, color=(0, 0, 255), thickness=1):
        '''
        draw the new edge
        '''
        cv2.line(src_map, tuple(qnear.pos), tuple(qnew.pos),
                 color, thickness)
        cv2.imshow('RRT_CONNECT', src_map)
        cv2.waitKey(50)

    def IsArrival(self, qnew, qgoal, step_size):
        '''
        if the distance between qnew and qgoal less than threshold, 
        and the path of qnew to qgoal is collsion-free, 
        the next vertices is qgoal, obviously
        '''
        if self.Distance(qnew.pos, qgoal.pos) > step_size:
            return False

        if self.CollsionFree(qnew, qgoal, step_size):
            return True

        return False

    def FindPath(self, vertices):
        '''
        find the complete path with node in vertices propagate with parent pointer
        '''
        path = []
        node = vertices[-1]
        path.append(node)
        while node.parent:
            path.append(node.parent)
            node = node.parent

        path.reverse()
        return path

    def DrawPath(self, src_map, path, color=None, thickness=3):
        '''
        draw the complete path
        '''
        if color is None:
            color = (random.randint(0, 255),
                     random.randint(0, 255),
                     random.randint(0, 255))

        node = path[0]
        for next_node in path:
            cv2.line(src_map, tuple(node.pos), tuple(next_node.pos),
                     color, thickness)
            cv2.imshow('RRT_CONNECT', src_map)
            cv2.waitKey(50)
            node = next_node
        return

    def SmoothPath(self, path, grid_size):
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
            if self.CollsionFree(pre_node, next_node, grid_size) == False:
                smooth_path.append(cur_node)
                pre_node = cur_node
            cur_node = next_node

        # the last node
        smooth_path.append(cur_node)
        return smooth_path

    def MergeTree(self, vertices_t1, vertices_t2):
        '''
        merget tree, vetices_t2 is the tree from pgaol, to reverse the parent pointer
        '''

        path_t1 = self.FindPath(vertices_t1)
        self.DrawPath(self.src_map, path_t1)

        path_t2 = self.FindPath(vertices_t2)
        self.DrawPath(self.src_map, path_t2)

        if path_t1[0] is self.qstart:
            path = path_t1 + list(reversed(path_t2))
        elif path_t1[0] is self.qgoal:
            path = path_t2 + list(reversed(path_t1))

        return path

    def Planning(self):
        '''
        RRT_CONNECT planning
        '''

        vertices_t1 = []
        self.AddVertices(self.qstart, vertices_t1)
        self.AddEdges(None, self.qstart)

        vertices_t2 = []
        self.AddVertices(self.qgoal, vertices_t2)
        self.AddEdges(None, self.qgoal)

        k = 0
        new_qgoal = self.qgoal
        while k <= self.max_steps:
            k += 1

            '''
            第一颗树， 从起点开始， 随机节点为目标点
            '''
            qrand_t1 = self.GenerateRandomNode(
                [0, self.map_shape[0]], [0, self.map_shape[1]],
                new_qgoal, self.goal_prob)

            _, qnear_t1 = self.FindNearestNode(qrand_t1, vertices_t1)
            qnew_t1 = self.ExtendTree(qnear_t1, qrand_t1, self.step_size)

            if qnew_t1 and self.CollsionFree(qnear_t1, qnew_t1, self.grid_size):

                self.AddVertices(qnew_t1, vertices_t1)
                self.AddEdges(qnear_t1, qnew_t1)

                self.DrawEdges(self.src_map, qnear_t1, qnew_t1)

                '''
                第二颗树，从终点开始，以第一棵树的新节点为目标点
                '''
                k += 1
                _, qnear_t2 = self.FindNearestNode(qnew_t1, vertices_t2)
                qnew_t2 = self.ExtendTree(qnear_t2, qnew_t1, self.step_size)

                if qnew_t2 and self.CollsionFree(qnear_t2, qnew_t2, self.grid_size):

                    self.AddVertices(qnew_t2, vertices_t2)
                    self.AddEdges(qnear_t2, qnew_t2)

                    self.DrawEdges(self.src_map, qnear_t2, qnew_t2)

                    '''
                    两棵树的新节点再产生新节点， 添加到终点树中，一直到不符合要求
                    '''
                    while k <= self.max_steps:
                        k += 1
                        qnew_mid = self.ExtendTree(
                            qnew_t2, qnew_t1, self.step_size)

                        if qnew_mid and self.CollsionFree(qnew_t2, qnew_mid, self.grid_size):

                            self.AddVertices(qnew_mid, vertices_t2)
                            self.AddEdges(qnew_t2, qnew_mid)

                            self.DrawEdges(self.src_map, qnew_t2, qnew_mid)

                            qnew_t2 = qnew_mid
                        else:
                            break

                        if self.IsArrival(qnew_t2, qnew_t1, self.step_size):
                            break

                if self.IsArrival(qnew_t2, qnew_t1, self.step_size):
                    print("Found")
                    '''
                    最后一个节点连接
                    '''
                    copy_qnew_t1 = copy.deepcopy(qnew_t1)
                    self.AddVertices(copy_qnew_t1, vertices_t2)
                    self.AddEdges(qnew_t2, copy_qnew_t1)

                    path = self.MergeTree(vertices_t1, vertices_t2)
                    self.DrawPath(self.src_map, path)
                    smooth_path = self.SmoothPath(path, self.grid_size)
                    self.DrawPath(self.src_map, smooth_path)

                    return True

            if (len(vertices_t2) < len(vertices_t1)):
                vertices_t2, vertices_t1 = vertices_t1, vertices_t2
                # edges_t2, edges_t1 = edges_t1, edges_t2

                new_qgoal = vertices_t2[-1]

        print("NotFound")
        return False


if __name__ == "__main__":

    map_path = 'map/area6.png'
    qstart = [20, 20]
    qgoal = [480, 480]
    max_steps = 10000
    step_size = 20
    goal_prob = 0.01
    grid_size = 10

    rrt_connect = RRT_CONNECT(map_path, qstart, qgoal, grid_size,
                              step_size, max_steps, goal_prob)
    input('press any key to start planning:')
    rrt_connect.Planning()
    input('press any key to quit:')
