import cv2
import numpy as np
import copy
import random
import math


class Node(object):
    def __init__(self, pos=[0, 0]):
        self.pos = pos
        self.parent = None


class RRT_STAR(object):
    def __init__(self, map_path, qstart, qgoal, grid_size, step_size, neighbor_radius,
                 max_steps=1000, goal_prob=0.0):
        '''
        initialize RRT_STAR
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
        self.neighbor_radius = neighbor_radius  # 潜在父节点的判定半径

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
        cv2.imshow('RRT_STAR', self.src_map)
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
        if qnear and qnew:
            cv2.line(src_map, tuple(qnear.pos), tuple(qnew.pos),
                     color, thickness)
            cv2.imshow('RRT_STAR', src_map)
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

    def FindPathByParentPointer(self, qnode):
        path = []
        path.append(qnode)
        while qnode.parent:
            path.append(qnode.parent)
            qnode = qnode.parent

        path.reverse()
        return path

    def FindPath(self, vertices):
        '''
        find the complete path with node in vertices propagate with parent pointer
        '''
        return self.FindPathByParentPointer(vertices[-1])

    def DrawPath(self, src_map, path, color=None, thickness=2):
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
            cv2.imshow('RRT_STAR', src_map)
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

    def FindNeighbors(self, qnew, vertices, radius):
        '''
        find protential parent nodes
        '''
        neighbors = list(filter(lambda qnode: self.Distance(
            qnode.pos, qnew.pos) < radius, vertices))

        neighbors_free = []
        for parent in neighbors:
            if self.CollsionFree(parent, qnew, self.grid_size):
                neighbors_free.append(parent)

        return neighbors_free

    def CalculatePathLength(self, path):
        '''
        calculate the path length
        '''
        len_path = 0
        pre_qnode = path[0]
        for qnode in path[1:]:
            len_path += self.Distance(pre_qnode.pos, qnode.pos)
            pre_qnode = qnode

        return len_path

    def RewireNewParent(self, qnew, vertices, protential_parents):
        '''
        rewire the edges of the qnew and its protential_parents
        '''
        len_qnew_path = float('inf')
        qnew_parent = None
        for parent in protential_parents:
            parent_path = self.FindPathByParentPointer(parent)
            len_parent_path = self.CalculatePathLength(parent_path)
            tmp_len_path = len_parent_path + \
                self.Distance(qnew.pos, parent.pos)

            if tmp_len_path < len_qnew_path:
                len_qnew_path = tmp_len_path
                qnew_parent = parent

        self.AddVertices(qnew, vertices)
        self.AddEdges(qnew_parent, qnew)
        self.DrawEdges(self.src_map, qnew_parent, qnew)

        return len_qnew_path, qnew_parent

    def RewireNewChild(self, qnew, len_qnew_path, protential_children):
        '''
        rewire the edges of the qnew and its protential_children
        '''
        qnew_children = None
        for child in protential_children:
            tmp_len_path = len_qnew_path + \
                self.Distance(qnew.pos, child.pos)

            if self.CollsionFree(child, qnew, self.grid_size):
                child_path = self.FindPathByParentPointer(child)
                if tmp_len_path < self.CalculatePathLength(child_path):
                    qnew_children = child
                    self.AddEdges(qnew, qnew_children)
                    self.DrawEdges(self.src_map, qnew,
                                   qnew_children)
        return

    def Planning(self):
        '''
        RRT_STAR planning
        '''

        vertices = []
        self.AddVertices(self.qstart, vertices)
        self.AddEdges(None, self.qstart)

        k = 0
        while k <= self.max_steps:
            k += 1

            qrand = self.GenerateRandomNode(
                [0, self.map_shape[0]], [0, self.map_shape[1]],
                self.qgoal, self.goal_prob)

            _, qnear = self.FindNearestNode(qrand, vertices)
            qnew = self.ExtendTree(qnear, qrand, self.step_size)

            if qnew and self.CollsionFree(qnear, qnew, self.grid_size):

                neighbors_free = self.FindNeighbors(
                    qnew, vertices, self.neighbor_radius)

                '''
                neighbors = list(filter(lambda qnode: self.Distance(
                    qnode.pos, qnew.pos) < self.neighbor_radius, vertices))
                neighbors_free = []
                for parent in neighbors:
                    if self.CollsionFree(parent, qnew, self.grid_size):
                        neighbors_free.append(parent)
                '''

                len_qnew_path, qnew_parent = self.RewireNewParent(
                    qnew, vertices, neighbors_free)

                '''
                len_qnew_path = float('inf')
                qnew_parent = None
                for parent in neighbors_free:
                    parent_path = self.FindPathByParentPointer(parent)
                    len_parent_path = self.CalculatePathLength(parent_path)
                    tmp_len_path = len_parent_path + \
                        self.Distance(qnew.pos, parent.pos)

                    if tmp_len_path < len_qnew_path:
                        len_qnew_path = tmp_len_path
                        qnew_parent = parent                

                self.AddVertices(qnew, vertices)
                self.AddEdges(qnew_parent, qnew)
                self.DrawEdges(self.src_map, qnew_parent, qnew)
                '''

                if qnew_parent:
                    neighbors_free.remove(qnew_parent)
                self.RewireNewChild(qnew, len_qnew_path, neighbors_free)
                '''
                qnew_children = None
                for child in neighbors_free:
                    tmp_len_path = len_qnew_path + \
                        self.Distance(qnew.pos, child.pos)

                    if self.CollsionFree(child, qnew, self.grid_size):
                        child_path = self.FindPathByParentPointer(child)
                        if tmp_len_path < self.CalculatePathLength(child_path):
                            qnew_children = child
                            self.AddEdges(qnew, qnew_children)
                            self.DrawEdges(self.src_map, qnew,
                                           qnew_children)
                '''

                if self.IsArrival(qnew, self.qgoal, self.step_size):
                    print("Found")

                    self.AddVertices(self.qgoal, vertices)
                    self.AddEdges(qnew, self.qgoal)
                    self.DrawEdges(self.src_map, qnew, self.qgoal)

                    path = self.FindPath(vertices)
                    self.DrawPath(self.src_map, path)
                    smooth_path = self.SmoothPath(path, self.grid_size)
                    self.DrawPath(self.src_map, smooth_path)

                    return True

        print("NotFound")
        return False


if __name__ == "__main__":

    map_path = 'map/area6.png'
    qstart = [20, 20]
    qgoal = [480, 480]
    max_steps = 1000
    step_size = 20
    goal_prob = 0.01
    grid_size = 10
    neighbor_radius = 3 * step_size

    rrt_star = RRT_STAR(map_path, qstart, qgoal, grid_size,
                        step_size, neighbor_radius, max_steps, goal_prob)
    input('press any key to start planning:')
    rrt_star.Planning()
    input('press any key to quit:')
