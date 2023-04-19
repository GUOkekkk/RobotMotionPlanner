# The PRM planner witht the Dijkstra's algorithm
import math
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors


class Edge:
    def __init__(self, to_node, length):
        self.to_node = to_node
        self.length = length


class Graph:
    def __init__(self):
        self.nodes = set()
        self.edges = dict()

    def add_node(self, node):
        self.nodes.add(node)

    def add_edge(self, from_node, to_node, length):
        edge = Edge(to_node, length)
        if from_node in self.edges:
            from_node_edges = self.edges[from_node]
        else:
            self.edges[from_node] = dict()
            from_node_edges = self.edges[from_node]
        from_node_edges[to_node] = edge


def min_dist(q, dist):
    """
    Returns the node with the smallest distance in q.
    Implemented to keep the main algorithm clean.
    """
    min_node = None
    for node in q:
        if min_node == None:
            min_node = node
        elif dist[node] < dist[min_node]:
            min_node = node

    return min_node


INFINITY = float("Infinity")


def heuristic(a, b):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def to_array(prev, from_node):
    """Creates an ordered list of labels as a route."""
    previous_node = prev[from_node]
    route = [from_node]

    while previous_node != INFINITY:
        route.append(previous_node)
        temp = previous_node
        previous_node = prev[temp]

    route.reverse()
    return route


def dis_point_to_seg_line(p, a, b):
    a, b, p = np.array(a), np.array(b), np.array(p)  # trans to np.array
    d = np.divide(b - a, np.linalg.norm(b - a))  # normalized tangent vector
    s = np.dot(a - p, d)  # signed parallel distance components
    t = np.dot(p - b, d)
    h = np.maximum.reduce([s, t, 0])  # clamped parallel distance
    c = np.cross(p - a, d)  # perpendicular distance component
    return np.hypot(h, np.linalg.norm(c))


def plotPoints(points):
    x = [item[0] for item in points]
    y = [item[1] for item in points]
    plt.scatter(x, y, c="black", s=1)


class ASTAR:
    """
    Class for PRM planning

    Returns:
        _type_: _description_
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y, cost):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent = None

    def __init__(
        self,
        start,
        goal,
        obstacle_list,
        rand_area,
        num_samples=500,
        k_neighbor=10,
        expand_dis=3.0,
        path_resolution=0.5,
        goal_sample_rate=5,
        max_iter=500,
        robot_radius=0.0,
    ):
        self.expand_dis = expand_dis
        self.rand_area = rand_area
        self.start = start
        self.goal = goal
        self.num_samples = num_samples
        self.k_neighbor = k_neighbor
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius
        self.graph = Graph()
        self.solutionFound = False

    def planning(self, initialRandomSeed=None):
        # Use the random seed to check the performance
        seed = initialRandomSeed
        # Keep resampling if no solution found
        while not self.solutionFound:

            print("Trying with random seed {}".format(seed))
            np.random.seed(seed)

            # Generate n random samples called milestones
            self.genCoords()

            # Check if milestones are collision free
            self.checkIfCollisonFree()

            # Link each milestone to k nearest neighbours.
            # Retain collision free links as local paths.
            self.findNearestNeighbour()

            # Search for shortest path from start to end node - Using Dijksta's shortest path alg
            self.shortestPath()
        return self.path

    def genCoords(self):
        self.coordsList = (
            np.random.random(size=(self.num_samples, 2)) * (self.rand_area[1] - self.rand_area[0]) + self.rand_area[0]
        )
        # Adding begin and end points
        self.start = np.asarray(self.start)
        self.goal = np.asarray(self.goal)
        self.coordsList = np.concatenate((self.coordsList, self.start.reshape(1, 2), self.goal.reshape(1, 2)), axis=0)

    def checkIfCollisonFree(self):
        collision = False
        self.collisionFreePoints = np.array([])
        for point in self.coordsList:
            collision = self.checkPointCollision(point)
            if not collision:
                if self.collisionFreePoints.size == 0:
                    self.collisionFreePoints = point
                else:
                    self.collisionFreePoints = np.vstack([self.collisionFreePoints, point])
        # plotPoints(self.collisionFreePoints)

    def findNearestNeighbour(self):
        X = self.collisionFreePoints
        knn = NearestNeighbors(n_neighbors=self.k_neighbor)
        knn.fit(X)
        distances, indices = knn.kneighbors(X)
        self.collisionFreePaths = np.empty((1, 2), int)
        for i, p in enumerate(X):
            # Ignoring nearest neighbour - nearest neighbour is the point itself
            for j, neighbour in enumerate(X[indices[i][1:]]):
                start_line = p
                end_line = neighbour
                if not self.checkPointCollision(start_line) and not self.checkPointCollision(end_line):
                    if not self.checkLineCollision(start_line, end_line):
                        self.collisionFreePaths = np.concatenate(
                            (self.collisionFreePaths, p.reshape(1, 2), neighbour.reshape(1, 2)), axis=0
                        )

                        a = str(self.findNodeIndex(p))
                        b = str(self.findNodeIndex(neighbour))
                        self.graph.add_node(a)
                        self.graph.add_edge(a, b, distances[i, j + 1])
                        x = [p[0], neighbour[0]]
                        y = [p[1], neighbour[1]]
                        plt.plot(x, y, c="purple", linewidth=0.7)

    def astar(self):
        graph = self.graph
        source = self.startNode
        goal = self.endNode

        q = set()
        dist = {}
        prev = {}
        g_score = {}

        for v in graph.nodes:
            dist[v] = INFINITY
            prev[v] = INFINITY
            g_score[v] = INFINITY
            q.add(v)

        dist[source] = 0
        g_score[source] = 0
        f_score = g_score.copy()

        while q:
            # node with the least distance selected first
            current = min_dist(q, f_score)

            if current == goal:
                return dist, prev

            q.remove(current)

            try:
                if current in graph.edges:
                    for _, v in graph.edges[current].items():
                        tentative_g_score = g_score[current] + v.length

                        if tentative_g_score < g_score[v.to_node]:
                            # a shorter path to v has been found
                            prev[v.to_node] = current
                            v_point = self.findPointsFromNode(v.to_node)
                            goal_point = self.findPointsFromNode(goal)
                            dist[v.to_node] = dist[current] + heuristic(v_point, goal_point)
                            g_score[v.to_node] = tentative_g_score
                            f_score[v.to_node] = g_score[v.to_node] + dist[v.to_node]
            except:
                pass

        return None, None

    def shortestPath(self):
        self.startNode = str(self.findNodeIndex(self.start))
        self.endNode = str(self.findNodeIndex(self.goal))

        dist, prev = self.astar()

        pathToEnd = to_array(prev, self.endNode)

        if len(pathToEnd) > 1:
            self.solutionFound = True
        else:
            return

        pointsToDisplay = [(self.findPointsFromNode(path)) for path in pathToEnd]
        self.path = np.vstack((self.start, pointsToDisplay, self.goal))
        # pointsToEnd = [str(self.findPointsFromNode(path)) for path in pathToEnd]
        # print("****Output****")
        # print(
        #     "The quickest path from {} to {} is: \n {} \n with a distance of {}".format(
        #         self.collisionFreePoints[int(self.startNode)],
        #         self.collisionFreePoints[int(self.endNode)],
        #         " \n ".join(pointsToEnd),
        #         str(dist[self.endNode]),
        #     )
        # )

    def checkLineCollision(self, start_line, end_line):
        for (ox, oy, size) in self.obstacle_list:
            if dis_point_to_seg_line([ox, oy], start_line, end_line) <= (size + self.robot_radius):
                return True
        return False

    def findNodeIndex(self, p):
        return np.where((self.collisionFreePoints == p).all(axis=1))[0][0]

    def findPointsFromNode(self, n):
        return self.collisionFreePoints[int(n)]

    def checkPointCollision(self, point):
        for (ox, oy, size) in self.obstacle_list:
            dist = np.linalg.norm(([ox - point[0], oy - point[1]]))
            if dist <= (size + self.robot_radius):
                return True
        return False

    def draw_graph(self):
        plt.title("The A* planner")
        plt.plot(self.start[0], self.start[1], "or", label="start position")
        plt.plot(self.goal[0], self.goal[1], "xr", label="goal position")
        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)
        plt.legend()
        plt.axis("equal")
        plt.plot(self.start, self.path[1], c="purple", linewidth=0.7, label="Probabilistic Roadmap")
        plotPoints(self.collisionFreePoints)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)
