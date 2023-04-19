# import the package
import math
import random
import matplotlib.pyplot as plt
import numpy as np
import time

# import the planner
from RRTPlanner import RRT
from PRMplanner import PRM
from ASTARPlanner import ASTAR


# define a function to calculate the length of the path
def cal_len_path(path):
    path = np.asarray(path)
    len_path = 0
    for i in range(len(path) - 1):
        len_path += np.linalg.norm((path[i] - path[i + 1]))

    return len_path


# define the main function
def main(sx=0.0, sy=0.0, gx=6.0, gy=10.0, is_show=True, planner: str = "rrt"):
    print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2), (9, 5, 2), (8, 10, 1)]  # [x, y, radius]

    start = time.time()
    # Use the RRT planner
    # Set Initial parameters
    if planner == "rrt":
        print("-------------------------------")
        print("Using RRT planner")
        rrt = RRT(
            start=[sx, sy],
            goal=[gx, gy],
            rand_area=[-2, 15],
            obstacle_list=obstacleList,
            # play_area=[0, 10, 0, 14], # a box constrain
            robot_radius=0.8,
        )
        path = rrt.planning(animation=is_show)
        end = time.time()

        # Draw final path
        if is_show:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], "-r", label="Final path")
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.legend()
            plt.show()

    elif planner == "prm":
        print("-------------------------------")
        print("Using PRM planner")

        prm = PRM(
            start=[sx, sy],
            goal=[gx, gy],
            rand_area=[-2, 15],
            obstacle_list=obstacleList,
            robot_radius=0.8,
            num_samples=500,
        )
        path = prm.planning(initialRandomSeed=10)
        end = time.time()

        # Draw final path
        if is_show:
            prm.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], "-r", label="Final path")
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.legend()
            plt.show()

    elif planner == "astar":
        print("-------------------------------")
        print("Using A* planner")
        astar = ASTAR(
            start=[sx, sy],
            goal=[gx, gy],
            rand_area=[-2, 15],
            obstacle_list=obstacleList,
            robot_radius=0.8,
            num_samples=500,
        )
        path = astar.planning(initialRandomSeed=10)
        end = time.time()

        # Draw final path
        if is_show:
            astar.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], "-r", label="Final path")
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.legend()
            plt.show()

    if path is None:
        print("Cannot find a path. Use: {:.4f} s".format(end - start))
    else:
        print("Found the path!! Use: {:.4f} s".format(end - start))
        print("The path length is {:.4f}".format(cal_len_path(path)))


if __name__ == "__main__":
    main(planner="rrt")
    main(planner="prm")
    main(planner="astar")
