# RobotMotionPlanner
This is a repository of three different 2D robot motion planners, RRT, PRM and A*.
By adjusting the `main()` function to change the robot start position, end position, the size of the robot, the obstacle list and the planner type.

    # define the main function
    def main(sx=0.0, sy=0.0, gx=6.0, gy=10.0, is_show=True, planner: str = "rrt"):
        print("start " + __file__)

        # ====Obstacle List====
        obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2), (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
        
        
The result is like:
<p align="center">
      <center>
      <img src="https://github.com/GUOkekkk/RobotMotionPlanner/blob/main/pics/rrtplanner.png" width="25%" height="25%" />
      Figure 1. RRT Planner Result
      </center>
</p>

<p align="center">
      <center>
      <img src="https://github.com/GUOkekkk/RobotMotionPlanner/blob/main/pics/prmplanner.png" width="25%" height="25%" />
      Figure 1. PRM Planner Result
      </center>
</p>

<p align="center">
      <center>
      <img src="https://github.com/GUOkekkk/RobotMotionPlanner/blob/main/pics/astarplanner.png" width="25%" height="25%" />
      Figure 1. Astar Planner Result
      </center>
</p>
