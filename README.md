# RobotMotionPlanner
This is a repository of three different 2D robot motion planners, RRT, PRM and A*.
By adjusting the `main()` function to change the robot start position, end position, the size of the robot, the obstacle list and the planner type.

    # define the main function
    def main(sx=0.0, sy=0.0, gx=6.0, gy=10.0, is_show=True, planner: str = "rrt"):
        print("start " + __file__)

        # ====Obstacle List====
        obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2), (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
        
        
The result is like:

<center>
    <img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);"
    src="https://github.com/GUOkekkk/RobotMotionPlanner/blob/main/pics/rrtplanner.png">
    <br>
    <div style="color:orange; border-bottom: 1px solid #d9d9d9;
    display: inline-block;
    color: #999;
    padding: 2px;">RRT Planner Result</div>
</center>

<center>
    <img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);"
    src="https://github.com/GUOkekkk/RobotMotionPlanner/blob/main/pics/prmplanner.png">
    <br>
    <div style="color:orange; border-bottom: 1px solid #d9d9d9;
    display: inline-block;
    color: #999;
    padding: 2px;">PRM Planner Result</div>
</center>




<center>
    <img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);"
    src="https://github.com/GUOkekkk/RobotMotionPlanner/blob/main/pics/astarpalnner.png">
    <br>
    <div style="color:orange; border-bottom: 1px solid #d9d9d9;
    display: inline-block;
    color: #999;
    padding: 2px;">Astar Planner Result</div>
</center>


<center><img src="https://github.com/GUOkekkk/RobotMotionPlanner/blob/main/pics/astarpalnner.png"/></center>
<center>Astar Planner Result</center>

