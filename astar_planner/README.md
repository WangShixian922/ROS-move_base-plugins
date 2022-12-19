This global planner uses the A* algorithm to generate a global path. The planner inherits interfaces defined by nav_core as a move_base plugin and refers to
the Navfn and Carrot planners.

To use this A* planner, you should put it in the src folder of your workspace and compile it. After that, the launch file for move_base should be modified 
correspondingly. To be more specific,

param name="base_global_planner" value="astar_planner/AstarPlanner"

The main process of A* is maintaining the open and closed set. For each search loop, the node in the open set with the smallest f value is popped as the 
current node, and all the f values of the current node’s successors are updated, which will be put in the open set. After searching, the current node is 
marked as closed. Once the target point is included in successors, the searching process is done. And the optimal path can be obtained by a backing search 
of the predecessor, starting from the target point.

If it's difficult to build the ROS environment, you can refer to the Matlab version I uploaded before.

Any suggestion is appreciated. Do not hesitate to drop it.
