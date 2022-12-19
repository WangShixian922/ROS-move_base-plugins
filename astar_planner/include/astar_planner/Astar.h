/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef Astar_H
#define Astar_H
#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>

// cost defs
#define COST_UNKNOWN_ROS 255 // 255 is unknown cost
#define COST_OBS 254         // 254 for forbidden regions
#define COST_OBS_ROS 253     // ROS values of 253 are obstacles

// astar cost values are set to
// COST_NEUTRAL + COST_FACTOR * costmap_cost_value.
// Incoming costmap cost values are in the range 0 to 252.
// With COST_NEUTRAL of 50, the COST_FACTOR needs to be about 0.8 to
// ensure the input values are spread evenly over the output range, 50
// to 253.  If COST_FACTOR is higher, cost values will have a plateau
// around obstacles and the planner will then treat (for example) the
// whole width of a narrow hallway as equally undesirable and thus
// will not plan paths down the center.

#define COST_NEUTRAL 1  // Set this to "open space" value
#define COST_FACTOR 0.8 // Used for translating costs in AStar::setCostmap()

// Define the cost type in the case that it is not set. However, this allows
// clients to modify it without changing the file. Arguably, it is better to require it to
// be defined by a user explicitly
#ifndef COSTTYPE
#define COSTTYPE unsigned char // Whatever is used...
#endif

namespace astar_planner
{
    class AStar
    {
    public:
        AStar(int xs, int ys);
        ~AStar();
        void setMapSize(int xs, int ys);
        void setCostmap(const COSTTYPE *cmap, bool isROS = true, bool allow_unknown = true); /** sets up the cost map */
        bool expand(int start[2], int goal[2]);

        /**
         * @brief  Generates the path by back-searching for predecessors
         */
        bool generatePath(std::vector<geometry_msgs::PoseStamped> &plan);

        int goal[2];
        int start[2];
        int nx, ny, ns; /** size of grid, in pixels */

        struct AstarNode
        {
            int x; // x coordinate
            int y;
            int n;    // index
            double g; // cost
            double h; // heuristic cost
            double f;
            int pred_x; // record of the predecessor, used by back-searching
            int pred_y;
        };

        static bool greaterSort(AstarNode *a, AstarNode *b);
        std::vector<AstarNode *> openSet, closedSet;
        AstarNode *Astar_node, *current_node, *temp;
        COSTTYPE *costarr;
        geometry_msgs::PoseStamped waypoint;
    };
}

#endif