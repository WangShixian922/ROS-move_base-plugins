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

#include "../include/astar_planner/Astar.h"

namespace astar_planner
{
    //
    // Set/Reset map size
    //
    void AStar::setMapSize(int xs, int ys)
    {
        nx = xs;
        ny = ys;
        ns = nx * ny;

        if (costarr)
            delete[] costarr;

        costarr = new COSTTYPE[ns];                // cost array, 2d config space
        memset(costarr, 0, ns * sizeof(COSTTYPE)); // set all members as 0
    }

    //
    // set up cost array, usually from ROS
    //
    void AStar::setCostmap(const COSTTYPE *cmap, bool isROS, bool allow_unknown)
    {
        COSTTYPE *cm = costarr;
        if (isROS) // ROS-type cost array
        {
            for (int i = 0; i < ny; i++)
            {
                int k = i * nx;
                for (int j = 0; j < nx; j++, k++, cmap++, cm++)
                {
                    // This transforms the incoming cost values:
                    // COST_OBS                 -> COST_OBS (incoming "lethal obstacle")
                    // COST_OBS_ROS             -> COST_OBS (incoming "inscribed inflated obstacle")
                    // values in range 0 to 252 -> values from COST_NEUTRAL to COST_OBS_ROS.
                    *cm = COST_OBS;
                    int v = *cmap;
                    if (v < COST_OBS_ROS)
                    {
                        v = COST_NEUTRAL + COST_FACTOR * v;
                        if (v >= COST_OBS)
                            v = COST_OBS - 1;
                        *cm = v;
                    }
                    else if (v == COST_UNKNOWN_ROS && allow_unknown)
                    {
                        v = COST_OBS - 1;
                        *cm = v;
                    }
                }
            }
        }

        else // not a ROS map, just a PGM
        {
            for (int i = 0; i < ny; i++)
            {
                int k = i * nx;
                for (int j = 0; j < nx; j++, k++, cmap++, cm++)
                {
                    *cm = COST_OBS;
                    if (i < 7 || i > ny - 8 || j < 7 || j > nx - 8)
                        continue; // don't do borders
                    int v = *cmap;
                    if (v < COST_OBS_ROS)
                    {
                        v = COST_NEUTRAL + COST_FACTOR * v;
                        if (v >= COST_OBS)
                            v = COST_OBS - 1;
                        *cm = v;
                    }
                    else if (v == COST_UNKNOWN_ROS)
                    {
                        v = COST_OBS - 1;
                        *cm = v;
                    }
                }
            }
        }
    }

    bool AStar::greaterSort(AstarNode *a, AstarNode *b)
    {
        return (a->f > b->f);
    }

    bool AStar::expand(int start[2], int goal[2])
    {
        // ROS_INFO("start is [%d,%d]", start[0], start[1]);
        // set up the start node
        Astar_node->x = start[0];
        Astar_node->y = start[1];
        Astar_node->n = Astar_node->y * nx + Astar_node->x;
        Astar_node->g = 0;
        Astar_node->h = sqrt(pow((Astar_node->x - goal[0]), 2) + pow((Astar_node->y - goal[1]), 2));
        Astar_node->f = Astar_node->g + Astar_node->h;
        Astar_node->pred_x = Astar_node->x;
        Astar_node->pred_y = Astar_node->y;
        openSet.push_back(Astar_node);
        // ROS_INFO("the first node, x=%d, y=%d, n=%d, g=%f, h=%f, f=%f, pred_x=%d, pred_y=%d", Astar_node->x, Astar_node->y, Astar_node->n, Astar_node->g, Astar_node->h, Astar_node->f, Astar_node->pred_x, Astar_node->pred_y);
        // ROS_INFO("1.the size of oS is %d", openSet.size());
        // ROS_INFO("1.the value of An is %d", Astar_node);

        Astar_node = new AstarNode;
        // ROS_INFO("2.the size of oS is %d", openSet.size());
        // ROS_INFO("2.the value of An is %d", Astar_node);

        // int k;
        // k = 0;

        while (true)
        {
            // ROS_INFO("3 .the size of oS is %d", openSet.size());

            if (openSet.empty())
            {
                // ROS_INFO("last node is [%d,%d]", current_node->x, current_node->y);
                ROS_INFO("no valid path");
                return false;
            }
            std::sort(openSet.begin(), openSet.end(), greaterSort); // sort the openset according to the f value
            current_node = openSet.back();                          // choose the node with smallest f value as the current node
            // if (1)
            // {
            //     ROS_INFO("current node is [%d,%d]", current_node->x, current_node->y);
            // }

            closedSet.push_back(current_node); // put the current node into closedset
            openSet.pop_back();                // remove the current node from openset
            // search the neighbors,8 connections
            // int k = 0;
            for (int i = -1; i < 2; i++)
            {
                for (int j = -1; j < 2; j++)
                {
                    if (i != 0 || j != 0) // don't expand to the current node itself
                    {
                        // k++;
                        bool closed = false;
                        Astar_node->x = current_node->x + i;
                        Astar_node->y = current_node->y + j;
                        Astar_node->n = Astar_node->y * nx + Astar_node->x;
                        for (std::vector<AstarNode *>::iterator it = closedSet.begin(); it != closedSet.end(); it++)
                        {
                            if ((*it)->n == Astar_node->n)
                            {
                                closed = true;
                            }
                        }

                        if (costarr[Astar_node->n] < COST_OBS && !closed && Astar_node->x <= nx && Astar_node->x >= 0 && Astar_node->y <= ny && Astar_node->y >= 0)
                        // don't expand to the obstacles and closed nodes, neither outside the bound
                        {
                            bool keepIt = false;
                            Astar_node->g = current_node->g + costarr[Astar_node->n];
                            // if the expanded node is already in the openset, reserve the one with smaller g value
                            for (std::vector<AstarNode *>::iterator it = openSet.begin(); it != openSet.end(); it++)
                            {
                                if ((*it)->n == Astar_node->n)
                                {

                                    if ((*it)->g > Astar_node->g)
                                    {
                                        delete (*it);
                                        *it = NULL;
                                        openSet.erase(it);
                                    }
                                    else
                                    {
                                        keepIt = true;
                                    }
                                }
                            }
                            if (!keepIt)
                            {
                                Astar_node->h = sqrt(pow((Astar_node->x - goal[0]), 2) + pow((Astar_node->y - goal[1]), 2));
                                Astar_node->f = Astar_node->g + Astar_node->h;
                                Astar_node->pred_x = current_node->x;
                                Astar_node->pred_y = current_node->y;
                                if (Astar_node->x == goal[0] && Astar_node->y == goal[1]) // once the goal is expanded, finish the expanding process
                                {
                                    closedSet.push_back(Astar_node);
                                    return true;
                                }
                                else
                                {
                                    openSet.push_back(Astar_node);
                                    Astar_node = new AstarNode;
                                    // ROS_INFO("add a new node");
                                }
                            }
                        }
                    }
                }
            }
            // ROS_INFO("the k is %d", k);

            // if (k < 3)
            // {
            //     for (std::vector<AstarNode *>::iterator it = openSet.begin(); it != openSet.end(); it++)
            //     {
            //         ROS_INFO("node, x=%d, y=%d, n=%d, g=%f, h=%f, f=%f, pred_x=%d, pred_y=%d.\n",
            //                  (*it)->x, (*it)->y, (*it)->n, (*it)->g, (*it)->h, (*it)->f, (*it)->pred_x, (*it)->pred_y);
            //     }
            //     k++;
            // }
        }
    }

    bool AStar::generatePath(std::vector<geometry_msgs::PoseStamped> &plan)
    {
        temp = closedSet.back();
        // ROS_INFO("the closedSet back [%d,%d],[%d,%d]", closedSet.back()->x, closedSet.back()->y, closedSet.back()->pred_x, closedSet.back()->pred_y);
        // ROS_INFO("1.the temp [%d,%d],[%d,%d]",temp->x,temp->y,temp->pred_x,temp->pred_y);
        // ROS_INFO("2.the temp [%d,%d],[%d,%d]",(*temp).x,(*temp).y,(*temp).pred_x,(*temp).pred_y);

        // ROS_INFO("the size of cS is %d", closedSet.size());
        waypoint.pose.position.x = (double)temp->x;
        waypoint.pose.position.y = (double)temp->y;
        // ROS_INFO("1.path node [%g,%g]", waypoint.pose.position.x, waypoint.pose.position.y);

        plan.push_back(waypoint);
        // ROS_INFO("1.the size of plan is %d", plan.size());

        while (temp->g != 0)
        {
            for (std::vector<AstarNode *>::iterator it = closedSet.begin(); it != closedSet.end(); it++)
            {
                if ((*it)->x == temp->pred_x && (*it)->y == temp->pred_y)
                {
                    // ROS_INFO("1.closed node [%d,%d],[%d,%d]", (*it)->x, (*it)->y, (*it)->pred_x, (*it)->pred_y);
                    temp = (*it);
                    // ROS_INFO("2.closed node [%d,%d],[%d,%d]", temp->x, temp->y, temp->pred_x, temp->pred_y);
                    waypoint.pose.position.x = (double)temp->x;
                    waypoint.pose.position.y = (double)temp->y;
                    // ROS_INFO("path node [%g,%g]", waypoint.pose.position.x, waypoint.pose.position.y);
                    plan.push_back(waypoint);
                    // ROS_INFO("2.the size of plan is %d", plan.size());
                }
            }
        }
        return true;
    }
    

    AStar::AStar(int xs, int ys)
    {
        costarr = NULL;
        Astar_node = NULL;
        current_node = NULL;
        temp = NULL;
        setMapSize(xs, ys);
        // goal and start
        Astar_node = new AstarNode;
        openSet.clear();
        closedSet.clear();
    }

    AStar::~AStar()
    {
        if (costarr)
            delete[] costarr;

        for (std::vector<AstarNode *>::iterator it = openSet.begin(); it != openSet.end(); it++)
        {
            if ((*it) != NULL)
            {
                delete (*it);
                *it = NULL;
                openSet.clear();
            }
        }

        for (std::vector<AstarNode *>::iterator it = closedSet.begin(); it != closedSet.end(); it++)
        {
            if ((*it) != NULL)
            {
                delete (*it);
                *it = NULL;
                closedSet.clear();
            }
        }
    }
}