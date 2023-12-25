/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *   * Neither the name of the institute nor the names of its
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
 *
 * Authors: Christoph Rösmann, Franz Albers
 *********************************************************************/

#include <teb_local_planner/graph_search.h>
#include <teb_local_planner/homotopy_class_planner.h>

#include <teb_local_planner/MMT_control.h>

namespace teb_local_planner
{

void GraphSearchInterface::DepthFirst(HcGraph& g, std::vector<HcGraphVertexType>& visited, const HcGraphVertexType& goal, double start_orientation,
                                      double goal_orientation, const geometry_msgs::Twist* start_velocity, bool free_goal_vel)
{
  // see http://www.technical-recipes.com/2011/a-recursive-algorithm-to-find-all-paths-between-two-given-nodes/ for details on finding all simple paths

  if ((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
    return; // We do not need to search for further possible alternative homotopy classes.

  HcGraphVertexType back = visited.back();

  /// Examine adjacent nodes
  HcGraphAdjecencyIterator it, end;
  for ( boost::tie(it,end) = boost::adjacent_vertices(back,g); it!=end; ++it)
  {
    if ( std::find(visited.begin(), visited.end(), *it)!=visited.end() )
      continue; // already visited

    if ( *it == goal ) // goal reached
    {
      visited.push_back(*it);

      // Add new TEB, if this path belongs to a new homotopy class
      hcp_->addAndInitNewTeb(visited.begin(), visited.end(), boost::bind(getVector2dFromHcGraph, _1, boost::cref(graph_)),
                             start_orientation, goal_orientation, start_velocity, free_goal_vel);

      visited.pop_back();
      break;
    }
  }

  /// Recursion for all adjacent vertices
  for ( boost::tie(it,end) = boost::adjacent_vertices(back,g); it!=end; ++it)
  {
    if ( std::find(visited.begin(), visited.end(), *it)!=visited.end() || *it == goal)
      continue; // already visited || goal reached


    visited.push_back(*it);

    // recursion step
    DepthFirst(g, visited, goal, start_orientation, goal_orientation, start_velocity, free_goal_vel);

    visited.pop_back();
  }
}



void lrKeyPointGraph::createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity, bool free_goal_vel)
{
  // Clear existing graph and paths
  clearGraph();
  if((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
    return;
  // Direction-vector between start and goal and normal-vector:
  Eigen::Vector2d diff = goal.position()-start.position();

  if (diff.norm()<cfg_->goal_tolerance.xy_goal_tolerance)
  {
    ROS_DEBUG("HomotopyClassPlanner::createProbRoadmapGraph():1 xy-goal-tolerance already reached.");
    if (hcp_->getTrajectoryContainer().empty())
    {
      ROS_INFO("HomotopyClassPlanner::createProbRoadmapGraph(): Initializing a small straight line to just correct orientation errors.");
      hcp_->addAndInitNewTeb(start, goal, start_velocity, free_goal_vel);
    }
    return;
  }

  Eigen::Vector2d normal(-diff[1],diff[0]); // normal-vector
  normal.normalize();
  normal = normal*dist_to_obst; // scale with obstacle_distance;

  // Insert Vertices
  HcGraphVertexType start_vtx = boost::add_vertex(graph_); // start vertex
  graph_[start_vtx].pos = start.position();
  diff.normalize();

  // store nearest obstacle keypoints -> only used if limit_obstacle_heading is enabled
  std::pair<HcGraphVertexType,HcGraphVertexType> nearest_obstacle; // both vertices are stored
  double min_dist = DBL_MAX;

  if (hcp_->obstacles()!=NULL)
  {
    for (ObstContainer::const_iterator it_obst = hcp_->obstacles()->begin(); it_obst != hcp_->obstacles()->end(); ++it_obst)
    {
      // check if obstacle is placed in front of start point
      Eigen::Vector2d start2obst = (*it_obst)->getCentroid() - start.position();
      double dist = start2obst.norm();
      if (start2obst.dot(diff)/dist<0.1)
        continue;

      // Add Keypoints
      HcGraphVertexType u = boost::add_vertex(graph_);
      graph_[u].pos = (*it_obst)->getCentroid() + normal;
      HcGraphVertexType v = boost::add_vertex(graph_);
      graph_[v].pos = (*it_obst)->getCentroid() - normal;

      // store nearest obstacle
      if (obstacle_heading_threshold && dist<min_dist)
      {
        min_dist = dist;
        nearest_obstacle.first = u;
        nearest_obstacle.second = v;
      }
    }
  }

  HcGraphVertexType goal_vtx = boost::add_vertex(graph_); // goal vertex
  graph_[goal_vtx].pos = goal.position();

  // Insert Edges
  HcGraphVertexIterator it_i, end_i, it_j, end_j;
  for (boost::tie(it_i,end_i) = boost::vertices(graph_); it_i!=end_i-1; ++it_i) // ignore goal in this loop
  {
    for (boost::tie(it_j,end_j) = boost::vertices(graph_); it_j!=end_j; ++it_j) // check all forward connections
    {
      if (it_i==it_j)
        continue;
      // TODO: make use of knowing in which order obstacles are inserted and that for each obstacle 2 vertices are added,
      // therefore we must only check one of them.
      Eigen::Vector2d distij = graph_[*it_j].pos-graph_[*it_i].pos;
      distij.normalize();
      // Check if the direction is backwards:
      if (distij.dot(diff)<=obstacle_heading_threshold)
        continue;


      // Check start angle to nearest obstacle
      if (obstacle_heading_threshold && *it_i==start_vtx && min_dist!=DBL_MAX)
      {
        if (*it_j == nearest_obstacle.first || *it_j == nearest_obstacle.second)
        {
          Eigen::Vector2d keypoint_dist = graph_[*it_j].pos-start.position();
          keypoint_dist.normalize();
          Eigen::Vector2d start_orient_vec( cos(start.theta()), sin(start.theta()) ); // already normalized
          // check angle
          if (start_orient_vec.dot(keypoint_dist) <= obstacle_heading_threshold)
          {
            ROS_DEBUG("createGraph() - deleted edge: limit_obstacle_heading");
            continue;
          }
        }
      }

      // Collision Check

      if (hcp_->obstacles()!=NULL)
      {
        bool collision = false;
        for (ObstContainer::const_iterator it_obst = hcp_->obstacles()->begin(); it_obst != hcp_->obstacles()->end(); ++it_obst)
        {
          if ( (*it_obst)->checkLineIntersection(graph_[*it_i].pos,graph_[*it_j].pos, 0.5*dist_to_obst) )
          {
            collision = true;
            break;
          }
        }
        if (collision)
          continue;
      }

      // Create Edge
      boost::add_edge(*it_i,*it_j,graph_);
    }
  }


  // Find all paths between start and goal!
  std::vector<HcGraphVertexType> visited;
  visited.push_back(start_vtx);
  DepthFirst(graph_,visited,goal_vtx, start.theta(), goal.theta(), start_velocity, free_goal_vel);
}



void ProbRoadmapGraph::createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity, bool free_goal_vel)
{
  // 清除现有的图和路径
  clearGraph();
  if((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
    return;
  // 开始和目标之间的方向向量和法向量:
  PoseSE2 goal2;
  if(MMT_min_turning_radius!=0){
    goal2=goal;
  }else{
    PoseSE2 goal22(goal.position(),start.theta());
    goal2=goal22;
  }
  Eigen::Vector2d diff = goal2.position()-start.position();
  double start_goal_dist = diff.norm();

  if (start_goal_dist<cfg_->goal_tolerance.xy_goal_tolerance)
  {
    ROS_DEBUG("HomotopyClassPlanner::createProbRoadmapGraph():2 xy-goal-tolerance already reached.");
    MMT_at_xy_terget = true;
    if (hcp_->getTrajectoryContainer().empty())
    {
      ROS_INFO("HomotopyClassPlanner::createProbRoadmapGraph(): Initializing a small straight line to just correct orientation errors.");
      hcp_->addAndInitNewTeb(start, goal2, start_velocity, free_goal_vel);
    }
    return;
  }else{
    MMT_at_xy_terget = false;
  }
  Eigen::Vector2d normal(-diff.coeffRef(1),diff.coeffRef(0)); // 法向量
  normal.normalize();

  // 现在在开始，目标和两侧之间的指定宽度之间采样顶点
  // 让我们从开始和目标之间的一个正方形区域开始（可能稍后将其更改为圆形或其他形状）

  double area_width = cfg_->hcp.roadmap_graph_area_width;

  boost::random::uniform_real_distribution<double> distribution_x(0, start_goal_dist * cfg_->hcp.roadmap_graph_area_length_scale);
  boost::random::uniform_real_distribution<double> distribution_y(0, area_width);

  double phi = atan2(diff.coeffRef(1),diff.coeffRef(0)); // 旋转区域的角度
  Eigen::Rotation2D<double> rot_phi(phi);

  Eigen::Vector2d area_origin;
  if (cfg_->hcp.roadmap_graph_area_length_scale != 1.0)
    area_origin = start.position() + 0.5*(1.0-cfg_->hcp.roadmap_graph_area_length_scale)*start_goal_dist*diff.normalized() - 0.5*area_width*normal; // 原点的左下角
  else
    area_origin = start.position() - 0.5*area_width*normal; // 原点的左下角

  // 插入顶点
  HcGraphVertexType start_vtx = boost::add_vertex(graph_); // 开始顶点
  graph_[start_vtx].pos = start.position();
  diff.normalize(); // 原地标准化


  // 开始采样
  for (int i=0; i < cfg_->hcp.roadmap_graph_no_samples; ++i)
  {
    Eigen::Vector2d sample;
    // 采样坐标
    sample = area_origin + rot_phi*Eigen::Vector2d(distribution_x(rnd_generator_), distribution_y(rnd_generator_));

    // 添加新的顶点
    HcGraphVertexType v = boost::add_vertex(graph_);
    graph_[v].pos = sample;
  }

  // 现在添加目标顶点
  HcGraphVertexType goal_vtx = boost::add_vertex(graph_); // 目标顶点
  graph_[goal_vtx].pos = goal2.position();


  // 插入边
  HcGraphVertexIterator it_i, end_i, it_j, end_j;
  for (boost::tie(it_i,end_i) = boost::vertices(graph_); it_i!=boost::prior(end_i); ++it_i) // 在这个循环中忽略目标
  {
    for (boost::tie(it_j,end_j) = boost::vertices(graph_); it_j!=end_j; ++it_j) // 检查所有前向连接
    {
      if (it_i==it_j) // 找到相同的顶点
        continue;

      Eigen::Vector2d distij = graph_[*it_j].pos-graph_[*it_i].pos;
      distij.normalize(); // 原地标准化

      // 检查方向是否向后:
      if (distij.dot(diff)<=obstacle_heading_threshold)
          continue; // diff已经标准化


      // 碰撞检查
      bool collision = false;
      for (ObstContainer::const_iterator it_obst = hcp_->obstacles()->begin(); it_obst != hcp_->obstacles()->end(); ++it_obst)
      {
        if ( (*it_obst)->checkLineIntersection(graph_[*it_i].pos,graph_[*it_j].pos, dist_to_obst) )
        {
          collision = true;
          break;
        }
      }
      if (collision)
        continue;

      // 创建边
      boost::add_edge(*it_i,*it_j,graph_);
    }
  }

  /// 在开始和目标之间找到所有路径！
  std::vector<HcGraphVertexType> visited;
  visited.push_back(start_vtx);
  DepthFirst(graph_,visited,goal_vtx, start.theta(), goal2.theta(), start_velocity, free_goal_vel);
}

} // end namespace
