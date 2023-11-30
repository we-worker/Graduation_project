#ifndef EDGE_PREFER_ROTDIR_H_
#define EDGE_PREFER_ROTDIR_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include "g2o/core/base_unary_edge.h"


namespace teb_local_planner
{

/**
 * @class EdgePreferRotDir
 * @brief Edge defining the cost function for penalzing a specified turning direction, in particular left resp. right turns
 * 
 * The edge depends on two consecutive vertices \f$ \mathbf{s}_i, \mathbf{s}_{i+1} \f$ and penalizes a given rotation direction
 * based on the \e weight and \e dir (\f$ dir \in \{-1,1\} \f$)
 * \e dir should be +1 to prefer left rotations and -1 to prefer right rotations  \n
 * \e weight can be set using setInformation(). \n
 * @see TebOptimalPlanner::AddEdgePreferRotDir
 */     
class EdgePreferRotDir : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose>
{
public:
    
  /**
   * @brief Construct edge.
   */    
  EdgePreferRotDir() 
  {
    _measurement = 1;
  }
 
  /**
   * @brief Actual cost function
   */    
  void computeError()
  {
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    
    _error[0] = penaltyBoundFromBelow( _measurement*g2o::normalize_theta(conf2->theta()-conf1->theta()) , 0, 0);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgePreferRotDir::computeError() _error[0]=%f\n",_error[0]);
  }

  /**
   * @brief Specify the prefered direction of rotation
   * @param dir +1 to prefer the left side, -1 to prefer the right side
   */ 
  void setRotDir(double dir)
  {
    _measurement = dir;
  }
  
  /** Prefer rotations to the right */
  void preferRight() {_measurement = -1;}
    
  /** Prefer rotations to the right */
  void preferLeft() {_measurement = 1;}  
    
  
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
    

} // end namespace

#endif




void TebOptimalPlanner::AddEdgesPreferRotDir()
{
  //TODO(roesmann): Note, these edges can result in odd predictions, in particular
  //                we can observe a substantional mismatch between open- and closed-loop planning
  //                leading to a poor control performance.
  //                At the moment, we keep these functionality for oscillation recovery:
  //                Activating the edge for a short time period might not be crucial and
  //                could move the robot to a new oscillation-free state.
  //                This needs to be analyzed in more detail!
  if (prefer_rotdir_ == RotType::none || cfg_->optim.weight_prefer_rotdir==0)
    return; // if weight equals zero skip adding edges!

  if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left)
  {
    ROS_WARN("TebOptimalPlanner::AddEdgesPreferRotDir(): unsupported RotType selected. Skipping edge creation.");
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,1,1> information_rotdir;
  information_rotdir.fill(cfg_->optim.weight_prefer_rotdir);
  
  for (int i=0; i < teb_.sizePoses()-1 && i < 3; ++i) // currently: apply to first 3 rotations
  {
    EdgePreferRotDir* rotdir_edge = new EdgePreferRotDir;
    rotdir_edge->setVertex(0,teb_.PoseVertex(i));
    rotdir_edge->setVertex(1,teb_.PoseVertex(i+1));      
    rotdir_edge->setInformation(information_rotdir);
    
    if (prefer_rotdir_ == RotType::left)
        rotdir_edge->preferLeft();
    else if (prefer_rotdir_ == RotType::right)
        rotdir_edge->preferRight();
    
    optimizer_->addEdge(rotdir_edge);
  }
}













