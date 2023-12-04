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







    阿卡曼车{
    // 限制最小转弯半径
    double angle_diff = g2o::normalize_theta( conf2->theta() - conf1->theta() );
    if (angle_diff == 0)
      _error[1] = 0; // 直线运动
    else if (cfg_->trajectory.exact_arc_length) // 使用半径的精确计算
      _error[1] = penaltyBoundFromBelow(fabs(deltaS.norm()/(2*sin(angle_diff/2))), cfg_->robot.min_turning_radius, 0.0);
    else
      _error[1] = penaltyBoundFromBelow(deltaS.norm() / fabs(angle_diff), cfg_->robot.min_turning_radius, 0.0); 
    }


      差分车{
            // 获取两个顶点的位置
      const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
      const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
      
      // 计算两个顶点位置的差值
      Eigen::Vector2d deltaS = conf2->position() - conf1->position();

      // 非全向约束
      _error[0] = fabs( ( cos(conf1->theta())+cos(conf2->theta()) ) * deltaS[1] - ( sin(conf1->theta())+sin(conf2->theta()) ) * deltaS[0] );

      // 正向驱动约束
      Eigen::Vector2d angle_vec ( cos(conf1->theta()), sin(conf1->theta()) );	   
      _error[1] = penaltyBoundFromBelow(deltaS.dot(angle_vec), 0,0);
      }

      四舵轮{
        只能使用conf1、2变量，下面提示的angular、linear变量都只是提示。

        要求角速度angular.z 与线速度linear.y 不能同时存在
        要求linear.y 变换要连续，不能突变
        当linear.y 存在时，linear.x 不能变化正负号。

        角速度angular.z 大于linear.x 时，linear.x只能为0
      }

      四舵轮{
        // 获取两个顶点的位置
        const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
        const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);

        // 计算两个顶点位置的差值
        Eigen::Vector2d deltaS = conf2->position() - conf1->position();

        // 计算角度差
        double angle_diff = g2o::normalize_theta( conf2->theta() - conf1->theta() );

        // 线速度和角速度不能同时存在
        if (fabs(deltaS[1]) > 0 && fabs(angle_diff) > 0) {
          _error[0] = 1;
        } else {
          _error[0] = 0;
        }

        // linear.y 变换要连续，不能突变
        if (fabs(deltaS[1] - last_deltaS[1]) > MAX_CHANGE) {
          _error[1] = 1;
        } else {
          _error[1] = 0;
        }

        // 当linear.y 存在时，linear.x 不能变化正负号
        if (deltaS[1] != 0 && sign(deltaS[0]) != sign(last_deltaS[0])) {
          _error[2] = 1;
        } else {
          _error[2] = 0;
        }

        // 角速度angular.z 大于linear.x 时，linear.x只能为0
        if (fabs(angle_diff) > fabs(deltaS[0])) {
          _error[3] = 1;
        } else {
          _error[3] = 0;
        }

        // 更新上一次的deltaS
        last_deltaS = deltaS;
      }




