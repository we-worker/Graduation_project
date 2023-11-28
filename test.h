



/**
 * @class EdgeVelocityHolonomic
 * @brief 定义了限制根据x，y和theta的平移和旋转速度的成本函数的边缘。
 * 
 * 该边缘依赖于三个顶点 \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ 并最小化: \n
 * \f$ \min \textrm{penaltyInterval}( [vx,vy,omega]^T ) \cdot weight \f$. \n
 * \e vx 表示相对于x轴的平移速度（使用有限差分计算）。 \n
 * \e vy 表示相对于y轴的平移速度（使用有限差分计算）。 \n
 * \e omega 使用两个偏航角的差商计算，然后归一化到[-pi, pi]。 \n
 * \e weight 可以使用setInformation()设置。 \n
 * \e penaltyInterval 表示惩罚函数，参见 penaltyBoundToInterval()。 \n
 * 错误/成本向量的维度为3：第一组件表示相对于x轴的平移速度，
 * 第二个组件表示相对于y轴的平移速度，第三个组件表示旋转速度。
 * @see TebOptimalPlanner::AddEdgesVelocity
 * @remarks 不要忘记调用setTebConfig()
 */  
class EdgeVelocityHolonomic : public BaseTebMultiEdge<3, double>
{
public:
  
  /**
   * @brief 构造边。
   */       
  EdgeVelocityHolonomic()
  {
    this->resize(3); // 由于我们从g2o::BaseMultiEdge派生，设置所需的顶点数量
  }
  
  /**
   * @brief 实际的成本函数
   */  
  /*
  GitHub Copilot: 这段代码是一个名为`computeError1`的函数，它是在全向轮机器人的运动模型中计算速度误差的部分。这个函数主要用于确保机器人的速度在设定的最大速度限制内。
首先，函数从三个顶点中获取两个位姿（`conf1`和`conf2`）和一个时间差（`deltaT`）。然后，计算两个位姿之间的位置差（`deltaS`）。
接着，函数计算`conf1`的角度的余弦和正弦值，然后使用这些值将`conf2`的位置转换到`conf1`的机器人框架中。这是通过一个逆2D旋转矩阵完成的，得到的结果是`r_dx`和`r_dy`。
然后，函数计算线速度（`vx`和`vy`）和角速度（`omega`）。线速度是位置差除以时间差，角速度是角度差除以时间差。
接下来，函数计算剩余的最大线速度（`max_vel_trans_remaining_x`和`max_vel_trans_remaining_y`）。这是通过从最大线速度的平方中减去当前线速度的平方，然后取平方根得到的。

  
  
  */
  void computeError1()//全向轮
  {
    ROS_ASSERT_MSG(cfg_, "必须在EdgeVelocityHolonomic()上调用setTebConfig");
    //首先，函数从三个顶点中获取两个位姿（`conf1`和`conf2`）和一个时间差（`deltaT`）。然后，计算两个位姿之间的位置差（`deltaS`）。
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
    Eigen::Vector2d deltaS = conf2->position() - conf1->position();

    //接着，函数计算`conf1`的角度的余弦和正弦值，然后使用这些值将`conf2`的位置转换到`conf1`的机器人框架中。这是通过一个逆2D旋转矩阵完成的，得到的结果是`r_dx`和`r_dy`。
    double cos_theta1 = std::cos(conf1->theta());
    double sin_theta1 = std::sin(conf1->theta()); 
    
    // 将conf2转换为当前机器人框架conf1（逆2d旋转矩阵）
    double r_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
    double r_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();
    
    //然后，函数计算线速度（`vx`和`vy`）和角速度（`omega`）。线速度是位置差除以时间差，角速度是角度差除以时间差。
    double vx = r_dx / deltaT->estimate();
    double vy = r_dy / deltaT->estimate();
    double omega = g2o::normalize_theta(conf2->theta() - conf1->theta()) / deltaT->estimate();

    //接下来，函数计算剩余的最大线速度（`max_vel_trans_remaining_x`和`max_vel_trans_remaining_y`）。
    //这是通过从最大线速度的平方中减去当前线速度的平方，然后取平方根得到的。
    double max_vel_trans_remaining_y;
    double max_vel_trans_remaining_x;
    max_vel_trans_remaining_y = std::sqrt(std::max(0.0, cfg_->robot.max_vel_trans * cfg_->robot.max_vel_trans - vx * vx)); 
    max_vel_trans_remaining_x = std::sqrt(std::max(0.0, cfg_->robot.max_vel_trans * cfg_->robot.max_vel_trans - vy * vy)); 

    //然后，函数计算最大的x和y方向的速度，这是通过取剩余最大线速度和设定的最大速度中的较小值得到的。
    double max_vel_y = std::min(max_vel_trans_remaining_y, cfg_->robot.max_vel_y);
    double max_vel_x = std::min(max_vel_trans_remaining_x, cfg_->robot.max_vel_x);
    double max_vel_x_backwards = std::min(max_vel_trans_remaining_x, cfg_->robot.max_vel_x_backwards);

    // 我们不对全向机器人的线速度应用惩罚epsilon，因为vx或yy可能接近零
    //最后，函数计算速度误差（`_error`），这是通过将当前速度限制在最大速度范围内得到的。如果速度超出最大速度，将会应用一个惩罚值。函数最后检查计算出的误差是否是有限的，如果不是，将会抛出一个错误。
    _error[0] = penaltyBoundToInterval(vx, -max_vel_x_backwards, max_vel_x, 0.0);
    _error[1] = penaltyBoundToInterval(vy, max_vel_y, 0.0);
    _error[2] = penaltyBoundToInterval(omega, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]) && std::isfinite(_error[2]),
                   "EdgeVelocityHolonomic::computeError() _error[0]=%f _error[1]=%f _error[2]=%f\n",_error[0],_error[1],_error[2]);
  }
// 非全向轮的速度误差计算
void computeError()
{
  // 确保已经调用setTebConfig设置了配置
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocity()");
  
  // 获取两个位姿和时间差
  const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
  
  // 计算两个位姿之间的位置差
  const Eigen::Vector2d deltaS = conf2->estimate().position() - conf1->estimate().position();
  
  // 计算位置差的范数（即距离）
  double dist = deltaS.norm();
  
  // 计算两个位姿之间的角度差
  const double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
  
  // 如果配置中设置了使用精确的弧长，并且角度差不为0// 使用实际弧长，而不是两点的直线距离
  if (cfg_->trajectory.exact_arc_length && angle_diff != 0)
  {
    // 计算弧长
    double radius =  dist/(2*sin(angle_diff/2));
    dist = fabs( angle_diff * radius ); // actual arg length!
  }
  
  // 计算速度（距离除以时间）// 线速度
  double vel = dist / deltaT->estimate();
  
  // 考虑方向，使用sigmoid函数进行平滑处理
  vel *= fast_sigmoid( 100 * (deltaS.x()*cos(conf1->theta()) + deltaS.y()*sin(conf1->theta())) ); // consider direction
  
  // 计算角速度（角度差除以时间）
  const double omega = angle_diff / deltaT->estimate();
  
  // 计算速度误差，将速度限制在最大速度范围内，如果超出范围，将会应用一个惩罚值// 线速度需要限制在最大和最大倒退的范围内
  _error[0] = penaltyBoundToInterval(vel, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(omega, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);

  // 检查计算出的误差是否是有限的，如果不是，将会抛出一个错误
  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeVelocity::computeError() _error[0]=%f _error[1]=%f\n",_error[0],_error[1]);
}

public:

EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};



    void computeError()
    {

      // 正向驱动约束
      Eigen::Vector2d angle_vec ( cos(conf1->theta()), sin(conf1->theta()) );	   
      _error[1] = penaltyBoundFromBelow(deltaS.dot(angle_vec), 0,0);
      // epsilon=0, 否则会将第一个带点推离起点

      // 确保_error[0]和_error[1]是有限的
      ROS_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]), "EdgeKinematicsDiffDrive::computeError() _error[0]=%f _error[1]=%f\n",_error[0],_error[1]);
    }
  /**
   * @brief 实际的成本函数
   */    
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "你必须在EdgeKinematicsCarlike()上调用setTebConfig");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    
    Eigen::Vector2d deltaS = conf2->position() - conf1->position();

    // 非全向约束
    _error[0] = fabs( ( cos(conf1->theta())+cos(conf2->theta()) ) * deltaS[1] - ( sin(conf1->theta())+sin(conf2->theta()) ) * deltaS[0] );

    // 限制最小转弯半径
    double angle_diff = g2o::normalize_theta( conf2->theta() - conf1->theta() );
    if (angle_diff == 0)
      _error[1] = 0; // 直线运动
    else if (cfg_->trajectory.exact_arc_length) // 使用半径的精确计算，计算弧长的公式。
      _error[1] = penaltyBoundFromBelow(fabs(deltaS.norm()/(2*sin(angle_diff/2))), cfg_->robot.min_turning_radius, 0.0);
    else
      _error[1] = penaltyBoundFromBelow(deltaS.norm() / fabs(angle_diff), cfg_->robot.min_turning_radius, 0.0); 
    // 这个边缘不受epsilon参数的影响，用户可能会在min_turning_radius参数上添加额外的边距。
    
    ROS_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]), "EdgeKinematicsCarlike::computeError() _error[0]=%f _error[1]=%f\n",_error[0],_error[1]);
  }