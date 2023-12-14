/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
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
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef _EDGE_KINEMATICS_H
#define _EDGE_KINEMATICS_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/teb_config.h>

#include <cmath>

namespace teb_local_planner
{

  /**
   * @class EdgeKinematicsDiffDrive
   * @brief Edge defining the cost function for satisfying the non-holonomic kinematics of a differential drive mobile robot.
   *
   * The edge depends on two vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1} \f$ and minimizes a geometric interpretation
   * of the non-holonomic constraint:
   * 	- C. Rösmann et al.: Trajectory modification considering dynamic constraints of autonomous robots, ROBOTIK, 2012.
   *
   * The \e weight can be set using setInformation(): Matrix element 1,1: (Choose a very high value: ~1000). \n
   * A second equation is implemented to penalize backward motions (second element of the error /cost vector). \n
   * The \e weight can be set using setInformation(): Matrix element 2,2: (A value ~1 allows backward driving, but penalizes it slighly). \n
   * The dimension of the error / cost vector is 2: the first component represents the nonholonomic constraint cost,
   * the second one backward-drive cost.
   * @see TebOptimalPlanner::AddEdgesKinematics, EdgeKinematicsCarlike
   * @remarks Do not forget to call setTebConfig()
   */
  class EdgeKinematicsDiffDrive : public BaseTebBinaryEdge<2, double, VertexPose, VertexPose>
  {
  public:
    /**
     * @brief Construct edge.
     */
    EdgeKinematicsDiffDrive()
    {
      this->setMeasurement(0.);
    }

    /**
     * @brief Actual cost function
     */
    void computeError()
    {
      // 确保已经调用了setTebConfig
      ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeKinematicsDiffDrive()");
      // 获取两个顶点的位置
      const VertexPose *conf1 = static_cast<const VertexPose *>(_vertices[0]);
      const VertexPose *conf2 = static_cast<const VertexPose *>(_vertices[1]);

      // 计算两个顶点位置的差值
      Eigen::Vector2d deltaS = conf2->position() - conf1->position();

      // 非全向约束
      _error[0] = fabs((cos(conf1->theta()) + cos(conf2->theta())) * deltaS[1] - (sin(conf1->theta()) + sin(conf2->theta())) * deltaS[0]);

      // 正向驱动约束
      Eigen::Vector2d angle_vec(cos(conf1->theta()), sin(conf1->theta()));
      _error[1] = penaltyBoundFromBelow(deltaS.dot(angle_vec), 0, 0);
      // epsilon=0, 否则会将第一个带点推离起点

      // 确保_error[0]和_error[1]是有限的
      ROS_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]), "EdgeKinematicsDiffDrive::computeError() _error[0]=%f _error[1]=%f\n", _error[0], _error[1]);
    }

#ifdef USE_ANALYTIC_JACOBI
#if 1
    /**
     * @brief 计算在computeError()中指定的成本函数的雅可比矩阵。
     */
    void linearizeOplus()
    {
      // 确保已经调用了setTebConfig
      ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeKinematicsDiffDrive()");
      // 获取两个顶点的位置
      const VertexPose *conf1 = static_cast<const VertexPose *>(_vertices[0]);
      const VertexPose *conf2 = static_cast<const VertexPose *>(_vertices[1]);

      // 计算两个顶点位置的差值
      Eigen::Vector2d deltaS = conf2->position() - conf1->position();

      // 计算各个角度的正弦和余弦值
      double cos1 = cos(conf1->theta());
      double cos2 = cos(conf2->theta());
      double sin1 = sin(conf1->theta());
      double sin2 = sin(conf2->theta());
      double aux1 = sin1 + sin2;
      double aux2 = cos1 + cos2;

      // 计算误差的导数
      double dd_error_1 = deltaS[0] * cos1;
      double dd_error_2 = deltaS[1] * sin1;
      double dd_dev = penaltyBoundFromBelowDerivative(dd_error_1 + dd_error_2, 0, 0);

      // 计算非全向约束的符号
      double dev_nh_abs = g2o::sign((cos(conf1->theta()) + cos(conf2->theta())) * deltaS[1] -
                                    (sin(conf1->theta()) + sin(conf2->theta())) * deltaS[0]);

      // conf1的雅可比矩阵
      _jacobianOplusXi(0, 0) = aux1 * dev_nh_abs;                               // nh x1
      _jacobianOplusXi(0, 1) = -aux2 * dev_nh_abs;                              // nh y1
      _jacobianOplusXi(1, 0) = -cos1 * dd_dev;                                  // drive-dir x1
      _jacobianOplusXi(1, 1) = -sin1 * dd_dev;                                  // drive-dir y1
      _jacobianOplusXi(0, 2) = (-dd_error_2 - dd_error_1) * dev_nh_abs;         // nh angle
      _jacobianOplusXi(1, 2) = (-sin1 * deltaS[0] + cos1 * deltaS[1]) * dd_dev; // drive-dir angle1

      // conf2的雅可比矩阵
      _jacobianOplusXj(0, 0) = -aux1 * dev_nh_abs;                                  // nh x2
      _jacobianOplusXj(0, 1) = aux2 * dev_nh_abs;                                   // nh y2
      _jacobianOplusXj(1, 0) = cos1 * dd_dev;                                       // drive-dir x2
      _jacobianOplusXj(1, 1) = sin1 * dd_dev;                                       // drive-dir y2
      _jacobianOplusXj(0, 2) = (-sin2 * deltaS[1] - cos2 * deltaS[0]) * dev_nh_abs; // nh angle
      _jacobianOplusXj(1, 2) = 0;                                                   // drive-dir angle1
    }
#endif
#endif

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /**
   * @class EdgeKinematicsCarlike
   * @brief 定义了满足类似汽车移动机器人的非全向运动学的成本函数的边缘。
   *
   * 这个边缘依赖于两个顶点 \f$ \mathbf{s}_i, \mathbf{s}_{ip1} \f$ 并最小化非全向约束的几何解释：
   *  - C. Rösmann等：考虑自动机器人的动态约束的轨迹修改，ROBOTIK，2012。
   *
   * 这个定义与差分驱动机器人的定义相同。
   * 此外，这个边缘包含了类似汽车的机器人所需的最小转弯半径。
   * 转弯半径由 \f$ r=v/omega \f$ 定义。
   *
   * 可以使用setInformation()设置 \e weight：矩阵元素1,1：（选择一个非常高的值：~1000）。\n
   * 第二个方程强制执行最小转弯半径。
   * 可以使用setInformation()设置 \e weight：矩阵元素2,2。 \n
   * 错误/成本向量的维度为3：第一个组件代表非全向约束成本，
   * 第二个是后向驱动成本，第三个是最小转弯半径
   * @see TebOptimalPlanner::AddEdgesKinematics, EdgeKinematicsDiffDrive
   * @remarks 从下面限制转弯半径不受penalty_epsilon参数的影响，
   *          用户可能会在min_turning_radius参数上添加额外的边距。
   * @remarks 不要忘记调用setTebConfig()
   */
  class EdgeKinematicsCarlike : public BaseTebBinaryEdge<2, double, VertexPose, VertexPose>
  {
  public:
    /**
     * @brief 构造边缘。
     */
    EdgeKinematicsCarlike()
    {
      this->setMeasurement(0.);
    }

    /**
     * @brief 实际的成本函数
     */
    void computeError()
    {
      ROS_ASSERT_MSG(cfg_, "你必须在EdgeKinematicsCarlike()上调用setTebConfig");
      const VertexPose *conf1 = static_cast<const VertexPose *>(_vertices[0]);
      const VertexPose *conf2 = static_cast<const VertexPose *>(_vertices[1]);

      Eigen::Vector2d deltaS = conf2->position() - conf1->position();

      // 非全向约束
      _error[0] = fabs((cos(conf1->theta()) + cos(conf2->theta())) * deltaS[1] - (sin(conf1->theta()) + sin(conf2->theta())) * deltaS[0]);

      // 限制最小转弯半径
      double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
      if (angle_diff == 0)
        _error[1] = 0;                            // 直线运动
      else if (cfg_->trajectory.exact_arc_length) // 使用半径的精确计算
        _error[1] = penaltyBoundFromBelow(fabs(deltaS.norm() / (2 * sin(angle_diff / 2))), cfg_->robot.min_turning_radius, 0.0);
      else
        _error[1] = penaltyBoundFromBelow(deltaS.norm() / fabs(angle_diff), cfg_->robot.min_turning_radius, 0.0);
      // 这个边缘不受epsilon参数的影响，用户可能会在min_turning_radius参数上添加额外的边距。

      ROS_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]), "EdgeKinematicsCarlike::computeError() _error[0]=%f _error[1]=%f\n", _error[0], _error[1]);
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  class EdgeKinematicsFourWheeled : public BaseTebBinaryEdge<5, double, VertexPose, VertexPose>
  {
  public:
    EdgeKinematicsFourWheeled()
    {
      this->setMeasurement(0.);
    }


    //TODO 修改使得，仅有角速度存在时，angle=-90°
    double calculateAngle(double vx, double vy,double angle_v)
    {
      double angle = std::atan2(vy, vx) * 180 / M_PI;
      if (angle > 90)
      {
        angle -= 180;
      }
      if (angle < -90)
      {
        angle += 180;
      }

      if(std::fabs(angle_v)>=0.05 && std::fabs(vx)<=0.1  && std::fabs(vy)<=0.1){
        angle=180;
      }

      return angle;
    }

    void computeError()
    {
      ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeKinematicsFourWheeled()");
      const VertexPose *conf1 = static_cast<const VertexPose *>(_vertices[0]);
      const VertexPose *conf2 = static_cast<const VertexPose *>(_vertices[1]);
      const VertexPose *conf_old = static_cast<const VertexPose *>(_vertices[2]);
      //参考加速度部分
      // const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
      // const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
      // const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

      Eigen::Vector2d deltaS = conf2->position() - conf1->position();
      Eigen::Vector2d last_deltaS = conf1->position() - conf_old->position();
      double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
      double angle_diff_old = g2o::normalize_theta(conf1->theta() - conf_old->theta());

      // 非全向约束
      _error[0] = fabs((cos(conf1->theta()) + cos(conf2->theta())) * deltaS[1] - (sin(conf1->theta()) + sin(conf2->theta())) * deltaS[0]);

      // linear.y 变换要连续，不能突变
      // if (fabs(deltaS[1] - last_deltaS[1]) > 0) {
      //   _error[2] = fabs(deltaS[1] - last_deltaS[1])*fabs(deltaS[1] - last_deltaS[1]);
      // } else {
      //   _error[2] = 0;
      // }

      // 转为机器人坐标系下的角度
      double cos_theta1 = std::cos(conf1->theta());
      double sin_theta1 = std::sin(conf1->theta()); 
      // 将conf2转换为当前机器人框架conf1（逆2d旋转矩阵）
      double r_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
      double r_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();

      // 转为机器人坐标系下的角度
      double cos_theta_old = std::cos(conf1->theta());
      double sin_theta_old = std::sin(conf1->theta()); 
      // 将conf2转换为当前机器人框架conf1（逆2d旋转矩阵）
      double r_dx_old =  cos_theta_old*deltaS.x() + sin_theta_old*deltaS.y();
      double r_dy_old = -sin_theta_old*deltaS.x() + cos_theta_old*deltaS.y();

      double angle_deltaS = calculateAngle(r_dx, r_dy,angle_diff);
      double angle_last_deltaS = calculateAngle(r_dx_old,r_dy_old,angle_diff_old);
      double angle_diff2 = angle_deltaS - angle_last_deltaS;


      // 线速度和角速度不能同时存在
      if (fabs(r_dy) > 0 && fabs(angle_diff) > 0)
      {
        _error[1] = r_dy*r_dy;
      }
      else
      {
        _error[1] = 0;
      }


      // 判定上一时刻如果x变化为0，而角速度存在时，此时如果角速度存在 为零，误差等于此时刻x
      if (fabs(r_dx_old) < fabs(angle_last_deltaS) && angle_last_deltaS==0 && angle_deltaS==0 )
      {
        _error[2] = r_dx*r_dx;
      }
      else
      {
        _error[2] = 0;
      }

      // 线速度斜移角度变化要连续
      if (r_dx==0 || r_dy == 0 || r_dx_old== 0 || r_dy_old== 0)
      {
        _error[3] = 0;
      }
      else
      {
        _error[3] = angle_diff2*angle_diff2;
      }

      // 正向驱动约束
      Eigen::Vector2d angle_vec(cos(conf1->theta()), sin(conf1->theta()));
      _error[4] = penaltyBoundFromBelow(deltaS.dot(angle_vec), 0, 0);
      // 角速度angular.z 大于linear.x 时，linear.x只能为0,需要优化
      // if (deltaS[0]!=0 && fabs(angle_diff) > fabs(deltaS[0])) {
      //   _error[4] = 1;//(deltaS[0]*deltaS[0])
      // } else {
      //   _error[4] = 0;
      // }

      ROS_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]) && std::isfinite(_error[2]) && std::isfinite(_error[3]), "EdgeKinematicsFourWheeled::computeError() _error[0]=%f _error[1]=%f _error[2]=%f _error[3]=%f\n", _error[0], _error[1], _error[2], _error[3]);
      // ROS_INFO("EdgeKinematicsFourWheeled::computeError() _error[0]=%f _error[1]=%f _error[2]=%f _error[3],angle_diff2=%f\n",_error[0],_error[1],_error[2],_error[3]);
      //变化似乎不连续
        // ROS_INFO("conf1=%f,conf1=%f,count_111=%d\n",conf1->position()[0],conf1->position()[1], count_111);

      // if (_error[3] * 100 > 1)
      //  std::cout << "EdgeKinematicsFourWheeled::computeError()  angle_deltaS=" << angle_deltaS << ",angle_last=" << angle_last_deltaS << ",angle_diff2=" << _error[3] * 100 << std::endl;
     // if (deltaS!=last_deltaS)
      // {
      //   /* code */
      //   last_deltaS = deltaS;
      // }
      // count_111++;
      
    }



    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  };

} // end namespace

#endif