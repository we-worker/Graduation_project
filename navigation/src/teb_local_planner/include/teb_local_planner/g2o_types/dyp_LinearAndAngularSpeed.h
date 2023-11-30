
#ifndef LinearAndAngular_H_
#define LinearAndAngular_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include "g2o/core/base_unary_edge.h"



namespace teb_local_planner
{
	class EdgeNoSimultaneousLinearAndAngularSpeed : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose>
	{
	public:
		EdgeNoSimultaneousLinearAndAngularSpeed() 
		{
			_measurement = 0;
		}

		void computeError()
		{
			const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
			const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
			Eigen::Vector2d deltaS = conf2->position() - conf1->position();
			
			double cos_theta1 = std::cos(conf1->theta());
			double sin_theta1 = std::sin(conf1->theta()); 
			
			// 将conf2转换为当前机器人框架conf1（逆2d旋转矩阵）
			double r_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
			double linear_y_speed = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();

			double angular_speed = g2o::normalize_theta(conf2->theta() - conf1->theta());

			if (linear_y_speed != 0 && angular_speed != 0)
			{
				_error[0] = 1;
				// ROS_INFO("Simultaneous linear and angular speed detected");
			}
			else
			{
				_error[0] = 0;
			}

			ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeNoSimultaneousLinearAndAngularSpeed::computeError() _error[0]=%f\n",_error[0]);
		}

	public: 
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
} // end namespace


#endif