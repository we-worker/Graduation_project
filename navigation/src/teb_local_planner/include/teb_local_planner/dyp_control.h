#ifndef DYP_CONTROL_H
#define DYP_CONTROL_H

#include <Eigen/Core>

extern int dyp_no_infeasible_plans_;  // 声明全局变量
extern bool at_xy_terget;	//是否已经到达目标
extern double dyp_min_turning_radius;
extern bool dyp_use_Carlike;//这次仿真是否使用Carlike模型
extern bool dyp_first_judge_Carlike;//是否第一次判断Carlike模型

#endif