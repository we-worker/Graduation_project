#ifndef MMT_CONTROL_H
#define MMT_CONTROL_H

// MultiModalTEB 多模式TEB代码缩写 MMT
#include <Eigen/Core>
extern int MMT_no_infeasible_plans_;  // 声明全局变量
extern bool MMT_at_xy_terget;	//是否已经到达目标
extern double MMT_min_turning_radius;
extern bool MMT_use_Carlike;//这次仿真是否使用Carlike模型
extern bool MMT_first_judge_Carlike;//是否第一次判断Carlike模型

#endif