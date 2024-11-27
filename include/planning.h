#pragma once
#include <vector>
// #include "../include/algorithm.h"
#include "../include/Node_EP.h"
#include "../include/structures.h"
// #include "../include/robot.h"

class Algo;
class Robot;
class MCTS;

class Planning {
public:
	std::vector<PResult> mcts_planning(std::vector<Robot>& robots, taskDict& newTaskDict, std::vector<int>& partition, 
						std::vector<int>& opBots, int iter, int step, bool continue_plan, bool full_replan, MCTS& planner);

	std::vector<int> mdutec(int instance, std::vector<int>& tasks, Robot& robot, std::vector<std::vector<int>> &prec, std::vector<std::vector<double>> & rzLoc);

private:
	void assignTasks(taskDict& tasksAssigned, Eigen::Vector2d& loc, Robot& robot, int zero);

	void assignTasks1(taskDict& tasksAssigned, Eigen::Vector2d& loc, Robot& robot);

	void assignTasks(taskDict& tasksAssigned, Robot& robot);

	void assignTasksCondition(taskDict& tasksAssigned, std::vector<int>& partition, taskDict& newTaskDict, int opBotsr);
};

