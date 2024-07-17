#pragma once
#include <vector>
#include "algorithm.h"
#include "Node_EP.h"


class Robot;
class MCTS;

class Planning {
public:
	std::vector<PResult> mcts_planning(std::vector<Robot>& robots, taskDict& newTaskDict, std::vector<int>& partition, 
						std::vector<int>& opBots, int iter, int step, bool continue_plan, bool full_replan, MCTS& planner);

	std::vector<int> mdutec(int instance, std::vector<int>& tasks, Robot& robot, std::vector<std::vector<int>> &prec, std::vector<std::vector<double>> & rzLoc);

private:
	void assignTasks(taskDict& tasksAssigned, Eigen::Vector2d& loc, Robot& robot, int zero) {
		for (int i = 0; i < robot.taskQueue.size(); i++) {
			tasksAssigned.one.push_back(loc.x());
			tasksAssigned.two.push_back(loc.y());
			if (zero == 0) {
				tasksAssigned.three.push_back(0);
			}
			else {
				tasksAssigned.three.push_back(robot.taskQueue[i]);
			}

			tasksAssigned.four.push_back(-1);
			tasksAssigned.five.push_back(0);

		}
	}

	void assignTasks1(taskDict& tasksAssigned, Eigen::Vector2d& loc, Robot& robot) {
		
			tasksAssigned.one.push_back(loc.x());
			tasksAssigned.two.push_back(loc.y());
			tasksAssigned.three.push_back(-1);
			tasksAssigned.four.push_back(-1);
			tasksAssigned.five.push_back(0);
	}

	void assignTasks(taskDict& tasksAssigned, Robot& robot) {
		for (int i = 0; i < robot.taskQueue.size(); i++) {
			tasksAssigned.one.push_back(robot.globalTaskLocs[robot.taskQueue[i]].x());
			tasksAssigned.two.push_back(robot.globalTaskLocs[robot.taskQueue[i]].y());
			tasksAssigned.three.push_back(robot.taskQueue[i]);
			tasksAssigned.four.push_back(-1);
			tasksAssigned.five.push_back(0);
		}

	}

	void assignTasksCondition(taskDict& tasksAssigned, std::vector<int>& partition, taskDict& newTaskDict, int opBotsr) {
		for (int i = 0; i < partition.size(); i++) {
			if (partition[i] == opBotsr) {
				tasksAssigned.one.push_back(newTaskDict.one[i]);
				tasksAssigned.two.push_back(newTaskDict.two[i]);
				tasksAssigned.three.push_back(newTaskDict.three[i]);
				tasksAssigned.four.push_back(newTaskDict.four[i]);
				tasksAssigned.five.push_back(newTaskDict.five[i]);
			}
		}
	}
};

