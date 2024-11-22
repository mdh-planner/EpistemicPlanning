#include <numeric>
#include "planning.h"
#include "MCTS_MTSP_EP.h"
#include "BasicECTSP.h"

using namespace std;

vector<PResult> Planning::mcts_planning(std::vector<Robot>& robots, taskDict& newTaskDict, std::vector<int>& partition,
	std::vector<int>& opBots, int iterations, int step, bool continue_plan, bool full_replan, MCTS &planner) {

	double exploration_param = 500;
	vector<shared_ptr<Node_EP>> root;
	root.resize(opBots.size());
	vector<taskDict> tasksAssigned(opBots.size());
	vector<PResult> planningResults(opBots.size());
	
	//MCTS planner;
	for (int r = 0; r < opBots.size(); r++) {
		int rr = opBots[r];
		int numOldTasks = robots[rr].taskQueue.size();
		int start_city = 0;
		
		double time = robots[opBots[r]].dt * step;
		root[r] = make_shared<Node_EP>();

		Eigen::Vector2d tmp(robots[rr].pose.x, robots[rr].pose.y);
		assignTasks1(tasksAssigned[r], tmp, robots[rr]); // thisBot.pose(1:2) 0 0 0

		if (!continue_plan && !full_replan) {
			int maxVal = 0;

			for (int i = 0; i < partition.size(); i++) {
				if (partition[i] == opBots[r] && i > maxVal) {
					maxVal = i;
				}
			}

			if (maxVal < newTaskDict.one.size()) { // we assume that each column is the same size
				
				assignTasksCondition(tasksAssigned[r], partition, newTaskDict, opBots[r]); // newTaskDict(partition == opBots(r),:)
				assignTasks(tasksAssigned[r], robots[rr]); // [thisBot.globalTaskLocs(thisBot.taskQueue,:), ...
														   //thisBot.taskQueue', zeros(numOldTasks,2)]
				vector<int> city_names(tasksAssigned[r].one.size());
				iota(city_names.begin(), city_names.end(), 0);
				root[r]->initGraph(start_city, vector<int>(1, start_city), nullptr);
				
				planningResults[r] = planner.MCTS_MTSP_EP(root[r], city_names, iterations, exploration_param, tasksAssigned[r], time, robots[opBots[r]]);

			}
			else {
				
				assignTasks(tasksAssigned[r], robots[rr]); // [thisBot.globalTaskLocs(thisBot.taskQueue,:), ...
														   //thisBot.taskQueue', zeros(numOldTasks,2)]
				for (int i = 0; i < tasksAssigned[r].one.size(); i++) {
					planningResults[r].tour_states[i].x = tasksAssigned[r].one[i];
					planningResults[r].tour_states[i].y = tasksAssigned[r].two[i];
				}

				if (tasksAssigned[r].one.size() > 1) {
					iota(planningResults[r].best_tour.begin(), planningResults[r].best_tour.end(), 0);
				}
				else {
					planningResults[r].best_tour = { 0 };	
				}
				root[r]->initGraph(start_city, vector<int>(1, start_city), nullptr);
			}
		}
		else if (full_replan) {
			assignTasksCondition(tasksAssigned[r], partition, newTaskDict, opBots[r]);
			vector<int> city_names(tasksAssigned[r].one.size());
			iota(city_names.begin(), city_names.end(), 0);
			root[r]->initGraph(start_city, vector<int>(1, start_city), nullptr);
		

			planningResults[r] = planner.MCTS_MTSP_EP(root[r], city_names, iterations, exploration_param, tasksAssigned[r], time, robots[opBots[r]]);
			
		}
		else {
			vector<int> city_names(tasksAssigned[r].one.size());
			iota(city_names.begin(), city_names.end(), 0);
			//root[r] = robots[opBots[r]].root; //?

			//!!! MISSING HERE tasksAssigned{r} = [robots(opBots(r)).pose(1:2) 0 0 0; robots(opBots(r)).tasksAssigned];!!!
		
			planningResults[r] = planner.MCTS_MTSP_EP(root[r], city_names, iterations, exploration_param, tasksAssigned[r], time, robots[opBots[r]]);
		}
		planningResults[r].root = root;
		planningResults[r].tasksAssigned = tasksAssigned;
	}

	return planningResults;
}

vector<int> Planning::mdutec(int instance, vector<int> & tasks, Robot &robot, vector<vector<int>>& prec, vector<vector<double>> & rzLoc) {

	// process data for mdutec algo
	vector<pair<double, double>> taskLoc(tasks.size());
	
		for (int j = 0; j < tasks.size(); j++) {
			taskLoc[j].first = robot.globalTaskLocs[tasks[j]].x();
			taskLoc[j].second = robot.globalTaskLocs[tasks[j]].y();
		}
	
	vector<Pose> startLoc(1);
	//startLoc.push_back(robot.pose);

	if (!rzLoc.empty()) {
		startLoc[0].x = rzLoc[0][0];
		startLoc[0].x = rzLoc[0][1];
	}
	else {
		startLoc[0] = robot.pose;
	}
	vector<Pose> depots(robot.depot.size());
	for (int i = 0; i < robot.depot.size(); i++) {
		depots[i].x = robot.depot[i].x();
		depots[i].y = robot.depot[i].y();
	}
	vector<int> result = runAlgorithm(1, tasks, taskLoc, startLoc, depots, prec);

	return result;
}







