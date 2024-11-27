// #pragma once
#include <vector>
#include <unordered_set>
#include "../include/Node_EP.h"
// #include "../include/robot.h"
// #include "algo.h"
#include "../include/structures.h"

class Robot;
class MCTS {
public:
	MCTS();
	PResult MCTS_MTSP_EP(std::shared_ptr<Node_EP>& root_ptr, std::vector<int>& city_names, int iterations,
		double exploration_param, taskDict& tasksAssigned, double time, Robot& robot);
	double compute_tour_reward(std::vector<int>& complete_tour, taskDict& tasks, double time, Robot& robot, std::vector<Pose>& tour_states);

private:
	int iter;
	//for debugging only
	std::vector<std::vector<int>> remaining_cities_matlab;
	void read_remaining_cities(std::string& filename);
	/*---------*/
	std::shared_ptr<Node_EP> traverse(std::shared_ptr<Node_EP> & node, std::vector<int>& all_cities, double exploration_param);
	std::shared_ptr<Node_EP> expand(std::shared_ptr<Node_EP>& node, std::vector<int>& all_cities);
	double rollout(std::shared_ptr<Node_EP>& node, std::vector<int>& all_cities, taskDict& tasks, double time, Robot& robot);
	void backpropagate(std::shared_ptr<Node_EP> node, double reward);

	std::pair<double, double> policy_reward(Robot& robot, Pose& next_loc, int taskB, double time, double total_time, taskDict& tasks);
	Pose findIntersectPoint(Robot& robot, Robot& pB, Pose locA, std::vector<Eigen::Vector2d>& taskLocsB, double start_time, double total_time);

	// copied from  //-- Algorithm.h --\\

	double calculateNorm(const Pose& pose1, const Pose& pose2) {

		// Calculate the difference between corresponding elements
		double dx = pose1.x - pose2.x;
		double dy = pose1.y - pose2.y;

		// Calculate and return the Euclidean norm (L2 norm)
		return sqrt(dx * dx + dy * dy);
	}

	double calculateNorm(const Pose& pose1, const Eigen::Vector2d& pose2) {

		// Calculate the difference between corresponding elements
		double dx = pose1.x - pose2.x();
		double dy = pose1.y - pose2.y();

		// Calculate and return the Euclidean norm (L2 norm)
		return sqrt(dx * dx + dy * dy);
	}

	double calculateNorm(const Eigen::Vector2d& pose1, const Eigen::Vector2d& pose2) {

		// Calculate the difference between corresponding elements
		double dx = pose1.x() - pose2.x();
		double dy = pose1.y() - pose2.y();

		// Calculate and return the Euclidean norm (L2 norm)
		return sqrt(dx * dx + dy * dy);
	}

	std::vector<int> my_setdiff(std::vector<int> X, std::vector<int> Y) {

		std::unordered_set<int> set_Y(Y.begin(), Y.end());
		std::vector<int> Z;

		// Loop over elements in X and add those not in Y to the result
		for (int x : X) {
			if (set_Y.find(x) == set_Y.end()) {
				Z.push_back(x);
			}
		}

		return Z;
	}
};