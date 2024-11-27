#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <random>
#include <unordered_set>
#include <set>
#include "../include/MCTS_MTSP_EP.h"
#include "../include/randomGenerator.h"
#include "../include/robot.h"


using namespace std;

void read_and_remove_first_line(const std::string& filename, std::vector<int>& ds) {
	std::ifstream infile(filename);
	std::ofstream tempfile("tempfile.txt");
	std::string line;
	bool first_line_read = false;

	if (!infile.is_open() || !tempfile.is_open()) {
		std::cerr << "Unable to open file" << std::endl;
		return;
	}

	while (std::getline(infile, line)) {
		if (!first_line_read) {
			std::stringstream ss(line);
			int value;
			while (ss >> value && value > -1) {
				ds.push_back(value-1);
			}
			first_line_read = true;
		}
		else {
			tempfile << line << std::endl;
		}
	}

	infile.close();
	tempfile.close();

	// Replace original file with the temporary file
	std::remove(filename.c_str());
	std::rename("tempfile.txt", filename.c_str());
}

void MCTS::read_remaining_cities(string &filename) {
	std::ifstream inputFile(filename);
	
	std::string line;

	if (!inputFile.is_open()) {
		std::cerr << "Unable to open file" << std::endl;
	}

	while (std::getline(inputFile, line)) {
		std::istringstream iss(line);
		std::vector<int> tempVector;
		int num;
		bool emptyLine = true;

		while (iss >> num) {
			if (num != -1) {
				tempVector.push_back(num-1);
			}
			emptyLine = false;
		}

		if (emptyLine || (tempVector.size() == 1 && tempVector[0] == -1)) {
			remaining_cities_matlab.push_back({});
		}
		else {
			remaining_cities_matlab.push_back(tempVector);
		}
	}

	inputFile.close();
}

void read_and_remove_first_number(const std::string& filename, int& rnd) {
	std::ifstream infile(filename);
	std::ofstream tempfile("tempfile.txt");
	bool first_number_read = false;
	std::string line;

	if (!infile.is_open() || !tempfile.is_open()) {
		std::cerr << "Unable to open file" << std::endl;
		return;
	}

	while (std::getline(infile, line)) {
		std::istringstream iss(line);
		int number;
		while (iss >> number) {
			if (!first_number_read) {
				rnd = number;
				first_number_read = true;
			}
			else {
				tempfile << number << " ";
			}
		}
		if (!first_number_read) {
			rnd = -1;  // Indicate that the file was empty or the number was not found
		}
		tempfile << "\n";  // Add newline to maintain file structure
	}

	infile.close();
	tempfile.close();

	// Replace the original file with the temporary file
	std::remove(filename.c_str());
	std::rename("tempfile.txt", filename.c_str());
}

void printNode(std::shared_ptr<Node_EP>& root_ptr) {
	cout << " ************************** Printing NODE ******************************" << endl;
	cout << root_ptr->city << "  |  size:" << root_ptr->children.size() << endl;
	for (auto ch : root_ptr->children) {
		cout << "city: " << ch->city << endl;
		cout << "tour: [" << flush;
		for (auto t : ch->tour) {
			cout << t << " " << flush;
		}
		cout << "]" << endl;
		cout << "tour: [" << flush;
		for (auto t : ch->visited) {
			cout << t << " " << flush;
		}
		cout << "]" << endl;
		cout << "visits: " << ch->visits << endl;
		std::cout << std::fixed << "value: " << ch->value << endl;
		cout << "-----------------------------------------" << endl;
	}
	cout << endl; cout << endl;
	cout << "======================================================" << endl;
}

MCTS::MCTS() {
	/*std::string filename = "C:\\Users\\bmc01\\Desktop\\SMC 2024\\glocal-main\\IROS24\\datasample2.txt";
	read_remaining_cities(filename);*/
}

PResult MCTS::MCTS_MTSP_EP(std::shared_ptr<Node_EP>& root_ptr, std::vector<int>& all_cities, int iterations,
	double exploration_param, taskDict& tasksAssigned, double time, Robot& robot){
	
	cout << "Replanning!" << endl;
	for (iter = 0; iter < iterations; iter++) {
		/*if ((iter + 1) % 1 == 0) {
			cout << "MCTS Iteration: " << iter << endl;
		}*/
		exploration_param *= 0.999;
		auto leaf = traverse(root_ptr, all_cities, exploration_param);
		auto reward = rollout(leaf, all_cities, tasksAssigned, time, robot);
		backpropagate(leaf, reward);
		//printNode(root_ptr);
	}
	// Extracting the best tour after MCTS
	PResult _res;
 	_res.best_tour.push_back(root_ptr->city);
	while (!root_ptr->children.empty()) {
		root_ptr = root_ptr->best_child(0);
		_res.best_tour.push_back(root_ptr->city);
	}

	compute_tour_reward(_res.best_tour, tasksAssigned, time, robot, _res.tour_states);

	return _res;
}

shared_ptr<Node_EP> MCTS::traverse(shared_ptr<Node_EP>& node, vector<int>& all_cities, double exploration_param) {
	shared_ptr<Node_EP> current_node = node; // Create a local copy of the shared pointer
	shared_ptr<Node_EP> leaf;

	while (!current_node->children.empty() || !current_node->is_fully_expanded(all_cities.size())) {
		if (!current_node->is_fully_expanded(all_cities.size())) {
			leaf = expand(current_node, all_cities);
			return leaf;
		}
		else {
			// Update current_node to its best child for traversal
			current_node = current_node->best_child(exploration_param);
		}
	}
	leaf = current_node;
	return leaf;
}

shared_ptr<Node_EP> MCTS::expand(shared_ptr<Node_EP>& node, vector<int>& all_cities) {
	
	// Prepare the vector to hold the results
	std::shared_ptr<Node_EP> child = std::make_shared<Node_EP>();  // Properly initialize the child pointer
	//shared_ptr<Node_EP> child;
	std::vector<int> unvisited_cities;
	std::unordered_set<int> visited_set(node->visited.begin(), node->visited.end());

	for (int city : all_cities) {
		if (visited_set.find(city) == visited_set.end()) {
			unvisited_cities.push_back(city);
		}
	}

	int rnd = getRandomIntegerInRange(0, (int)unvisited_cities.size() - 1);
	// 
	/*--------FOR TESTING-------------------------*/
	 // File name
	//std::string filename = "C:\\Users\\bmc01\\Desktop\\SMC 2024\\glocal-main\\IROS24\\rnd_value2.txt";

	//int rnd;

	//read_and_remove_first_number(filename, rnd);

	//rnd--;
	/*-------------------------------------------------*/
	int new_city = unvisited_cities[rnd];
	auto tmp = node->tour;
	tmp.push_back(new_city);
	child->initGraph(new_city, tmp, node);
	node->children.push_back(child);
	// Use std::set to find unique elements
	std::set<int> unique_elements(node->visited.begin(), node->visited.end());
	unique_elements.insert(node->tour.begin(), node->tour.end());
	unique_elements.insert(new_city);

	// Now convert the set back to a vector 
	node->visited.assign(unique_elements.begin(), unique_elements.end());

	return child;
}

double MCTS::rollout(shared_ptr<Node_EP>& node, vector<int>& all_cities, taskDict & tasks, double time, Robot &robot) {
	// Prepare the vector to hold the results
	std::vector<int> remaining_cities;
	std::unordered_set<int> visited_set(node->tour.begin(), node->tour.end());

	for (int city : all_cities) {
		if (visited_set.find(city) == visited_set.end()) {
			remaining_cities.push_back(city);
		}
	}

	vector<int> shuffled_remaining_cities, complete_tour;
	//getRandomIntVectorFromOtherVector(remaining_cities, shuffled_remaining_cities);
	/*----------FOR TESTING ------------------*/
	//std::string filename = "C:\\Users\\bmc01\\Desktop\\SMC 2024\\glocal-main\\IROS24\\datasample2.txt";
	//read_and_remove_first_line(filename, remaining_cities);
	/*----------------------------------------------*/
	/*remaining_cities = remaining_cities_matlab[0];
	remaining_cities_matlab.erase(remaining_cities_matlab.begin());*/

	complete_tour = node->tour;
	complete_tour.insert(complete_tour.end(), remaining_cities.begin(), remaining_cities.end()); // Append the shuffled cities
	vector<Pose> tour_states;
	double reward = compute_tour_reward(complete_tour, tasks, time, robot, tour_states);
	return -reward;
}

void MCTS::backpropagate(shared_ptr<Node_EP> node, double reward) {
	while (node) {
		node->visits++;
		node->value += reward;
		node = node->parent;
	}
}

double MCTS::compute_tour_reward(std::vector<int>& complete_tour, taskDict& tasks, double time, Robot& robot, vector<Pose> & tour_states) {
	double total_reward = 0;
	double total_time = time;
	auto next_loc = robot.pose;

	tour_states.push_back(robot.pose);

	for (int i = 0; i < complete_tour.size() - 1; i++) {
		auto rt = policy_reward(robot, next_loc, complete_tour[i + 1], time, total_time, tasks);
		total_reward += rt.first;
		total_time = rt.second;
		tour_states.push_back(next_loc);
	}

	// Close the loop (Traveling back to the depot)
	total_reward += calculateNorm(next_loc, robot.globalTaskLocs.back()) / robot.max_v;
	return total_reward;
}

pair<double,double> MCTS::policy_reward(Robot& robot, Pose& next_loc, int taskB, double start_time, double total_time, taskDict& tasks) {
	double time = total_time, reward = 0;
	std::vector<Eigen::Vector2d> taskLocsB;
	auto whichBot = tasks.three[taskB]; 
	auto whichParticle = tasks.four[taskB];
	auto currentLoc = next_loc;
	
	Pose intersectPoint;

	if (tasks.five[taskB] > 0 && total_time < tasks.five[taskB]) {
		
		//auto pB = robot.particles[whichBot][whichParticle];

		if (robot.particles[whichBot][whichParticle].plan.empty()) {
			taskLocsB = robot.depot;
		}
		else {
			for (int i = 0;  i < robot.particles[whichBot][whichParticle].plan.size(); i++) {
				Eigen::Vector2d tmp;
				tmp.x() = robot.particles[whichBot][whichParticle].plan[i].x;
				tmp.y() = robot.particles[whichBot][whichParticle].plan[i].y;
				taskLocsB.push_back(tmp);
			}
		}
		intersectPoint = findIntersectPoint(robot, robot.particles[whichBot][whichParticle], currentLoc, taskLocsB, start_time, total_time);
		double timeToNext = calculateNorm(intersectPoint, currentLoc) / robot.max_v;

		if (total_time + timeToNext > tasks.five[taskB]) {
			reward = 1e6;
		}
		else {
			reward = timeToNext;
			time += timeToNext;
			next_loc = intersectPoint;
		}
	}
	else if (tasks.five[taskB] > 0) {
		reward = 1e6;
	}
	else if (tasks.three[taskB] > -1 && tasks.four[taskB] >= 0 && robot.numParticles[tasks.three[taskB]] <= tasks.four[taskB]) {
		//auto pB = robot.particles[whichBot].back();
		auto path_left = my_setdiff(robot.particles[whichBot].back().originalPlan, robot.particles[whichBot].back().taskQueue);
		std::reverse(path_left.begin(), path_left.end());
		double cost = 0;

		if (!path_left.empty()) {
			for (int p = 0; p < path_left.size() - 1; p++) {
				cost += calculateNorm(robot.globalTaskLocs[path_left[p + 1]], robot.globalTaskLocs[path_left[p]]) / robot.max_v;
			}
	}
		time += cost;
		reward = cost;

		if (!path_left.empty()) {
			next_loc.x = robot.globalTaskLocs[path_left[0]].x();
			next_loc.y = robot.globalTaskLocs[path_left[0]].y();
		}
		else {
			next_loc.x = robot.depot[0].x(); // HARDCODED depot to 1
			next_loc.y = robot.depot[0].y(); // HARDCODED depot to 1
		}

	}
	else if (tasks.three[taskB] > 0 && tasks.four[taskB] >= 0) {
		//auto pB = robot.particles[whichBot].back();
		if (robot.particles[whichBot].back().plan.empty()) {
			
			taskLocsB = robot.depot;
		}
		else {
			taskLocsB.clear(); taskLocsB.resize(robot.particles[whichBot].back().plan.size());
			for (int i = 0; i < robot.particles[whichBot].back().plan.size(); i++) {
				taskLocsB[i].x() = robot.particles[whichBot].back().plan[i].x;
				taskLocsB[i].y() = robot.particles[whichBot].back().plan[i].y;
			}
			
		}

		intersectPoint = findIntersectPoint(robot, robot.particles[whichBot].back(), robot.pose, taskLocsB, start_time, total_time);
		double timeToNext = calculateNorm(intersectPoint, currentLoc) / robot.max_v;
		reward = timeToNext;
		time += timeToNext;
		next_loc = intersectPoint;
	}
	else {
		intersectPoint.x = tasks.one[taskB]; intersectPoint.y = tasks.two[taskB];
		double timeToNext = calculateNorm(intersectPoint, currentLoc) / robot.max_v;
		reward = timeToNext;
		time += timeToNext;
		next_loc = intersectPoint;
	}


	return make_pair(reward, time);
}

Pose MCTS::findIntersectPoint(Robot &robot, Robot &pB, Pose locA, vector<Eigen::Vector2d>& taskLocsB, double start_time, double currTime) {
	double radius = 0, timePast = 0;
	int taskCount = 0, numTasks = taskLocsB.size(), numTimesInLoop = 0;
	auto locB = pB.pose;
	
	Pose intersectPoint; intersectPoint.x = -1; intersectPoint.y = -1; intersectPoint.theta = 1 - 1;

	/*We assume locA is already in the right location but locB may need to
	be simulated forward in time.If not, startTime = currTime */
	while ((calculateNorm(robot.pose, locB) - radius) >= (robot.commRange - robot.commError)) {
	
		//cout << "numTimesInLoop: " << numTimesInLoop << endl;
		auto distToTaskB = calculateNorm(locB, taskLocsB[taskCount]);
		auto angleToTaskB = atan2(taskLocsB[taskCount].y() - locB.y, taskLocsB[taskCount].x() - locB.x);
		locB.x += pB.max_v * cos(angleToTaskB) * robot.dt;
		locB.y += pB.max_v * sin(angleToTaskB) * robot.dt;
		//cout << "locB.x: " << locB.x << " locB.y: " << locB.y << endl;
		if (start_time + timePast >= currTime) {
			radius += robot.max_v * robot.dt;
		}

		
		if (distToTaskB < robot.goalReached && taskCount < numTasks) {
			taskCount++;
		}
		else if (distToTaskB < robot.goalReached && taskCount >= numTasks) {
			taskLocsB.clear(); taskLocsB.resize(1);
			taskLocsB[0] = pB.depot[0]; // HARDCODED DEPOT TO 1
			taskCount = 0; numTasks = 1;
		}

		timePast += robot.dt;
		numTimesInLoop++;
	
		if (numTimesInLoop > 1e8) {
			cout << "numTimesInLoop reached max value" << endl;
			
			return intersectPoint;
		}
	}
	//cout << "Y: " << locA.y << " X: " << locA.x << endl;
	double angleToGoal = atan2(locB.y - locA.y, locB.x - locA.x);
	intersectPoint.x = locA.x + cos(angleToGoal) * radius;
	intersectPoint.y = locA.y + sin(angleToGoal) * radius;

	return intersectPoint;
}