// #pragma once
// // #include <numbers>
// #include <random>
// #include <string>
// #include <numeric>
// #include <unordered_set>
// #include <set>

// #include "robot.h"
// #include "planning.h"
// #include "logger.h"

// class Problem;
// class MCTS;

// /// <summary>
// /// 1 depot only, it is hard coded.
// /// </summary>

// class Algorithm {

// public:
// 	Algorithm(std::string& _init, std::string& _alloc);
// 	void run2();
// 	void findIntersections();
// 	void ifConnected(bool _connected, std::vector<int> & bins, int i);
// 	void updateParticles();
// 	void move_particle_forces(Robot &robot, Robot & particle);
// 	void move_robot_forces(Robot& robot);
// 	Point move_robot_forces_sim(Robot& robot, double obsRho);
// 	void failingMechanism(std::vector<int> &, std::vector<int> &, std::vector<bool> &, int, int);
// 	void task_check_mcts_all(std::vector<int>& conBots, std::vector<int> & opBots, std::vector<int> &bins, int step, int id);
// 	void replan_check_mcts(std::vector<int>& conBots, std::vector<int>& opBots, int step, int k, MCTS& mcts); // implement
// 	void goal_check_mcts(int i, int k, double goalReached);

// 	bool obstacles;
// private:

// 	void findIntersectionPoint(Robot& robot, Pose& locB, Robot pB, double velA, double velB, double goalReached, int step);
// 	void findMinTimes(int step, int k, std::vector<int>& conBots, std::vector<int>& deadBots, std::set<int>& botsComm, std::vector<std::vector<double>>& rzLoc, std::vector<double>& minTimes);
// 	int computePartition(taskDict& newTaskDict, std::vector<double>& timeForTask, std::vector<Pose> & new_loc, std::vector<double>& cost, std::vector<int>& opBots, int tNew, int k);
// 	double computeTourLength(Robot& robot, Pose& pose, std::vector<int>& tour);
// 	void insertPlan(Robot& robot, std::vector<Plan>& whichTasks);
// 	void update_robots(Robot& robot, PResult& planningResults, std::vector<Plan>& whichTasks, std::vector<int>& opBots, std::vector<int>& conBots, std::vector<int>& deadBots, int ops);
// 	void update_robots2(Robot& robot, std::vector<int>& deadBots, std::vector<int>& conBots);
// 	void updateGoal(Robot& robot, int step);
// 	void replanOriginal(std::vector<int>& conBots, std::vector<int>& opBots, int step, int k, MCTS& mcts);
// 	/*===============================================================================================================*/
// 	/* Helper Methods */
// 	bool allRobotsAtFinalPosition(const std::vector<Robot>& robots) {
// 		for (const auto& robot : robots) {
// 			if (!robot.at_final_pos) return false;
// 		}
// 		return true;
// 	}
	
// 	bool condition1(); 
// 	bool condition2();
// 	bool condition3(); 
// 	bool condition4(std::vector<int>& whichBotsOperGlobal, std::vector<bool>& operBotsGlobal);

// 	template<typename T>
// 	bool ifAnyGreater(const std::vector<T>& x, T y) {
// 		for (const auto& a : x) {
// 			if (a > y) return true;
// 		}
// 		return false;
// 	}

// 	bool ifAnyGreaterVector(std::vector<int>& x, std::vector<int>& z, int y) {
// 		for (int i = 0; i < z.size(); i++) {
// 			if (ifAnyGreater(x, y)) {
// 				return true;
// 			}
// 		}
// 		return false;
// 	}

// 	template<typename T>
// 	bool ifAnySmaller(const std::vector<T>& x, T y) {
// 		for (const auto& a : x) {
// 			if (a < y) return true;
// 		}
// 		return false;
// 	}

// 	bool ifAll(const std::vector<int>& x) {
// 		for (const auto& a : x) {
// 			if (a != 0) return false;
// 		}
// 		return true;
// 	}

// 	template<typename T>
// 	std::vector<T> getUniqueElements(const std::vector<T>& vec1, const std::vector<T>& vec2) {
// 		if (vec1.empty()) {
// 			return vec2; // Return vec2 directly if vec1 is empty
// 		}
// 		else if (vec2.empty()) {
// 			return vec1; // Return vec1 directly if vec2 is empty
// 		}

// 		// Combine both std::vectors into one if both are non-empty
// 		std::vector<T> combined;
// 		combined.reserve(vec1.size() + vec2.size()); // Reserve memory upfront
// 		combined.insert(combined.end(), vec1.begin(), vec1.end());
// 		combined.insert(combined.end(), vec2.begin(), vec2.end());

// 		// Sort the combined std::vector to bring duplicates together
// 		sort(combined.begin(), combined.end());

// 		// Use unique followed by erase to remove duplicates
// 		combined.erase(unique(combined.begin(), combined.end()), combined.end());

// 		return combined;
// 	}

// 	std::vector<int> getUniqueElements(const std::vector<int>& vec1, const std::vector<int>& vec2) {
// 		// Combine both std::vectors into one
// 		std::vector<int> combined(vec1.begin(), vec1.end());
// 		combined.insert(combined.end(), vec2.begin(), vec2.end());

// 		// Sort the combined std::vector to bring duplicates together
// 		sort(combined.begin(), combined.end());

// 		// Use unique followed by erase to remove duplicates
// 		// unique returns an iterator to the element that follows the last element not removed
// 		// erase actually removes the duplicates
// 		combined.erase(unique(combined.begin(), combined.end()), combined.end());

// 		return combined;
// 	}

// 	template<typename T>
// 	std::pair<bool,int> checkSum(std::vector<T> vec1, int k) {
// 		// doing this from matlab ->  if sum(robots(k1).commQueue == k3)>0
// 		if (vec1.empty()) {
// 			return std::make_pair(false,-1);
// 		}

// 		for (int i = 0; i < vec1.size(); i++) {
// 			if (vec1[i] == k) {
// 				return std::make_pair(true,i);
// 			}
// 		}

// 		return std::make_pair(false, -1);
// 	}

// 	// Base template for comparing std::vectors of any type
// 	template <typename T>
// 	bool isEqual(const std::vector<T>& vec1, const std::vector<T>& vec2) {
// 		if (vec1.size() != vec2.size()) {
// 			return false;
// 		}

// 		for (size_t i = 0; i < vec1.size(); ++i) {
// 			if (!(vec1[i] == vec2[i])) {
// 				return false;
// 			}
// 		}

// 		return true;
// 	}
// 	// Overload for comparing std::vectors of std::vectors
// 	template <typename T>
// 	bool isEqual(const std::vector<std::vector<T>>& vec1, const std::vector<std::vector<T>>& vec2) {
// 		if (vec1.size() != vec2.size()) {
// 			return false;
// 		}

// 		for (size_t i = 0; i < vec1.size(); ++i) {
// 			if (!isEqual(vec1[i], vec2[i])) { // Use the same function recursively
// 				return false;
// 			}
// 		}

// 		return true;
// 	}

// 	bool hasMatchingElement(const Robot& robot) {
// 		for (const auto& row : robot.plan) {
// 			for (const auto& row2 : robot.dfwm) {
// 				if (row.tq1 == row2) { // Check the third column
// 					return true; // Found a match
// 				}
// 			}
// 		}
// 		return false; // No match found
// 	}

// 	template <typename T>
// 	bool hasMatchingElement(const std::vector<T> & vec1, T toCompare ) {
// 		for (auto elem : vec1) {
// 			if (elem == toCompare) { 
// 				return true; // Found a match
// 			}
// 		}
// 		return false; // No match found
// 	}
// 	// check if there are any deadBots
// 	bool isThereDeadBots() {
		
// 			int totalSum = 0;
// 			for (const auto& robot : robots) {
// 				for (int deadBot : robot.deadBots) {
// 					totalSum += deadBot;
// 				}
// 			}

// 			return totalSum > 0;	
// 	}

// 	void findConnectedDeadBots(std::vector<int> & conBots, std::vector<int> &deadBots) {
// 		//for (int index : conBots) {
// 		//	int sum = 0;
// 		//	// Assuming index is valid
// 		//	for (int db : robots[index].deadBots) {
// 		//		sum += db;
// 		//	}
// 		//	if (sum > 0) {
// 		//		deadBots.push_back(index); // Store the index of the robot with dead bots > 0
// 		//	}
// 		//}
		
// 		std::vector<std::vector<int>> tmp(conBots.size());

// 		for (int ii = 0; ii < conBots.size(); ++ii) {
// 			int robotIndex = conBots[ii];
// 			tmp[ii].resize(robots[robotIndex].deadBots.size());
// 			for (int jj = 0; jj < robots[robotIndex].deadBots.size(); ++jj) {
// 				tmp[ii][jj] = robots[robotIndex].deadBots[jj];
// 			}
// 		}

// 		std::vector<int> tmp2(robots[0].deadBots.size());
// 		for (int ii = 0; ii < robots[0].deadBots.size(); ii++) {
// 			for (int jj = 0; jj < tmp.size(); jj++) {
// 				tmp2[ii] += tmp[jj][ii];
// 			}
// 		}

// 		for (int ii = 0; ii < tmp2.size(); ii++) {
// 			if (tmp2[ii] > 0) {
// 				deadBots.push_back(ii);
// 			}
// 		}
// 	}
// 	//check if plans are equal
// 	bool isPlanEqual(const std::vector<Plan>& plan1, const std::vector<Plan>& plan2) {
// 		if (plan1.size() != plan2.size()) {
// 			//std::cout << "Plans are not the same!" << std::endl;
// 			return false;
// 		}

// 		for (size_t i = 0; i < plan1.size(); ++i) {
// 			if (plan1[i].x != plan2[i].x) {
// 				return false;
// 			}
// 			if (plan1[i].y != plan2[i].y) {
// 				return false;
// 			}
// 			if (plan1[i].tq1 != plan2[i].tq1) {
// 				return false;
// 			}
// 			if (plan1[i].tq2 != plan2[i].tq2) {
// 				return false;
// 			}
// 			if (plan1[i].tq3 != plan2[i].tq3) {
// 				return false;
// 			}
// 			if (plan1[i].id != plan2[i].id) {
// 				return false;
// 			}
// 		}

// 		return true;
// 	}

// 	void copyRobot2Particle(Robot & robot, Robot & particle, int p) {
		
// 		// I assume commented fields do not change. saving some time.
		

// 		//particle.map = robot.map;
// 		particle.pose = robot.pose;
// 		particle.angles = robot.angles;
// 		particle.maxRange = robot.maxRange;
// 		particle.commRange = robot.commRange;
// 		particle.max_v = robot.max_v * (1.0 - ((p + 1.0) - 1.0) * robot.decay); //matched the one in matlab
// 		//particle.depot = robot.depot;
// 		particle.ID = robot.ID;
// 		//particle.dt = robot.dt;
// 		//particle.pose_sim = robot.pose_sim;
// 		particle.numParticles = robot.numParticles;
// 		particle.state = robot.state;
// 		particle.taskQueue = robot.taskQueue;
// 		particle.originalPlan = robot.originalPlan;
// 		//particle.globalTaskLocs = robot.globalTaskLocs;
// 		particle.reachedNewParticle = robot.reachedNewParticle;
// 		particle.goalLoc = robot.goalLoc;
// 		particle.particleFollow = robot.particleFollow;
// 		particle.particleGuess = robot.particleGuess;
// 		particle.particleLastUpdate = robot.particleLastUpdate;
// 		particle.robotLastUpdate = robot.robotLastUpdate;
// 		particle.connected_replan = robot.connected_replan;
// 		particle.replan = robot.replan;
// 		particle.commQueue = robot.commQueue;
// 		particle.rzQueue = robot.rzQueue;
// 		particle.dfwm = robot.dfwm;
// 		particle.tasks_done = robot.tasks_done;
// 		particle.deadBots = robot.deadBots;
// 		particle.root = robot.root;
// 		particle.iamdifferent = robot.iamdifferent;
// 		particle.at_final_pos = robot.at_final_pos;
// 		particle.commError = robot.commError;
// 		particle.planID = robot.planID;
// 	}

// 	double calculateDistance(const Pose& a, const Eigen::Vector2d& b) {
// 		return sqrt(pow(b.x() - a.x, 2) + pow(b.y() - a.y, 2)); 
// 	}

// 	double calculateDistance(const Pose& a, const Point& b) {
// 		return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
// 	}

// 	double wrapToPi(double angle) {
// 		constexpr double pi = 3.14159265358979323846;
// 		// Wrap angle to [-pi, pi] range
// 		while (angle > pi) {
// 			angle -= 2 * pi;
// 		}
// 		while (angle <= -pi) {
// 			angle += 2 * pi;
// 		}
// 		return angle;
// 	}

// 	double calculateNorm(const Robot& thisParticle) {

// 		// Calculate the difference between corresponding elements
// 		double dx = thisParticle.pose.x - thisParticle.plan[0].x;
// 		double dy = thisParticle.pose.y - thisParticle.plan[0].y;

// 		// Calculate and return the Euclidean norm (L2 norm)
// 		return sqrt(dx * dx + dy * dy);
// 	}

// 	double calculateNorm(const Pose& pose1, const Pose &pose2) {

// 		// Calculate the difference between corresponding elements
// 		double dx = pose1.x - pose2.x;
// 		double dy = pose1.y - pose2.y;

// 		// Calculate and return the Euclidean norm (L2 norm)
// 		return sqrt(dx * dx + dy * dy);
// 	}

// 	double calculateNorm(const Pose& pose1, const Eigen::Vector2d& pose2) {

// 		// Calculate the difference between corresponding elements
// 		double dx = pose1.x - pose2.x();
// 		double dy = pose1.y - pose2.y();

// 		// Calculate and return the Euclidean norm (L2 norm)
// 		return sqrt(dx * dx + dy * dy);
// 	}

// 	double calculateNorm(const Pose& pose1, const Point& pose2) {

// 		// Calculate the difference between corresponding elements
// 		double dx = pose1.x - pose2.x;
// 		double dy = pose1.y - pose2.y;

// 		// Calculate and return the Euclidean norm (L2 norm)
// 		return sqrt(dx * dx + dy * dy);
// 	}

// 	double calculateNorm(const Pose& pose1, const Plan& pose2) {

// 		// Calculate the difference between corresponding elements
// 		double dx = pose1.x - pose2.x;
// 		double dy = pose1.y - pose2.y;

// 		// Calculate and return the Euclidean norm (L2 norm)
// 		return sqrt(dx * dx + dy * dy);
// 	}

// 	std::vector<int> my_setdiff(std::vector<int> X, std::vector<int> Y) {

// 		std::unordered_set<int> set_Y(Y.begin(), Y.end());
// 		std::vector<int> Z;

// 		// Loop over elements in X and add those not in Y to the result
// 		for (int x : X) {
// 			if (set_Y.find(x) == set_Y.end()) {
// 				Z.push_back(x);
// 			}
// 		}

// 		return Z;
// 	}

// 	void my_setdiff(std::vector<int> &vec, int Y) {

// 		// Find the element in the std::vector
// 		auto it = std::find(vec.begin(), vec.end(), Y);

// 		// Check if the element was found
// 		if (it != vec.end()) {
// 			// Remove the element
// 			vec.erase(it);
// 		}
// 	}
	
// 	// Function to calculate std::pairwise distance matrix
// 	std::vector<std::vector<int>> calculateDistanceMatrix(const std::vector<Robot>& robots, double rangeCondition) {
// 		std::vector<std::vector<int>> distMat(robots.size(), std::vector<int>(robots.size(), 0.0));
// 		for (size_t i = 0; i < numBots; ++i) {
// 			for (size_t j = 0; j < numBots; ++j) {
// 				double dx = robots[i].pose.x - robots[j].pose.x;
// 				double dy = robots[i].pose.y - robots[j].pose.y;
// 				double dist = sqrt(dx * dx + dy * dy);
// 				if (dist < rangeCondition) {
// 					dist = 1;
// 				}
// 				else {
// 					dist = 0;
// 				}

// 				distMat[i][j] = dist;
// 				distMat[j][i] = dist; // Ensure the matrix is symmetric
// 			}
// 		}
// 		return distMat;
// 	}

// 	// Function to calculate std::pairwise distance matrix
// 	std::vector<std::vector<int>> calculateDistanceMatrix(const std::vector<Pose>& poses, double rangeCondition) {
// 		std::vector<std::vector<int>> distMat(poses.size(), std::vector<int>(poses.size(), 0.0));
// 		for (size_t i = 0; i < poses.size(); ++i) {
// 			for (size_t j = 0; j < poses.size(); ++j) {
// 				double dx = poses[i].x - poses[j].x;
// 				double dy = poses[i].y - poses[j].y;
// 				double dist = sqrt(dx * dx + dy * dy);
// 				if (dist < rangeCondition) {
// 					dist = 1;
// 				}
// 				else {
// 					dist = 0;
// 				}

// 				distMat[i][j] = dist;
// 				distMat[j][i] = dist; // Ensure the matrix is symmetric
// 			}
// 		}
// 		return distMat;
// 	}

// 	void dfs(int node, const std::vector<std::vector<int>>& graph, std::vector<bool>& visited, std::vector<int>& bins, int componentLabel) {
// 		visited[node] = true; // Mark the current node as visited
// 		bins[node] = componentLabel; // Assign current node to the current component

// 		for (int i = 0; i < graph[node].size(); ++i) {
// 			if (graph[node][i] == 1 && !visited[i]) { // If there's a connection and it's not visited
// 				dfs(i, graph, visited, bins, componentLabel);
// 			}
// 		}
// 	}

// 	std::vector<int> findConnectedComponents(const std::vector<std::vector<int>>& graph) {
// 		std::vector<int> bins(graph.size(), -1); // Initialize components std::vector
// 		std::vector<bool> visited(graph.size(), false); // Track visited nodes
// 		int componentLabel = 0; // Start labeling components from 0

// 		for (int i = 0; i < graph.size(); ++i) {
// 			if (!visited[i]) {
// 				dfs(i, graph, visited, bins, componentLabel);
// 				componentLabel++; // Increment component label for the next component
// 			}
// 		}

// 		return bins;
// 	}

// 	// basically doing this -> find(bins(j)==bins); whichConns(whichConns == j) = [];
// 	std::vector<int> findBins(std::vector<int>& bins, int id) {
// 		std::vector<int> _bins;
// 		for (int i = 0; i < bins.size(); i++) {
// 			if (i != id && bins[i] == bins[id]) {
// 				_bins.emplace_back(i);
// 			}
// 		}

// 		return _bins;
// 	}

// 	// basically doing this -> find(true);
// 	template <typename T>
// 	std::vector<int> find(std::vector<T>& bins, T compare) {
// 		std::vector<int> _bins;
// 		for (int i = 0; i < bins.size(); i++) {
// 			if (bins[i] == compare) {
// 				_bins.emplace_back(i);
// 			}
// 		}

// 		return _bins;
// 	}

// 	// doing this unique([A B])
// 	template <typename T>
// 	void mergeUnique(std::vector<T>& vec1, std::vector<T>& vec2, std::vector<T>& inserter) {
// 		std::set<T> uniqueElements(vec1.begin(), vec1.end());
// 		uniqueElements.insert(vec2.begin(), vec2.end());

// 		inserter.clear();
// 		inserter.insert(inserter.end(), uniqueElements.begin(), uniqueElements.end());
// 	}

// 	void findLiveAgents(std::vector<int> &bins, int k, std::vector<int> &opBots) {
		

// 		for (size_t i = 0; i < bins.size(); ++i) {
// 			if (bins[i] == bins[k] && robots[k].deadBots[i] < 1) {
// 				opBots.push_back(i);
// 			}
// 		}
// 		/*for (int i = 0; i < bins.size(); i++) {
// 			if (robots[k].deadBots[bins[i]] != 0) {
				
// 				opBots.erase(opBots.begin() + i);
				
// 			}
// 		}*/
// 		//opBots.erase(opBots.begin() + count + 2, opBots.end());
// 	}

// 	// Calculate the mean position of a group of agents
// 	std::vector<double> calculateMeanPosition(const std::vector<Pose>& poses) {
// 		double sumX = 0.0, sumY = 0.0; 
// 		size_t n = poses.size();
// 		for (int index = 0; index < n; index++) {
// 			sumX += poses[index].x;
// 			sumY += poses[index].y;
// 		}
		
// 		std::vector<double> _return(2);
// 		_return[0] = sumX / n; _return[1] = sumY / n;

// 		return _return;
// 	}

// 	// Calculate the mean position of a group of agents
// 	std::vector<double> calculateMeanPosition(const std::vector<Plan>& poses) {
// 		double sumX = 0.0, sumY = 0.0;
// 		size_t n = poses.size();
// 		for (int index = 0; index < n; index++) {
// 			sumX += poses[index].x;
// 			sumY += poses[index].y;
// 		}

// 		std::vector<double> _return(2);
// 		_return[0] = sumX / n; _return[1] = sumY / n;

// 		return _return;
// 	}

// 	// Calculate the mean position of a group of agents
// 	std::vector<double> calculateMeanPosition(const std::vector<std::vector<double>>& pos, std::pair<int, int> rows) {
// 		double sumX = 0.0, sumY = 0.0;
// 		int n = rows.second - rows.first;
// 		for (int index = rows.first; index < rows.second; index++) {
// 			sumX += pos[index][0];
// 			sumY += pos[index][1];
// 		}

// 		std::vector<double> _return(2);
// 		_return[0] = sumX / n; _return[1] = sumY / n;

// 		return _return;
// 	}

// 	// Basically matlab function sortRows(A,columns)
// 	void sortMatrixByColumns(std::vector<std::vector<double>>& matrix, const std::vector<int>& columns) {
// 		sort(matrix.begin(), matrix.end(), [&columns](const std::vector<double>& row1, const std::vector<double>& row2) {
// 			for (int index : columns) {
// 				if (row1[index] != row2[index]) {
// 					return (index > 0) ? (row1[index] < row2[index]) : (row1[index] > row2[index]);
// 				}
// 				// If equal, check next column
// 			}
// 			return false; // All compared columns are equal
// 		});
// 	}

// 	Pose findRz(Robot& robot, Robot& pB, Pose locA, double start_time, double &time) {
// 		std::vector<Eigen::Vector2d> taskLocsB;
// 		double currTime = start_time;

// 		if (pB.plan.empty()) {
// 			taskLocsB = robot.depot;
// 		}
// 		else {
// 			for (int i = 0; i < pB.plan.size(); i++) {
// 				Eigen::Vector2d tmp;
// 				tmp.x() = pB.plan[i].x;
// 				tmp.y() = pB.plan[i].y;
// 				taskLocsB.push_back(tmp);
// 			}
// 		}

// 		double radius = 0, timePast = 0;
// 		int taskCount = 0, numTasks = taskLocsB.size(), numTimesInLoop = 0;
// 		auto locB = pB.pose;

// 		Pose intersectPoint; intersectPoint.x = -1; intersectPoint.y = -1; intersectPoint.theta = 1 - 1;

// 		/*We assume locA is already in the right location but locB may need to
// 		be simulated forward in time.If not, startTime = currTime */
// 		while ((calculateNorm(robot.pose, locB) - radius) >= (robot.maxRange - robot.commError)) {

// 			//cout << "numTimesInLoop: " << numTimesInLoop << endl;
// 			auto distToTaskB = calculateNorm(locB, taskLocsB[taskCount]);
// 			auto angleToTaskB = atan2(taskLocsB[taskCount].y() - locB.y, taskLocsB[taskCount].x() - locB.x);
// 			locB.x += pB.max_v * cos(angleToTaskB) * robot.dt;
// 			locB.y += pB.max_v * sin(angleToTaskB) * robot.dt;
// 			//cout << "locB.x: " << locB.x << " locB.y: " << locB.y << endl;
// 			if (start_time + timePast >= currTime) {
// 				radius += robot.max_v * robot.dt;
// 			}


// 			if (distToTaskB < robot.goalReached && taskCount < numTasks) {
// 				taskCount++;
// 			}
// 			else if (distToTaskB < robot.goalReached && taskCount >= numTasks) {
// 				taskLocsB.clear(); taskLocsB.resize(1);
// 				taskLocsB[0] = pB.depot[0]; // HARDCODED DEPOT TO 1
// 				taskCount = 0; numTasks = 1;
// 			}

// 			timePast += robot.dt;
// 			numTimesInLoop++;

// 			if (numTimesInLoop > 1e8) {
// 				std::cout << "numTimesInLoop reached max value" << std::endl;

// 				return intersectPoint;
// 			}
// 		}
// 		//cout << "Y: " << locA.y << " X: " << locA.x << endl;
// 		double angleToGoal = atan2(locB.y - locA.y, locB.x - locA.x);
// 		intersectPoint.x = locA.x + cos(angleToGoal) * radius;
// 		intersectPoint.y = locA.y + sin(angleToGoal) * radius;
// 		time = start_time + timePast;
// 		return intersectPoint;
// 	}
	
// 	int numBots, numParticles, pIdx;
// 	std::vector<std::vector<double>> timeWindows;	
// 	std::vector<std::vector<std::vector<int>>> particleHistory;
// 	std::vector<Robot> robots;
// 	std::vector<int> whenFail, whoFail;
// 	Problem problem;
// 	std::vector<bool> flag;
// };