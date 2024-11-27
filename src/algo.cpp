#include <set>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <list>
#include <numeric>
#include <algorithm>

#include "../include/MCTS_MTSP_EP.h"
#include "../include/randomGenerator.h"
#include "../include/planning.h"
#include "../include/algo.h"
#include "../include/logger.h"

using namespace std;
using namespace Eigen;

Algo::Algo(string& _init, string& _alloc) {
 
	obstacles = false;
	problem.readConfigurations(_init, _alloc);
	numBots = problem.robotLocs.size();
	robots.resize(numBots);

	for (int i = 0; i < numBots; i++) {
		robots[i].initRobots(i, problem);
	}
	for (int i = 0; i < numBots; i++) {
		robots[i].initParticles(robots, i);
		robots[i].initMap(robots[i], problem);
	}
	numParticles = robots[0].numP; // we assume the same number of particles for each robot;
	pIdx = numParticles - 1;

	// Resize the 3D vector inline
	particleHistory.resize(numBots);
	flag.resize(numBots);
	for (int i = 0; i < numBots; ++i) {
		flag[i] = true;
		particleHistory[i].resize(numBots);
	}

	//debugger::displayRobotInfo(robots);
	findIntersections();


}

void Algo::run() {

	int numIters = 1500;
	whenFail = {  150,350 }; // Simulation step when failure occurs, could be randomized
	whoFail = {  2,1 }; // ID of the robot that will experience failure, could be randomized
	int howFail = 2; // The nature or impact of the failure, could be randomized
	int numHistory = 60; // How far back in history we consider for some calculations
	int obsDist = 5;
	LOG logger;

	logger.robotPose.resize(numBots);
	logger.particlePose.resize(numBots);
	logger.goalLoc.resize(numBots);
	MCTS mcts;

	vector<bool> operBotsGlobal(numBots, true);
	int connected_cycles = 0;
	int i = 1;
	double rangeCondition = 2 * obsDist;
	bool condition = true;

	while (condition)
	{
		if ((i + 1) % 100 == 0) {
			cout << "Iteration: " << i << endl;
		}

		auto distMat = calculateDistanceMatrix(robots, rangeCondition);
		auto bins = findConnectedComponents(distMat);
		bool connected = ifAll(bins);

		ifConnected(connected, bins, i);
		updateParticles();
		failingMechanism(whenFail, whoFail, operBotsGlobal, howFail, i);

		/*for ( int rr = 0; rr < robots.size(); rr++) {
			cout << "x: " << robots[rr].pose.x << " y: " << robots[rr].pose.y << endl;
		}*/

		for (int k = 0; k < numBots; k++) {
			auto conBots = find(bins, bins[k]);
			vector<int> opBots;
			task_check_mcts_all(conBots, opBots, bins, i, k);
			replan_check_mcts(conBots, opBots, i, k, mcts); // implement
			goal_check_mcts(i, k, robots[0].goalReached);

			// robots[k].pose_sim 0; cmd_vel_sim = 0; cmd_vel_rbt = 0; We dont need this, we are not running real robots
			int whichFollow = min(pIdx, robots[k].particleFollow);
			move_robot_forces(robots[k]);

			// log data
			logger.robotPose[k].push_back(robots[k].pose);
			logger.goalLoc[k].push_back(robots[k].goalLoc);

			logger.particlePose[k].resize(numBots);
			for (int rr = 0; rr < numBots; rr++) {
				logger.particlePose[k][rr].resize(numParticles);
				for (int p = 0; p < numParticles; p++) {
					logger.particlePose[k][rr][p].push_back(robots[k].particles[rr][p].pose);
				}
			}
		}

		

		std::vector<int> whichBotsOperGlobal;
		for (int i = 0; i < operBotsGlobal.size(); ++i) {
			if (operBotsGlobal[i]) { // Check if the value is true
				whichBotsOperGlobal.push_back(i); // Store index 
			}
		}

		i++;
		if (i == 5000) {
			break;
		}
		bool c1 = condition1();
		bool c2 = condition2();
		bool c3 = condition3();
		bool c4 = condition4(whichBotsOperGlobal, operBotsGlobal);

		condition = ((c1 || !c2) && (i < numIters)) || (c3 || !c4);

		int tmpCounter = 0;
		for (int ii = 0; ii < numBots; ii++) {

			if (robots[ii].at_final_pos == true) {
				tmpCounter++;
			}
		}
		if (numBots == tmpCounter) {
			break;
		}
	}

	logger.writePose2File(robots);
	logger.writeParticle2File();
}

bool Algo::condition1() {
	// Condition 1
	for (int i = 0; i < numBots; i++) {
		double tmp = calculateNorm(robots[i].pose, robots[i].depot[0]); // DEPOT HARDCODED TO 1
		if (tmp < robots[i].goalReached) {
			return false;
		}
	}
	return true;
}

bool Algo::condition2() {
	// Condition 2
	for (int i = 0; i < numBots; i++) {
		if (!robots[i].plan.empty()) {
			return true;
		}
	}
	return false;
}

bool Algo::condition3() {
	// Condition 3

	for (int i = 0; i < numBots; i++) {
		if (robots[i].at_final_pos == false) {
			return false;
		}
	}

	return true;
}

bool Algo::condition4(vector<int>& whichBotsOperGlobal, vector<bool>& operBotsGlobal) {
	vector<int> v1(robots[0].globalTaskLocs.size());
	std::iota(v1.begin(), v1.end(), 0);
	auto res = my_setdiff(v1, robots[whichBotsOperGlobal[0]].tasks_done);
	if (res.empty()) {
		for (int i = 0; i < numBots; i++) {
			double tmp = calculateNorm(robots[i].pose, robots[i].depot[0]); // DEPOT HARDCODED TO 1
			if (tmp < robots[i].goalReached || operBotsGlobal[i] == false) {
				robots[i].at_final_pos = true;
			}
		}
		return true;
	}
	return false;

}

void Algo::findIntersections() {

	vector<vector<vector<double>>> connections(numBots);
	vector<Pose> poses(numBots);
	vector<double> dist(numBots);
	vector<int> task_holds(numBots, 0);
	Vector2d taskToDo;
	int count = 0;
	double limit = 1;
	double rangeCondition = 2 * robots[0].maxRange;

	for (int i = 0; i < numBots; i++) {
		dist[i] = calculateDistance(robots[i].pose, robots[i].depot[0]); // HARDCODED FOR ONLY 1 DEPOT
		poses[i] = robots[i].pose;
	}

	while (ifAnyGreater(dist, limit)) {
		for (int i = 0; i < numBots; i++) {
			if (task_holds[i] >= robots[i].taskQueue.size()) {
				taskToDo = robots[i].depot[0]; // HARDCODED FOR ONLY 1 DEPOT
			}
			else {
				taskToDo = robots[i].globalTaskLocs[robots[i].taskQueue[task_holds[i]]];
			}

			double agentDist = calculateDistance(poses[i], taskToDo);

			if (agentDist < limit) {
				task_holds[i]++;
			}

			double theta_d = atan2(taskToDo(1) - poses[i].y, taskToDo(0) - poses[i].x);
			poses[i].x += robots[i].max_v * cos(theta_d) * robots[i].dt;
			poses[i].y += robots[i].max_v * sin(theta_d) * robots[i].dt;

		}

		auto distMat = calculateDistanceMatrix(poses, rangeCondition);
		auto components = findConnectedComponents(distMat); // I assume the range is the same on all robots

		for (int i = 0; i < numBots; i++) {
			auto whichConns = findBins(components, i);

			if (whichConns.size() > 1) {

				for (int k = 0; k < whichConns.size(); k++) {
					vector<double> tmpConn;
					tmpConn = calculateMeanPosition(poses);
					tmpConn.emplace_back(i);
					tmpConn.emplace_back(whichConns[k]);
					tmpConn.emplace_back(robots[0].dt * count);
					connections[i].emplace_back(tmpConn);
				}
			}
		}

		for (int i = 0; i < numBots; i++) {
			dist[i] = calculateDistance(poses[i], robots[i].depot[0]); // HARDCODED FOR ONLY 1 DEPOT
		}
		count++;

	}

	for (int k = 0; k < numBots; k++) {
		auto rz = connections[k];
		if (!rz.empty()) {
			double startTime = rz[0].back();
			double hold = startTime;
			int startRow = 0;

			vector<int> columns = { 2,3,4 };
			sortMatrixByColumns(rz, columns);

			for (int i = 1; i < rz.size(); i++) {
				double lastElement = rz[i].back(); // Accessing the last element of the i-th sub-vector
				if (abs(lastElement - (hold + robots[k].dt)) > 1e-4) {
					vector<double> tmp;
					double finishTime = rz[i - 1].back();
					auto avgPos = calculateMeanPosition(rz, make_pair(startRow, i));
					tmp.insert(tmp.end(), avgPos.begin(), avgPos.end());
					tmp.emplace_back(k);
					tmp.emplace_back(rz[i - 1][3]);
					tmp.emplace_back(startTime);
					tmp.emplace_back(finishTime);
					timeWindows.emplace_back(tmp);
					startTime = rz[i].back();
					startRow = i;
					hold = startTime;
				}
				else {
					hold += robots[k].dt;
				}
			}
		}
	}
}

void Algo::ifConnected(bool _connected, vector<int>& bins, int i) {

	if (_connected) {
		for (int k = 0; k < numBots; k++) {
			robots[k].particleFollow = 1;
			robots[k].particleGuess = vector<int>(numBots, 0); // not sure if it should be 0 or 1

			for (int p = 0; p < numParticles; p++) {
				copyRobot2Particle(robots[k], robots[k].particles[k][p], p);
			}
		}
	}
	for (int k1 = 0; k1 < numBots; k1++) {
		for (int k2 = 0; k2 < numBots; k2++) {

			if (!isPlanEqual(robots[k2].plan, robots[k2].particles[k2][0].plan)) {
				robots[k2].iamdifferent = true;
			}

			if (robots[k1].max_v == 0) {
				robots[k1].deadBots[k1] = 1;
			}
			else {
				robots[k1].deadBots[k1] = 0;
			}

			if (bins[k1] == bins[k2] && k1 != k2) {
				if (robots[k1].max_v > 0 && robots[k2].deadBots[k1]) {
					robots[k2].deadBots[k1] = 0;
					robots[k1].replan = 2;
				}
				else if (robots[k1].max_v == 0 && !robots[k2].deadBots[k1]) {
					robots[k2].deadBots[k1] = 1;
					robots[k2].replan = 2;
				}

				robots[k1].robotLastUpdate[k2] = i;
				robots[k1].particleLastUpdate[k2] = i;
				robots[k1].tasks_done = getUniqueElements(robots[k1].tasks_done, robots[k2].tasks_done);

				auto cs = checkSum(robots[k1].commQueue, k2); // first is a bool, second is the index -> commIdx
				auto cs2 = checkSum(robots[k1].rzQueue, k2);
				if (cs.first) {
					robots[k1].commQueue.erase(robots[k1].commQueue.begin() + cs.second);
					robots[k1].replan = 2;
					if (robots[k2].max_v == 0) {
						robots[k1].deadBots[k2] = 1;
					}
					//robots[k1].dfwm = getUniqueElements(robots[k1].dfwm, vector<int>(1, k2)); // simply making k2 a single element vector, to fit the function template
				}
				else if (cs2.first) {
					robots[k1].rzQueue.erase(robots[k1].rzQueue.begin() + cs2.second);
					robots[k1].replan = 2;
					robots[k1].dfwm = getUniqueElements(robots[k1].dfwm, vector<int>(1, k2));
				}

				if (robots[k2].iamdifferent || _connected) {
					for (int p = 0; p < numParticles; p++) {
						copyRobot2Particle(robots[k2], robots[k1].particles[k2][p], p); // updating the belief of robot k1 about k2
					}
				}
			}

			for (int k3 = 0; k3 < numBots; k3++) {
				if (k3 != k1 && k3 != k2 && bins[k1] == bins[k2] && k1 != k2) {
					if (robots[k1].robotLastUpdate[k3] < robots[k2].robotLastUpdate[k3]) {
						robots[k1].robotLastUpdate[k3] = robots[k2].robotLastUpdate[k3];
						robots[k1].particleLastUpdate[k3] = robots[k2].particleLastUpdate[k3];
						if (robots[k2].deadBots[k3] == 1) {
							robots[k1].deadBots[k3] = 1;
							auto cs = checkSum(robots[k1].commQueue, k3);
							auto cs2 = checkSum(robots[k1].rzQueue, k3);
							if (cs.first) {
								robots[k1].commQueue.erase(robots[k1].commQueue.begin() + cs.second);
								robots[k1].replan = 2;
								robots[k1].dfwm = getUniqueElements(robots[k1].dfwm, vector<int>(1, k3));
							}
							else if (cs2.first) {
								robots[k1].rzQueue.erase(robots[k1].rzQueue.begin() + cs2.second);
								robots[k1].replan = 2;
								robots[k1].dfwm = getUniqueElements(robots[k1].dfwm, vector<int>(1, k3));
							}
						}

						for (int p = 0; p < numParticles; p++) {
							copyRobot2Particle(robots[k2].particles[k3][p], robots[k1].particles[k3][p], p);
							// there might be a problem here with vmax update with decay
						}

					}
					else if (robots[k1].particleLastUpdate[k3] < robots[k2].particleLastUpdate[k3]) {
						robots[k1].particleLastUpdate[k3] = robots[k2].particleLastUpdate[k3];
						robots[k1].particleGuess[k3] = robots[k2].particleGuess[k3];
					}
				}
				//cout << "k1: " << k1 << " k2: " << k2 << " k3: " << k3 << endl;
			}
		}
	}
}

void Algo::updateParticles() {

	for (int k = 0; k < numBots; k++) {
		for (int r = 0; r < numBots; r++) {
			for (int p = 0; p < numParticles; p++) {
				//If connected update particle guesses to current pose and speed
				robots[k].particles[r][p].goalLoc.x = robots[k].depot[0].x();  // HARDCODED FOR ONLY 1 DEPOT
				robots[k].particles[r][p].goalLoc.y = robots[k].depot[0].y(); // HARDCODED FOR ONLY 1 DEPOT
				if (!robots[k].particles[r][p].plan.empty()) {
					// Is a task complete ? Check if tasks should be added or are complete
					if (calculateNorm(robots[k].particles[r][p]) < robots[k].goalReached) {
						robots[k].particles[r][p].plan.erase(robots[k].particles[r][p].plan.begin());
						//						if (!robots[k].particles[r][p].plan.empty() && robots[k].particles[r][p].plan[0].tq2 == -1) { // should be this thisParticle.plan(1,4) == 0
						if (!robots[k].particles[r][p].taskQueue.empty()) {
							robots[k].particles[r][p].taskQueue.erase(robots[k].particles[r][p].taskQueue.begin());
						}

					}
					if (!robots[k].particles[r][p].plan.empty()) {
						robots[k].particles[r][p].goalLoc.x = robots[k].particles[r][p].plan[0].x;
						robots[k].particles[r][p].goalLoc.y = robots[k].particles[r][p].plan[0].y;
					}
				}
				move_particle_forces(robots[k], robots[k].particles[r][p]);
			}
		}
	}
}

void Algo::move_particle_forces(Robot& robot, Robot& particle) {
    constexpr double pi = 3.14159265358979323846;
	vector<Vector2d> obs; // To store (column, row) pairs
	auto goal = particle.goalLoc;
	// Find indices where robot.M0 > 0.5 and convert them to (column, row)
	double obsRho = robot.obsDist[0];
	if (obstacles) {
		for (int r = 0; r < robot.map.M0.size(); ++r) {
			for (int c = 0; c < robot.map.M0[r].size(); ++c) {
				if (robot.map.M0[r][c] > 0.5) {
					// Note: Adding 1 to each index to match MATLAB's 1-based indexing
					obs.push_back({ c + 1, r + 1 }); // Swap order to (column, row) to match MATLAB output
				}
			}
		}
	}
	if (!obs.empty()) {
		for (int i = 0; i < obs.size(); i++) {
			robot.obsDist.push_back(calculateDistance(robot.pose, obs[i]));
		}
	}
	vector<Vector2d> c_obs;
	vector<double> filteredObsDist;

	// Convert robot pose to Vector2d
	Vector2d robotPosition(particle.pose.x, particle.pose.y);

	for (size_t i = 0; i < robot.obsDist.size(); ++i) {
		if (robot.obsDist[i] < obsRho) {
			c_obs.push_back(obs[i]); // Add matching obs elements to c_obs
			filteredObsDist.push_back(robot.obsDist[i]); // Keep matching distances
		}
	}
	Vector2d obsSpring(0.0, 0.0); // Initialize obsSpring
	if (!c_obs.empty()) {
		double minVal = numeric_limits<double>::max();
		size_t minIdx = 0;
		for (size_t i = 0; i < robot.obsDist.size(); ++i) {
			if (robot.obsDist[i] < minVal) {
				minVal = robot.obsDist[i];
				minIdx = i;
			}
		}

		Vector2d dirObs = c_obs[minIdx] - robotPosition;

		if (minVal > 1.5) {
			double adjustmentFactor = (1.0 / 1.5 - 1.0 / minVal) * (1.0 / pow(minVal, 2));
			obsSpring = -(dirObs / minVal) * adjustmentFactor;
		}
		else {
			Vector2d signDirObs = dirObs.unaryExpr([](double v) {
				return copysign(1.0, v); // Use copysign to return the sign of v
			});
			auto constVal = Vector2d::Constant(1000.0);
			obsSpring = -signDirObs.array() * constVal.array();
		}
	}
	else {
		obsSpring = Vector2d::Zero(); // Use Eigen's Zero() method for a zero vector
	}

	//	 Calculate distance to the goal
	double goalDist = (goal - robotPosition).norm();

	Point goalNorm;
	if (goalDist > 0) {
		goalNorm = (goal - robotPosition) / goalDist;
	}

	Point goalSpring = goalNorm * goalDist;

	// Limit the magnitude of goalSpring by max_v
	//if (goalSpring.norm() > robot.max_v) {
	//	goalSpring = goalSpring * robot.max_v / goalSpring.norm();
	//}

	if (goalSpring.norm() > particle.max_v) {
		goalSpring = goalSpring * particle.max_v / goalSpring.norm();
	}

	// Calculate v
	Point v = goalSpring * 1.0;

	// Check if speed exceeds max_v
	/*if (v.norm() / robot.dt > robot.max_v) {
		v *= robot.max_v / v.norm();
	}*/
	if (v.norm() / robot.dt > particle.max_v) {
		v *= particle.max_v / v.norm();
	}

	// Check if velocity is below a threshold
	if (v.norm() < 0.1) {
		v.x = 0; v.y = 0; // Set velocity to zero if below threshold
	}

	// Calculate force
	Point force = v;

	// Calculate theta_d
	double theta_d = wrapToPi(atan2(v.y, v.x) - particle.pose.theta); // Assuming pose(3) is the angle
	theta_d = copysign(min(pi, abs(theta_d)), theta_d);

	// Calculate velocity component
	double vel = cos(theta_d) * v.norm();

	// Update pose
	particle.pose.x += (vel * cos(particle.pose.theta)) * robot.dt;
	particle.pose.y += (vel * sin(particle.pose.theta)) * robot.dt;
	particle.pose.theta = wrapToPi(particle.pose.theta + theta_d); // Assuming pose(2) is angle and wrapToPi is a custom function to wrap angle to [-pi, pi]

}

void Algo::move_robot_forces(Robot& robot) {
    constexpr double pi = 3.14159265358979323846;
	Point goalNorm, v;
	Pose goalSpring;

	if (robot.goalLoc.empty()) {
		robot.goalLoc.x = robot.depot[0].x(); // DEPOT IS HARDCODED TO 1
		robot.goalLoc.y = robot.depot[0].y(); // DEPOT IS HARDCODED TO 1
	}

	auto goalDist = calculateDistance(robot.pose, robot.goalLoc);

	if (goalDist > 0) {

		goalNorm.x = (robot.goalLoc.x - robot.pose.x) / goalDist;
		goalNorm.y = (robot.goalLoc.y - robot.pose.y) / goalDist;
	}
	else {
		goalNorm.x = 0; goalNorm.y = 0;
	}

	goalSpring.x = goalDist * goalNorm.x;
	goalSpring.y = goalDist * goalNorm.y;

	double val = sqrt(pow(goalSpring.x, 2) + pow(goalSpring.y, 2));
	if (val > robot.max_v) {
		goalSpring.x *= robot.max_v / val;
		goalSpring.y *= robot.max_v / val;
	}

	double goalSpringGain = 1.0;
	v.x = goalSpringGain * goalSpring.x;
	v.y = goalSpringGain * goalSpring.y;

	double val2 = sqrt(pow(v.x, 2) + pow(v.y, 2));

	if (val2 / robot.dt > robot.max_v) {
		v.x *= robot.max_v / val2;
		v.y *= robot.max_v / val2;
		val2 = sqrt(pow(v.x, 2) + pow(v.y, 2));
	}

	if (val2 < 0.1) {
		v.x = 0; v.y = 0;
		val2 = 0;
	}

	auto theta_d = wrapToPi(atan2(v.y, v.x) - robot.pose.theta);
	theta_d = std::copysign(1.0, theta_d) * std::min(pi, std::abs(theta_d));
	double vel = cos(theta_d) * val2;

	robot.pose.x += (vel * cos(robot.pose.theta)) * robot.dt;
	robot.pose.y += (vel * sin(robot.pose.theta)) * robot.dt;
	robot.pose.theta = wrapToPi(robot.pose.theta + theta_d);
}

Point Algo::move_robot_forces_sim(Robot& robot, double obsRho) {
	vector<Vector2d> obs; // To store (column, row) pairs
	//	auto goal = particle.goalLoc;
	// Find indices where robot.M0 > 0.5 and convert them to (column, row)
	// double obsRho = robot.obsDist[0];
	if (obstacles) {
		for (int r = 0; r < robot.map.M0.size(); ++r) {
			for (int c = 0; c < robot.map.M0[r].size(); ++c) {
				if (robot.map.M0[r][c] > 0.5) {
					// Note: Adding 1 to each index to match MATLAB's 1-based indexing
					obs.push_back({ c + 1, r + 1 }); // Swap order to (column, row) to match MATLAB output
				}
			}
		}
	}
	if (!obs.empty()) {
		for (int i = 0; i < obs.size(); i++) {
			robot.obsDist.push_back(calculateDistance(robot.pose, obs[i]));
		}
	}
	vector<Vector2d> c_obs;
	vector<double> filteredObsDist;

	// Convert robot pose to Vector2d
	Vector2d robotPosition(robot.pose.x, robot.pose.y);

	for (size_t i = 0; i < robot.obsDist.size(); ++i) {
		if (robot.obsDist[i] < obsRho) {
			c_obs.push_back(obs[i]); // Add matching obs elements to c_obs
			filteredObsDist.push_back(robot.obsDist[i]); // Keep matching distances
		}
	}
	Vector2d obsSpring(0.0, 0.0); // Initialize obsSpring
	if (!c_obs.empty()) {
		double minVal = numeric_limits<double>::max();
		size_t minIdx = 0;
		for (size_t i = 0; i < robot.obsDist.size(); ++i) {
			if (robot.obsDist[i] < minVal) {
				minVal = robot.obsDist[i];
				minIdx = i;
			}
		}

		Vector2d dirObs = c_obs[minIdx] - robotPosition;

		if (minVal > 1.5) {
			double adjustmentFactor = (1.0 / 1.5 - 1.0 / minVal) * (1.0 / pow(minVal, 2));
			obsSpring = -(dirObs / minVal) * adjustmentFactor;
		}
		else {
			Vector2d signDirObs = dirObs.unaryExpr([](double v) {
				return copysign(1.0, v); // Use copysign to return the sign of v
			});
			auto constVal = Vector2d::Constant(1000.0);
			obsSpring = -signDirObs.array() * constVal.array();
		}
	}
	else {
		obsSpring = Vector2d::Zero(); // Use Eigen's Zero() method for a zero vector
	}

	Point goalNorm, v;
	Pose goalSpring;

	auto goalDist = calculateDistance(robot.pose, robot.goalLoc);

	if (goalDist > 0) {
		goalNorm.x = (robot.goalLoc.x - robot.pose.x) / goalDist;
		goalNorm.y = (robot.goalLoc.y - robot.pose.y) / goalDist;
	}
	else {
		goalNorm.x = 0; goalNorm.y = 0;
	}

	goalSpring.x = goalDist * goalNorm.x;
	goalSpring.y = goalDist * goalNorm.y;

	double val = sqrt(pow(goalSpring.x, 2) + pow(goalSpring.y, 2));
	if (val > robot.max_v) {
		goalSpring.x *= robot.max_v / val;
		goalSpring.y *= robot.max_v / val;
	}

	v.x = 1 / (robot.dt * goalSpring.x);
	v.y = 1 / (robot.dt * goalSpring.y);

	double val2 = sqrt(pow(v.x, 2) + pow(v.y, 2));

	if (val2 / robot.dt > robot.max_v) {
		v.x *= robot.max_v / val2;
		v.y *= robot.max_v / val2;
		val2 = sqrt(pow(v.x, 2) + pow(v.y, 2));
	}

	if (val2 < 0.1) {
		v.x = 0; v.y = 0;
		val2 = 0;
	}

	auto theta_d = wrapToPi(atan2(v.y, v.x) - robot.pose.theta);
	double vel = val2;

	robot.pose.x += (vel * cos(robot.pose.theta)) * robot.dt;
	robot.pose.y += (vel * sin(robot.pose.theta)) * robot.dt;
	robot.pose.theta = wrapToPi(robot.pose.theta + theta_d);

	return v;
}

void Algo::failingMechanism(vector<int>& whenFail, vector<int>& whoFail, vector<bool>& operBotsGlobal, int howFail, int i) {
	for (int k = 0; k < numBots; k++) {
		if (ifAnySmaller(whenFail, i + 1)) {
			auto id = find(whenFail, i);
			bool flag = false;
			for (int ii : id) {
				if (k == whoFail[ii]) {
					flag = true;
				}
			}
			if (flag) {
				cout << "Robot " << k << ": Failed on iteration " << i << endl;
				robots[k].particleFollow = howFail;
				robots[k].iamdifferent = true;
				robots[k].max_v = 0;
				operBotsGlobal[k] = false;
			}

		}
	}
}

void Algo::task_check_mcts_all(vector<int>& conBots, vector<int>& opBots, vector<int>& bins, int step, int k) {

	//Find connected agents who aren't dead
	findLiveAgents(bins, k, opBots);


	//Who do we expect to be there?
	if (robots[k].max_v > 0) {
		for (int r = 0; r < numBots; r++) {
			if (k != r) {
				vector<int> plannedForAtDepot;
				// Which ones have I accounted for but am not assigned
				int particleIdx = min(robots[k].particleGuess[r], pIdx);
				auto otherPose = robots[k].particles[r][particleIdx].pose;

				//Who isn't here and I havent planned for
				if ((calculateNorm(robots[k].pose, otherPose) < robots[k].maxRange * 2 - 2) &&
					bins[k] != bins[r] &&
					!checkSum(robots[k].commQueue, r).first &&
					!checkSum(robots[k].dfwm, r).first) {

					robots[k].replan = 2; //Start with replan as if already in our queue
					// Add to particle guess
					robots[k].particleGuess[r]++;
					robots[k].particleLastUpdate[r] = step;
					robots[k].commQueue.push_back(r);

					// Any robots that cannot be found through particles
					if (robots[k].particleGuess[r] > pIdx) {
						robots[k].deadBots[r] = 1;
						robots[k].replan = 2; //
					}
				}
				else if ((calculateNorm(robots[k].pose, otherPose) < robots[k].maxRange * 2 - 2) &&
					bins[k] != bins[r] &&
					checkSum(robots[k].commQueue, r).first &&
					robots[k].deadBots[r] < 1 &&
					!checkSum(robots[k].dfwm, r).first)
				{
					robots[k].replan = 2;
					robots[k].particleLastUpdate[r] = step;
					robots[k].particleGuess[r]++;

					// Any robots that cannot be found through particles
					if (robots[k].particleGuess[r] > pIdx) {
						robots[k].deadBots[r] = 1;
					}

					if (calculateNorm(robots[k].pose, robots[k].depot[0]) < robots[k].maxRange * 2) { // HARDCODED FOR ONLY 1 DEPOT
						plannedForAtDepot.push_back(r);
						mergeUnique(plannedForAtDepot, conBots, plannedForAtDepot);
					}
				}

				//If connected and I am supposed to connect
				else if (bins[k] == bins[r] && checkSum(robots[k].commQueue, r).first) {
					robots[k].replan = 2;
					int count = 0;
					for (int i = 0; i < robots[k].commQueue.size(); i++) {
						if (robots[k].commQueue[i] == r) {
							int idx = robots[k].commQueue.size() - (count + 1);
							swap(robots[k].commQueue[i], robots[k].commQueue[idx]);
							count++;
						}
					}
					robots[k].commQueue.erase(robots[k].commQueue.begin() + count - 1, robots[k].commQueue.end());

					// We'll iterate backward to safely remove elements without invalidating indices
					for (auto it = robots[k].plan.rbegin(); it != robots[k].plan.rend(); ++it) {
						if ((*it).tq1 == r && (*it).tq2 > 0) {
							robots[k].plan.erase((it + 1).base());
						}
					}
				}
				else if (bins[k] == bins[r] && checkSum(robots[k].rzQueue, r).first) {
					robots[k].replan = 2;
					int count = 0;
					for (int i = 0; i < robots[k].rzQueue.size(); i++) {
						if (robots[k].rzQueue[i] == r) {
							int idx = robots[k].rzQueue.size() - (count + 1);
							swap(robots[k].rzQueue[i], robots[k].rzQueue[idx]);
							count++;
						}
					}
					robots[k].rzQueue.erase(robots[k].rzQueue.begin() + count - 1, robots[k].rzQueue.end());

					// We'll iterate backward to safely remove elements without invalidating indices
					for (auto it = robots[k].plan.rbegin(); it != robots[k].plan.rend(); ++it) {
						if ((*it).tq1 == r && (*it).tq3 > 0) {
							robots[k].plan.erase((it + 1).base());
						}
					}
				}
				else if (bins[k] == bins[r] && !checkSum(robots[k].dfwm, r).first) {
					// Step 1: Find positions where i >= whenFail elements
					vector<int> positions;
					for (size_t idx = 0; idx < whenFail.size(); ++idx) {
						if (step >= whenFail[idx]) {
							positions.push_back(idx);
						}
					}

					// Step 2: Filter whoFail based on these positions and check if any of the filtered values equals r
					bool anyMatch = false;
					for (int pos : positions) {
						if (whoFail[pos] == r) {
							anyMatch = true;
							break; // Stop searching once a match is found
						}
					}
					vector<double> tmpVec(2), tmpVec2(2);
					if (!robots[r].plan.empty()) {
						tmpVec[0] = robots[r].plan[0].x; tmpVec[1] = robots[r].plan[0].y;
					}
					tmpVec2[0] = robots[r].depot[0].x(); tmpVec2[1] = robots[r].depot[0].y();

					if (anyMatch && !robots[r].taskQueue.empty() && !isEqual(tmpVec, tmpVec2) && robots[k].deadBots[r] < 1) {
						robots[k].replan = 2;
						robots[k].dfwm.push_back(r);
						sort(robots[k].dfwm.begin(), robots[k].dfwm.end());
						// Step 3: Remove duplicates
						auto last = unique(robots[k].dfwm.begin(), robots[k].dfwm.end());
						robots[k].dfwm.erase(last, robots[k].dfwm.end());

						if (robots[r].max_v == 0) {
							robots[k].deadBots[r] = 1;
							findLiveAgents(bins, k, opBots);
							robots[k].replan = 2;
						}
					}

					if (calculateNorm(robots[k].pose, robots[k].depot[0]) < robots[k].maxRange * 2) { // HARDCODED FOR ONLY 1 DEPOT
						mergeUnique(plannedForAtDepot, conBots, plannedForAtDepot);
					}
				}

				if (calculateNorm(robots[k].pose, robots[k].depot[0]) < robots[k].maxRange * 2 && !checkSum(plannedForAtDepot, r).first) { // HARDCODED FOR ONLY 1 DEPOT
					mergeUnique(plannedForAtDepot, conBots, plannedForAtDepot);
				}
			}
		}

		vector<int> whichOp;
		std::vector<std::vector<int>> tmp(conBots.size());

		for (int ii = 0; ii < conBots.size(); ++ii) {
			int robotIndex = conBots[ii];
			tmp[ii].resize(robots[robotIndex].deadBots.size());
			for (int jj = 0; jj < robots[robotIndex].deadBots.size(); ++jj) {
				tmp[ii][jj] = robots[robotIndex].deadBots[jj];
			}
		}

		vector<int> tmp2(robots[0].deadBots.size());
		for (int ii = 0; ii < robots[0].deadBots.size(); ii++) {
			for (int jj = 0; jj < tmp.size(); jj++) {
				tmp2[ii] += tmp[jj][ii];
			}
		}

		for (int ii = 0; ii < tmp2.size(); ii++) {
			if (tmp2[ii] < 1) {
				whichOp.push_back(ii);
			}
		}

		// Don't replan redundantly
		if (isEqual(conBots, whichOp) && robots[k].connected_replan == 0 && isThereDeadBots()) {
			robots[k].replan = 2;
			for (auto const j : conBots) {
				robots[j].connected_replan = 1;
			}
		}
		else if (!isEqual(conBots, whichOp)) {
			for (auto const j : conBots) {
				robots[j].connected_replan = 0;
			}
		}
	}

	int sum = 0;
	for (int idx : conBots) {
		sum += robots[idx].replan;
	}

	if (sum < 1) {
		for (int a1 = 0; a1 < conBots.size(); a1++) {
			if (!robots[conBots[a1]].dfwm.empty()) {
				for (int a2 = 0; a2 < conBots.size(); a2++) {
					if (robots[conBots[a2]].commQueue.empty() && robots[conBots[a2]].rzQueue.empty()) {
						vector<int> tmpVec;
						for (int a3 = 0; a3 < conBots.size(); a3++) {
							tmpVec.insert(tmpVec.end(), robots[conBots[a3]].dfwm.begin(), robots[conBots[a3]].dfwm.end());
						}
						sort(tmpVec.begin(), tmpVec.end());
						auto last = unique(tmpVec.begin(), tmpVec.end());
						tmpVec.erase(last, tmpVec.end());
						robots[conBots[a2]].dfwm = tmpVec;
					}
				}
			}
		}
	}


}

void Algo::replan_check_mcts(vector<int>& conBots, vector<int>& opBots, int step, int k, MCTS& mcts) {

	Planning compute;
	vector<PResult> planningResults;
	PResult pR2;
	set<int> botsComm;
	vector<vector<double>> rzLoc;
	vector<double> minTimes;

	if (robots[k].replan == 1) {
		vector<int> deadBots;

		findConnectedDeadBots(conBots, deadBots);
		findMinTimes(step, k, conBots, deadBots, botsComm, rzLoc, minTimes);

		vector<vector<double>> lostLocs(robots[k].commQueue.size(), std::vector<double>(2, 0.0));

		for (int j = 0; j < robots[k].commQueue.size(); j++) {
			int whichBot = robots[k].commQueue[j];
			// is robot dead?
			if (robots[k].deadBots[whichBot] || robots[k].particleGuess[whichBot] > pIdx) {
				lostLocs[j].push_back(1e4); //Constant for now. Could replace with total dist of path or bounds of env
				lostLocs[j].push_back(1e4); //Constant for now. Could replace with total dist of path or bounds of env
			}
			else {
				if (!robots[k].particles[whichBot][robots[k].particleGuess[whichBot]].plan.empty()) {
					lostLocs[j] = calculateMeanPosition(robots[k].particles[whichBot][robots[k].particleGuess[whichBot]].plan);
				}
				else {
					lostLocs[j].push_back(robots[k].depot[0].x()); // HARDCODED DEPOT TO 1
					lostLocs[j].push_back(robots[k].depot[0].y()); // HARDCODED DEPOT TO 1
				}
			}
		}

		// Partition assignments if connected
		vector<vector<double>> taskCentroid(opBots.size(), std::vector<double>(2, 0.0));
		for (int ops = 0; ops < opBots.size(); ops++) {
			vector<Pose> whichPlan(robots[opBots[ops]].plan.size());
			for (int i = 0; i < robots[opBots[ops]].plan.size(); i++) {
				whichPlan[i].x = robots[opBots[ops]].plan[i].x;
				whichPlan[i].y = robots[opBots[ops]].plan[i].y;
			}
			whichPlan.push_back(robots[opBots[ops]].pose);
			taskCentroid[ops] = calculateMeanPosition(whichPlan);

		}
		taskDict newTaskDict;
		for (int ins = 0; ins < rzLoc.size(); ins++) {
			newTaskDict.one.push_back(rzLoc[ins][0]);
			newTaskDict.two.push_back(rzLoc[ins][1]);
		}

		for (int ins = 0; ins < lostLocs.size(); ins++) {
			newTaskDict.one.push_back(lostLocs[ins][0]);
			newTaskDict.two.push_back(lostLocs[ins][1]);
		}

		newTaskDict.three.insert(newTaskDict.three.end(), botsComm.begin(), botsComm.end());
		newTaskDict.three.insert(newTaskDict.three.end(), robots[k].commQueue.begin(), robots[k].commQueue.end());

		newTaskDict.four = vector<int>(newTaskDict.three.size(), -1);

		if (!botsComm.empty()) {
			newTaskDict.five = minTimes;;
		}

		vector<int> partition(newTaskDict.one.size(), 0);
		vector<Pose> new_loc(conBots.size());
		for (int pp = 0; pp < conBots.size(); pp++) {
			new_loc[pp] = robots[conBots[pp]].pose;
		}
		for (int tNew = 0; tNew < newTaskDict.one.size(); tNew++) {
			newTaskDict.four[tNew] = robots[k].particleGuess[newTaskDict.three[tNew]];
			vector<double> cost(opBots.size());
			vector<double> timeForTasks(cost.size());

			int assIdx = computePartition(newTaskDict, timeForTasks, new_loc, cost, opBots, tNew, k);
			partition[tNew] = conBots[assIdx];
			cost[assIdx] += timeForTasks[assIdx];

			if (tNew <= newTaskDict.three.size()) {
				vector<int> notAssigned = conBots;
				my_setdiff(notAssigned, conBots[assIdx]);
				for (int na = 0; na < notAssigned.size(); na++) {
					robots[notAssigned[na]].dfwm.push_back(newTaskDict.three[tNew]);
					robots[notAssigned[na]].dfwm.push_back(conBots[assIdx]);
				}
			}
			bool continue_plan = 0, full_replan = 0;
			int numMCTS_Iters = 1400;

			planningResults = compute.mcts_planning(robots, newTaskDict, partition, opBots, numMCTS_Iters, step, continue_plan, full_replan, mcts);

			for (int ops = 0; ops < opBots.size(); ops++) {
				Robot& robot = robots[opBots[ops]];
				vector<Plan> whichTasks;

				for (int i = 1; i < planningResults[opBots[ops]].best_tour.size(); i++) {
					update_robots(robot, planningResults[opBots[ops]], whichTasks, opBots, conBots, deadBots, ops);

					//// Remove entries in dfwm that are present in rzQueue
					//for (int q : robot.rzQueue) {
					//	auto it = std::find(robot.dfwm.begin(), robot.dfwm.end(), q);
					//	if (it != robot.dfwm.end()) { // If the element is found
					//		robot.dfwm.erase(it); // Remove the element
					//	}
					//}

					//// Remove entries in dfwm that are present in commQueue
					//for (int q : robot.commQueue) {
					//	auto it = std::find(robot.dfwm.begin(), robot.dfwm.end(), q);
					//	if (it != robot.dfwm.end()) { // If the element is found
					//		robot.dfwm.erase(it); // Remove the element
					//	}
					//}

					//If assigned a robot that is dead, adjust plan to account for backtracking
					if (ifAnyGreaterVector(robot.deadBots, robot.commQueue, 0)) {

						insertPlan(robot, whichTasks);

					}
				}
			}

			for (int r1 = 0; r1 < conBots.size(); r1++) {
				for (int r2 = 0; r2 < conBots.size(); r2++) {
					if (r1 != r2) {
						for (int p = 0; p < numParticles; p++) {
							robots[conBots[r1]].particles[conBots[r2]][p] = robots[conBots[r2]];
						}
					}
				}
			}
		}
	}
	else if (robots[k].replan == 2 && !opBots.empty()) {

		std::vector<int> tasksToDo;
		for (int index : conBots) {
			tasksToDo.insert(tasksToDo.end(), robots[index].taskQueue.begin(), robots[index].taskQueue.end());
		}
		std::sort(tasksToDo.begin(), tasksToDo.end());
		tasksToDo.erase(std::unique(tasksToDo.begin(), tasksToDo.end()), tasksToDo.end());

		std::vector<int> tasksDone;
		for (int index : conBots) {
			tasksDone.insert(tasksDone.end(), robots[index].tasks_done.begin(), robots[index].tasks_done.end());
		}

		tasksToDo = my_setdiff(tasksToDo, tasksDone);

		vector<int> deadBots;
		findConnectedDeadBots(conBots, deadBots);

		if (!tasksToDo.empty() && tasksToDo.size() != (robots[0].globalTaskLocs.size() + robots[0].depot.size())) {
			findMinTimes(step, k, conBots, deadBots, botsComm, rzLoc, minTimes);
		}

		vector<pair<double, double>> lostLocs(robots[k].commQueue.size(), { 0, 0 });

		for (int j = 0; j < robots[k].commQueue.size(); j++) {
			auto whichBot = robots[k].commQueue[j];
			bool ded = robots[k].deadBots[whichBot] || robots[k].particleGuess[whichBot] > pIdx;

			if (ded) {
				lostLocs[j].first = 1e4; lostLocs[j].second = 1e4; //Constant for now. Could replace with total dist of path or bounds of env
			}
			else {
				auto tmpCalc = calculateMeanPosition(robots[k].particles[whichBot][robots[k].particleGuess[whichBot]].plan);
				lostLocs[j].first = tmpCalc[0]; lostLocs[j].second = tmpCalc[1];
			}
		}
		taskDict newTaskDict, newTD;
		for (int ins = 0; ins < rzLoc.size(); ins++) {
			newTaskDict.one.push_back(rzLoc[ins][0]);
			newTaskDict.two.push_back(rzLoc[ins][1]);
		}

		for (int ins = 0; ins < lostLocs.size(); ins++) {
			newTaskDict.one.push_back(lostLocs[ins].first);
			newTaskDict.two.push_back(lostLocs[ins].second);
		}
		vector<int> newTaskBots;
		newTaskBots.insert(newTaskBots.end(), botsComm.begin(), botsComm.end());
		newTaskBots.insert(newTaskBots.end(), robots[k].commQueue.begin(), robots[k].commQueue.end());
		newTaskDict.three.insert(newTaskDict.three.end(), newTaskBots.begin(), newTaskBots.end());
		newTaskDict.four = vector<int>(newTaskDict.three.size(), -1);

		if (!botsComm.empty()) {
			newTaskDict.five = minTimes;
			for (int i = 0; i < newTaskDict.four.size() - newTaskDict.five.size(); i++) {
				newTaskDict.five.push_back(0);
			}
		}
		else {
			int sizeFive = newTaskDict.five.size();
			for (int i = 0; i < newTaskDict.four.size() - sizeFive; i++) {
				newTaskDict.five.push_back(0);
			}
		}

		for (int tNew = 0; tNew < newTaskDict.one.size(); tNew++) {
			newTaskDict.four[tNew] = robots[k].particleGuess[newTaskDict.three[tNew]];
		}

		// add all tasks (goal)
		for (int i = 0; i < tasksToDo.size(); i++) {
			newTaskDict.one.push_back(robots[k].globalTaskLocs[tasksToDo[i]][0]);
			newTaskDict.two.push_back(robots[k].globalTaskLocs[tasksToDo[i]][1]);
			newTaskDict.three.push_back(tasksToDo[i]);
			newTaskDict.four.push_back(-1);
			newTaskDict.five.push_back(0);
		}

		/* -- Vornoi weighted clustering -- */
		vector<int> partition(newTaskDict.one.size(), 0);
		vector<double> cost(conBots.size());
		vector<Pose> new_loc(conBots.size());
		for (int pp = 0; pp < conBots.size(); pp++) {
			new_loc[pp] = robots[conBots[pp]].pose;
		}

		vector<double> timeForTasks(cost.size());
		for (int tNew = 0; tNew < newTaskDict.one.size(); tNew++) {
			int assIdx = computePartition(newTaskDict, timeForTasks, new_loc, cost, conBots, tNew, k);
			partition[tNew] = conBots[assIdx];
			cost[assIdx] += timeForTasks[assIdx];

			if (tNew < newTaskBots.size()) {
				vector<int> notAssigned = conBots;
				my_setdiff(notAssigned, conBots[assIdx]);
				for (int na = 0; na < notAssigned.size(); na++) {
					robots[notAssigned[na]].dfwm.push_back(newTaskBots[tNew]);
					robots[notAssigned[na]].dfwm.push_back(conBots[assIdx]);
				}
			}
		}

		vector<vector<int>> tasks2do(opBots.size());
		vector<vector<int>> prec(numBots);

		for (int ops = 0; ops < opBots.size(); ops++) {
			vector<int> deadBotsIndices, path_left, total_path;
			Robot& robot = robots[opBots[ops]];

			for (int idx : robot.commQueue) {
				if (robot.deadBots[idx] > 0) {
					deadBotsIndices.push_back(idx);
				}
			}

			for (int b : deadBotsIndices) {
				if (robot.particleLastUpdate[b] < step) {
					continue;
				}

				auto particle = robot.particles[b].back();  // Access the last particle
				vector<int> calcParticleQueue, orderedPath;
				for (int i = 0; i < robot.plan.size(); i++) {

					if (robot.plan[i].tq1 >= 0) {
						calcParticleQueue.push_back(robot.plan[i].tq1);
					}

					if (robot.plan[i].order > 0) {
						orderedPath.push_back(robot.plan[i].tq1);
					}
				}

				std::set<int> unionSet(calcParticleQueue.begin(), calcParticleQueue.end());
				unionSet.insert(particle.taskQueue.begin(), particle.taskQueue.end());

				// Convert the set back to a vector
				std::vector<int> unionVector(unionSet.begin(), unionSet.end());
				auto path_left2 = my_setdiff(unionVector, orderedPath);

				taskDict newTaskDict;
				for (int ii = 0; ii < path_left2.size(); ii++) {
					newTaskDict.one.push_back(robot.globalTaskLocs[path_left2[ii]].x());
					newTaskDict.two.push_back(robot.globalTaskLocs[path_left2[ii]].y());
					newTaskDict.three.push_back(path_left2[ii]);
					newTaskDict.four.push_back(-1);
					newTaskDict.five.push_back(0);
				}
				/* -- Vornoi weighted clustering -- */
				vector<int> partition(newTaskDict.one.size(), 0);
				vector<double> cost(conBots.size());
				vector<Pose> new_loc(conBots.size());
				for (int pp = 0; pp < conBots.size(); pp++) {
					new_loc[pp] = robots[conBots[pp]].pose;
				}

				vector<double> timeForTasks(cost.size());
				for (int tNew = 0; tNew < newTaskDict.one.size(); tNew++) {
					int assIdx = computePartition(newTaskDict, timeForTasks, new_loc, cost, conBots, tNew, k);
					partition[tNew] = conBots[assIdx];
					cost[assIdx] += timeForTasks[assIdx];
				}

				path_left = my_setdiff(particle.originalPlan, particle.taskQueue);
				reverse(path_left.begin(), path_left.end());

				if (!path_left.empty()) {
					prec[b] = path_left;
					total_path.insert(total_path.end(), path_left.begin(), path_left.end());
				}

				if (!orderedPath.empty()) {
					prec[opBots[ops]] = orderedPath;
					total_path.insert(total_path.end(), orderedPath.begin(), orderedPath.end());
				}

				if (deadBotsIndices.back() == b) {
					int value = partition.size() - path_left2.size();
					for (int c = 0; c < conBots.size(); c++) {
						for (int ii = 0; ii < path_left2.size(); ii++) {
							if (opBots[c] == partition[ii + value]) {
								tasks2do[c].push_back(path_left2[ii]);
							}
						}
					}
				}
			}

			if (deadBotsIndices.empty()) {
				for (int i = 0; i < robot.plan.size(); i++) {
					tasks2do[ops].push_back(robot.plan[i].tq1);
				}
			}

			tasks2do[ops].insert(tasks2do[ops].end(), total_path.begin(), total_path.end());

			if (!tasks2do[ops].empty()) {
				
				auto tour = compute.mdutec(1, tasks2do[ops], robot, prec, rzLoc);

				robot.plan.clear();
				for (int ins = 0; ins < rzLoc.size(); ins++) {
					std::set<int>::iterator it = botsComm.begin();

					Plan plan;
					plan.x = rzLoc[ins][0];
					plan.y = rzLoc[ins][1];
					plan.tq1 = -1;
					plan.tq2 = *it;
					plan.tq3 = minTimes[ins];
					plan.order = 0;

					robot.plan.push_back(plan);

					std::advance(it, ins + 1); // Move the iterator to the third element (0-based index)
				}

				for (int i = 0; i < tour.size(); i++) {
					Plan plan;
					plan.x = robot.globalTaskLocs[tour[i]].x();
					plan.y = robot.globalTaskLocs[tour[i]].y();
					plan.tq1 = tour[i];
					plan.tq2 = -1;
					plan.tq3 = 0;

					for (int ii = 0; ii < prec.size(); ii++) {
						auto flag = std::find(prec[ii].begin(), prec[ii].end(), tour[i]) != prec[ii].end();

						if (flag) {
							plan.order = 1;
						}

					}
					robot.plan.push_back(plan);
				}

				int cnt = 0;
				for (const int bot : botsComm) {
					double time = 0;
					auto rzLoc = findRz(robot, robot.particles[bot][0], robot.pose, step * robot.dt, time); // HARDCODED 1 particle only
					robot.plan[cnt].x = rzLoc.x;
					robot.plan[cnt].y = rzLoc.y;
					robot.plan[cnt].tq3 = time;
					cnt++;
				}
			}
		}

		for (int ops = 0; ops < opBots.size(); ops++) {
			Robot& robot = robots[opBots[ops]];

			for (int dead : deadBots) {
				robot.deadBots[dead] = 1;
			}

			// Update other queues based on conditions
			robot.taskQueue.clear();
			robot.rzQueue.clear();
			//robot.commQueue.clear();

			for (size_t i = 0; i < robot.plan.size(); ++i) {
				if (robot.plan[i].tq2 < 0) { //if (whichTasks.four[i] < 1) {
					robot.taskQueue.push_back(robot.plan[i].tq1);
				}
				if (robot.plan[i].tq3 > 0) {
					robot.rzQueue.push_back(robot.plan[i].tq1);
				}
			}

				//// Remove entries in dfwm that are present in rzQueue
			for (int q : robot.rzQueue) {
				auto it = std::find(robot.dfwm.begin(), robot.dfwm.end(), q);
				if (it != robot.dfwm.end()) { // If the element is found
					robot.dfwm.erase(it); // Remove the element
				}
			}

			// Remove entries in dfwm that are present in commQueue
			for (int q : robot.commQueue) {
				auto it = std::find(robot.dfwm.begin(), robot.dfwm.end(), q);
				if (it != robot.dfwm.end()) { // If the element is found
					robot.dfwm.erase(it); // Remove the element
				}
			}
		}

		for (int r1 = 0; r1 < conBots.size(); r1++) {
			for (int r2 = 0; r2 < conBots.size(); r2++) {
				if (r1 != r2) {
					for (int p = 0; p < numParticles; p++) {
						copyRobot2Particle(robots[conBots[r2]], robots[conBots[r1]].particles[conBots[r2]][p], p);
					}
				}
			}
		}

		//replanOriginal(conBots, opBots, step, k, mcts); THIS IS THE MATLAB VERSION OF REPLANNING;
	}

	if (robots[k].replan > 0) {
		for (int a = 0; a < conBots.size(); a++) {
			robots[conBots[a]].replan = 0;
		}
	}

	if (robots[k].plan.empty()) {
		robots[k].state = "Do Depot";
	}
	else {
		robots[k].state = "Do Tasks";
	}
}

void Algo::goal_check_mcts(int step, int k, double goalReached) {

	if (robots[k].plan.empty()) {
		robots[k].goalLoc.x = robots[k].depot[0].x(); // HARDCODED DEPOT TO 1
		robots[k].goalLoc.y = robots[k].depot[0].y(); // HARDCODED DEPOT TO 1
		robots[k].state = "Do Depot";
	}
	else {
		robots[k].goalLoc.x = robots[k].plan[0].x;
		robots[k].goalLoc.y = robots[k].plan[0].y;

		bool isTask = robots[k].plan[0].tq2 == -1;
		bool isRz = robots[k].plan[0].tq3 > 0;
		bool isComm = !isTask & !isRz;

		if (isTask) { robots[k].state = "Do Tasks"; }
		else if (isRz) { robots[k].state = "Do Rz"; }
		else if (isComm) { robots[k].state = "Do Comm"; }
	}

	vector<double> distToTasks;
	for (const auto& taskLoc : robots[k].globalTaskLocs) {
		distToTasks.push_back(calculateNorm(robots[k].pose, taskLoc));
	}
	//|| calculateNorm(robots[k].pose, robots[k].plan[0]) < goalReached
	if (ifAnySmaller(distToTasks, goalReached)) {
		vector<int> whichClose;
		for (size_t i = 0; i < distToTasks.size(); ++i) {
			if (distToTasks[i] < goalReached + 0.2) {
				whichClose.push_back(i);
			}
		}
		mergeUnique(robots[k].tasks_done, whichClose, robots[k].tasks_done);
		if (whichClose.size() == 1) {
			robots[k].taskReached.push_back(make_pair(step, whichClose[0]));
		}
		else {
			cout << "whichClose has more than 1 element. fix this." << endl;
		}
		// Color check, this needs implementation
		/*
		// Check if any element in taskColorStore at indices specified by whichClose is < 1
		bool shouldUpdate = false;
		for (size_t index : whichClose) {
			if (taskColorStore[index] < 1) {
				shouldUpdate = true;
				break; // No need to check further once we know an update is needed
			}
		}

		// If the condition is true for any of those elements, update them
		if (shouldUpdate) {
			for (size_t index : whichClose) {
				taskColorStore[index] = k; // Update the specified elements to k
			}
		}
		*/
	}

	// Check if actually arrived at belief or movement error requires small perturbation
	// Plan is current location(1:2), robot index(3), particle(4), min rz time(5 - when applicable > 0)

	if (robots[k].state == "Do Rz" || robots[k].state == "Do Comm") {
		if (calculateNorm(robots[k].pose, robots[k].goalLoc) < goalReached) {
			auto whichBot = robots[k].plan[0].tq1;
			auto whichParticle = robots[k].plan[0].tq2;
			//int whichParticle = 0;
			if (whichParticle < numParticles) {
				auto pB = robots[k].particles[whichBot][whichParticle];
				auto whichLoc = robots[k].particles[whichBot][whichParticle].pose;
				if (calculateNorm(robots[k].pose, whichLoc) < robots[k].commRange - 2) {
					robots[k].plan.erase(robots[k].plan.begin()); // Removes the first row from the plan
				}
				else {
					/*auto velA = robots[k].max_v;
					auto velB = pB.max_v;*/
					if (pB.plan.empty()) {
						pB.plan.resize(1);
						pB.plan[0].x = robots[k].depot[0].x();
						pB.plan[0].y = robots[k].depot[0].y();
					}
					findIntersectionPoint(robots[k], whichLoc, pB, robots[k].max_v, pB.max_v, goalReached, step);

					//robots[k].goalLoc.x = 0; robots[k].goalLoc.y = 0;
					robots[k].goalLoc.x = robots[k].plan[0].x; robots[k].goalLoc.y = robots[k].plan[0].y;
				}
			}
			else {

				for (int r = 0; r < numBots; r++) {
					Plan tmpPlan;
					tmpPlan.x = robots[r].globalTaskLocs[robots[r].originalPlan[0]].x();
					tmpPlan.y = robots[r].globalTaskLocs[robots[r].originalPlan[0]].y();
					if ((robots[k].plan[0] == tmpPlan) && calculateNorm(robots[k].pose, robots[k].plan[0]) < robots[k].goalReached) {
						robots[k].deadBots[r] = 1;
						robots[k].replan = 2;
						robots[k].repFlag[r] = 1;
						auto tmptasks = my_setdiff(robots[r].taskQueue, robots[k].tasks_done);
						robots[k].taskQueue.insert(robots[k].taskQueue.end(), tmptasks.begin(), tmptasks.end());

					}
				}

				robots[k].plan.erase(robots[k].plan.begin());
				if (robots[k].plan.empty()) { // if isempty(robots(k).plan(:,3) == whichBot)

					robots[k].commQueue.erase(remove(robots[k].commQueue.begin(), robots[k].commQueue.end(), whichBot),
						robots[k].commQueue.end()); //commIdx = robots(k).commQueue == whichBot; robots(k).commQueue(commIdx) = [];
					vector<int> dummyVec = { whichBot };
					mergeUnique(robots[k].dfwm, dummyVec, robots[k].dfwm);
				}
			}
		}
		else if (hasMatchingElement(robots[k])) {
			updateGoal(robots[k], step);
		}
	}
	else if (calculateNorm(robots[k].pose, robots[k].goalLoc) < goalReached && robots[k].state != "Do Depot") {
		if (!robots[k].taskQueue.empty()) {
			robots[k].taskQueue.erase(robots[k].taskQueue.begin());
		}
		updateGoal(robots[k], step);
	}
	else if (!robots[k].plan.empty()) {
		/*if (hasMatchingElement(robots[k].tasks_done, robots[k].plan[0].tq1)) {
			updateGoal(robots[k], step);
		}*/

		while (!robots[k].plan.empty() && hasMatchingElement(robots[k].tasks_done, robots[k].plan[0].tq1)) {
			updateGoal(robots[k], step);
		}
	}

	if (!robots[k].reachedNewParticle) {
		robots[k].goalLoc.x = robots[k].particles[k][robots[k].particleFollow].pose.x;
		robots[k].goalLoc.y = robots[k].particles[k][robots[k].particleFollow].pose.y;
		if (calculateNorm(robots[k].pose, robots[k].goalLoc) < goalReached) {
			robots[k].reachedNewParticle = true;
		}
	}
}

void Algo::findIntersectionPoint(Robot& robot, Pose& locB, Robot pB, double velA, double velB, double goalReached, int step) {
	double radius = 0;
	//int taskCount = 1;
	int taskCount = 0;
	int numTasks = pB.plan.size();
	double startTime = robot.dt * step;
	double currTime = robot.dt * step;
	auto depot = robot.depot[0]; // HARDCODED DEPOT TO 1
	double timePast = 0;
	int numTimesInLoop = 0;

	// We assume locA is already in the right location but locB may need to
	// be simulated forward in time.If not, startTime = currTime

	while (calculateNorm(robot.pose, locB) >= (robot.maxRange * 2 - 2) * 2 - pB.commError) {  // this maxRange is weird, this is how it is in matlab
		auto distToTaskB = calculateNorm(locB, pB.plan[taskCount]);
		auto angleToTaskB = atan2(pB.plan[taskCount].y - locB.y, pB.plan[taskCount].x - locB.x);
		locB.x += cos(angleToTaskB) * velB * robot.dt;
		locB.y += sin(angleToTaskB) * velB * robot.dt;

		if ((startTime + timePast) >= currTime) {
			radius += velA * robot.dt;
		}
		if (distToTaskB < goalReached && taskCount < numTasks) {
			taskCount++;
		}
		else if (distToTaskB < goalReached && taskCount >= numTasks) {
			pB.plan[0].x = pB.depot[0].x(); pB.plan[0].y = pB.depot[0].y();
			//taskCount = 1; numTasks = 1;
			taskCount = 0; numTasks = 1;
		}

		timePast += robot.dt;
		numTimesInLoop++;

		if (numTimesInLoop > 1e8)
			cout << "numTimesInLoop > 1e8" << endl;
		return;
	}

	auto angleToGoal = atan2(locB.y - robot.pose.y, locB.x - robot.pose.x);
	robot.plan[0].x = robot.pose.x + cos(angleToGoal) * radius;
	robot.plan[0].y = robot.pose.y + sin(angleToGoal) * radius;

	return;
}

void Algo::findMinTimes(int step, int k, vector<int>& conBots, vector<int>& deadBots, set<int>& botsComm, vector<vector<double>>& rzLoc, vector<double>& minTimes) {

	vector<vector<double>> windowMins;

	if (!timeWindows.empty()) {
		std::set<int> botsCovered;
		botsCovered.insert(conBots.begin(), conBots.end());
		botsCovered.insert(robots[k].commQueue.begin(), robots[k].commQueue.end()); // Adjust based on your actual robots structure
		botsCovered.insert(deadBots.begin(), deadBots.end());

		for (const auto& row : timeWindows) {
			if (row[4] > step * robots[k].dt) { // Columns are zero-indexed 
				windowMins.push_back(row);
			}
		}

		std::vector<int> commsIdx(windowMins.size());
		for (size_t i = 0; i < windowMins.size(); ++i) {
			int botId = static_cast<int>(windowMins[i][2]);
			commsIdx[i] = (botsCovered.find(botId) != botsCovered.end()) ? 1 : 0;
		}

		std::set<int> botsInWindows;
		for (const auto& idx : windowMins) {
			botsInWindows.insert(static_cast<int>(idx[3])); // Assuming column 4 holds the relevant bot IDs
		}


		std::set_difference(botsInWindows.begin(), botsInWindows.end(),
			botsCovered.begin(), botsCovered.end(),
			std::inserter(botsComm, botsComm.end()));

	}

	minTimes.resize(botsComm.size());
	rzLoc.resize(botsComm.size());

	int i = 0;
	for (const int bot : botsComm) {
		double minTime = std::numeric_limits<double>::max();
		size_t minIdx = 0;
		bool found = false;

		for (size_t j = 0; j < windowMins.size(); ++j) {
			if (windowMins[j][2] == bot) { // Checking the third column for match
				if (windowMins[j][4] < minTime) { // Checking the fifth column for minimum
					minTime = windowMins[j][4];
					minIdx = j;
					found = true;
				}
			}
		}

		if (found) {
			minTimes[i] = minTime;
			rzLoc[i] = { windowMins[minIdx][0], windowMins[minIdx][1] }; // Assuming these are x, y positions
		}
		else {
			minTimes[i] = -1; // Indicating not found or not applicable
			rzLoc[i] = { 0, 0 }; // Default position if not found
		}
		i++;
	}
}

// compute_partition(robots(k),robot_speeds,robot_locs,tLoc,cost);
int Algo::computePartition(taskDict& newTaskDict, vector<double>& timeForTask, vector<Pose>& new_loc, vector<double>& cost, vector<int>& opBots, int tNew, int k) {
	vector<int> taskLeft;
	Pose task;
	task.x = newTaskDict.one[tNew]; task.y = newTaskDict.two[tNew];

	for (int i = 0; i < cost.size(); i++) {
		if ((newTaskDict.one[tNew] + newTaskDict.two[tNew]) > 1e3) {
			taskLeft = my_setdiff(robots[k].particles[newTaskDict.three[tNew]].back().originalPlan, robots[k].particles[newTaskDict.three[tNew]].back().taskQueue);
			timeForTask[i] = cost[i] + computeTourLength(robots[opBots[i]], new_loc[i], taskLeft);
		}
		else {
			timeForTask[i] = cost[i] + calculateNorm(task, new_loc[i]) / (robots[opBots[i]].max_v + 1e-4);
		}
	}

	// Find the iterator to the minimum element in the vector
	auto minIt = std::min_element(timeForTask.begin(), timeForTask.end());

	// Calculate the index of the minimum element
	int assIdx = std::distance(timeForTask.begin(), minIt);

	new_loc[assIdx] = task;

	return assIdx;
}

double Algo::computeTourLength(Robot& robot, Pose& pose, vector<int>& tour) {
	double time = 0;
	Pose current_loc = pose;
	Pose nextCityLoc;
	if (!tour.empty()) {
		for (int i = 0; i < tour.size() - 1; i++) {
			nextCityLoc.x = robot.globalTaskLocs[tour[i]].x();
			nextCityLoc.y = robot.globalTaskLocs[tour[i]].y();
			time += calculateNorm(nextCityLoc, current_loc) / robot.max_v;
			current_loc = nextCityLoc;
		}
	}
	return time;
}

void Algo::insertPlan(Robot& robot, vector<Plan>& whichTasks) {
	std::vector<int> deadBotsIndices;

	// Find indices of dead bots in communication queue
	for (int idx : robot.commQueue) {
		if (robot.deadBots[idx] > 0) {
			deadBotsIndices.push_back(idx);
		}
	}

	for (int b : deadBotsIndices) {

		if (robot.repFlag[b] == 0) {
			int insertPlace;
			for (size_t i = 0; i < whichTasks.size(); ++i) {
				if (whichTasks[i].tq1 == b && whichTasks[i].tq2 >= 0) {
					insertPlace = i;
				}
			}

			//insertPlace should always be the same as whichPlan id?

			auto particle = robot.particles[b].back();  // Access the last particle
			vector<int> path_left = my_setdiff(particle.originalPlan, particle.taskQueue);
			reverse(path_left.begin(), path_left.end());
			int _size = path_left.size() + robot.plan.size();
			robot.plan.resize(_size);


			int afterIdx = 0;
			for (int i = 0; i < path_left.size(); i++) {
				afterIdx = insertPlace + i;
				robot.plan[afterIdx].x = particle.globalTaskLocs[path_left[i]].x();
				robot.plan[afterIdx].y = particle.globalTaskLocs[path_left[i]].y();
				robot.plan[afterIdx].tq1 = whichTasks[insertPlace].tq1;
				robot.plan[afterIdx].tq2 = whichTasks[insertPlace].tq2;
				robot.plan[afterIdx].tq3 = whichTasks[insertPlace].tq3;
				robot.plan[afterIdx].id = path_left[i];
			}

			for (int i = insertPlace + 1; i < whichTasks.size(); i++) {
				afterIdx++;
				robot.plan[afterIdx] = whichTasks[i];
			}

			for (auto it = robot.plan.size() - 1; it >= 0; it--) {
				if (robot.plan[it].x == 0 &&
					robot.plan[it].y == 0 &&
					robot.plan[it].tq1 == 0 &&
					robot.plan[it].tq2 == 0 &&
					robot.plan[it].tq3 == 0 &&
					robot.plan[it].id == 0)
				{
					robot.plan.pop_back();

				}
				else {
					break;
				}

			}

			whichTasks = robot.plan;
		}
	}
}

// Helper functions

void Algo::updateGoal(Robot& robot, int step) {

	robot.plan.erase(robot.plan.begin()); // Removes the first row from the plan
	if (!robot.plan.empty()) {
		robot.goalLoc.x = robot.plan[0].x;
		robot.goalLoc.y = robot.plan[0].y;
	}
	else {
		robot.goalLoc.x = robot.depot[0].x(); // HARDCODED DEPOT TO 1
		robot.goalLoc.y = robot.depot[0].y(); // HARDCODED DEPOT TO 1
	}
}

void Algo::update_robots(Robot& robot, PResult& planningResults, vector<Plan>& whichTasks, vector<int>& opBots, vector<int>& conBots, vector<int>& deadBots, int ops)
{

	std::vector<int> selected_indices(planningResults.best_tour.begin() + 1, planningResults.best_tour.end());  // ignoring the first element

	for (int dead : deadBots) {
		robot.deadBots[dead] = 1;
	}

	// --- TO DO: I can ignore whichTasks and put this directly into robot.plan. It would speed up the process -- \\

	robot.root = planningResults.root[ops];

	// Assuming each task index in best_tour corresponds directly to the task in tasksAssigned
	int cnt = 0;
	whichTasks.resize(selected_indices.size());

	for (int index : selected_indices) {
		whichTasks[cnt].x = planningResults.tasksAssigned[ops].one[index];
		whichTasks[cnt].y = planningResults.tasksAssigned[ops].two[index];
		whichTasks[cnt].tq1 = planningResults.tasksAssigned[ops].three[index];
		whichTasks[cnt].tq2 = planningResults.tasksAssigned[ops].four[index];
		whichTasks[cnt].tq3 = planningResults.tasksAssigned[ops].five[index];
		cnt++;
	}

	// Update positions based on tour_states
	for (size_t i = 0; i < whichTasks.size(); ++i) {
		if (i < planningResults.tour_states.size() - 1) {  // Prevent out-of-bounds error
			whichTasks[i].x = planningResults.tour_states[i + 1].x;
			whichTasks[i].y = planningResults.tour_states[i + 1].y;
		}
	}

	robot.plan = whichTasks;
	// Update other queues based on conditions
	robot.taskQueue.clear();
	robot.rzQueue.clear();
	robot.commQueue.clear();

	for (size_t i = 0; i < whichTasks.size(); ++i) {
		if (whichTasks[i].tq2 < 0) { //if (whichTasks.four[i] < 1) {
			robot.taskQueue.push_back(whichTasks[i].tq1);
		}
		if (whichTasks[i].tq3 > 0) {
			robot.rzQueue.push_back(whichTasks[i].tq1);
		}
		if (whichTasks[i].tq2 > 0 && whichTasks[i].tq3 < 1) {
			robot.commQueue.push_back(whichTasks[i].tq1);
		}
	}

	// Update dfwm by merging with conBots and ensuring uniqueness
	std::set<int> dfwmSet(robot.dfwm.begin(), robot.dfwm.end());
	dfwmSet.insert(conBots.begin(), conBots.end());  // Insert contributing bots
	robot.dfwm.assign(dfwmSet.begin(), dfwmSet.end());  // Assign back to vector from set
}

void Algo::update_robots2(Robot& robot, vector<int>& deadBots, vector<int>& conBots)
{

	//std::vector<int> selected_indices(planningResults.best_tour.begin() + 1, planningResults.best_tour.end());  // ignoring the first element

	for (int dead : deadBots) {
		robot.deadBots[dead] = 1;
	}

	// --- TO DO: I can ignore whichTasks and put this directly into robot.plan. It would speed up the process -- \\

	// Update other queues based on conditions
	robot.taskQueue.clear();
	robot.rzQueue.clear();
	robot.commQueue.clear();

	for (size_t i = 0; i < robot.plan.size(); ++i) {
		if (robot.plan[i].tq2 < 0) { //if (whichTasks.four[i] < 1) {
			robot.taskQueue.push_back(robot.plan[i].tq1);
		}
		if (robot.plan[i].tq3 > 0) {
			robot.rzQueue.push_back(robot.plan[i].tq1);
		}
		if (robot.plan[i].tq2 > 0 && robot.plan[i].tq3 < 1) {
			robot.commQueue.push_back(robot.plan[i].tq1);
		}
	}

	// Update dfwm by merging with conBots and ensuring uniqueness
	std::set<int> dfwmSet(robot.dfwm.begin(), robot.dfwm.end());
	dfwmSet.insert(conBots.begin(), conBots.end());  // Insert contributing bots
	robot.dfwm.assign(dfwmSet.begin(), dfwmSet.end());  // Assign back to vector from set
}

void Algo::replanOriginal(vector<int>& conBots, vector<int>& opBots, int step, int k, MCTS& mcts) {

	Planning compute;
	vector<PResult> planningResults;
	PResult pR2;
	set<int> botsComm;
	vector<vector<double>> rzLoc;
	vector<double> minTimes;

	std::vector<int> tasksToDo;
	for (int index : conBots) {
		tasksToDo.insert(tasksToDo.end(), robots[index].taskQueue.begin(), robots[index].taskQueue.end());
	}
	std::sort(tasksToDo.begin(), tasksToDo.end());
	tasksToDo.erase(std::unique(tasksToDo.begin(), tasksToDo.end()), tasksToDo.end());

	std::vector<int> tasksDone;
	for (int index : conBots) {
		tasksDone.insert(tasksDone.end(), robots[index].tasks_done.begin(), robots[index].tasks_done.end());
	}

	tasksToDo = my_setdiff(tasksToDo, tasksDone);

	vector<int> deadBots;
	findConnectedDeadBots(conBots, deadBots);

	if (!tasksToDo.empty() && tasksToDo.size() != (robots[0].globalTaskLocs.size() + robots[0].depot.size())) {
		findMinTimes(step, k, conBots, deadBots, botsComm, rzLoc, minTimes);
	}

	vector<pair<double, double>> lostLocs(robots[k].commQueue.size(), { 0, 0 });

	for (int j = 0; j < robots[k].commQueue.size(); j++) {
		auto whichBot = robots[k].commQueue[j];
		bool ded = robots[k].deadBots[whichBot] || robots[k].particleGuess[whichBot] > pIdx;

		if (ded) {
			lostLocs[j].first = 1e4; lostLocs[j].second = 1e4; //Constant for now. Could replace with total dist of path or bounds of env
		}
		else {
			auto tmpCalc = calculateMeanPosition(robots[k].particles[whichBot][robots[k].particleGuess[whichBot]].plan);
			lostLocs[j].first = tmpCalc[0]; lostLocs[j].second = tmpCalc[1];
		}
	}
	taskDict newTaskDict, newTD;
	for (int ins = 0; ins < rzLoc.size(); ins++) {
		newTaskDict.one.push_back(rzLoc[ins][0]);
		newTaskDict.two.push_back(rzLoc[ins][1]);
	}

	for (int ins = 0; ins < lostLocs.size(); ins++) {
		newTaskDict.one.push_back(lostLocs[ins].first);
		newTaskDict.two.push_back(lostLocs[ins].second);
	}

	newTaskDict.three.insert(newTaskDict.three.end(), botsComm.begin(), botsComm.end());
	newTaskDict.three.insert(newTaskDict.three.end(), robots[k].commQueue.begin(), robots[k].commQueue.end());

	newTaskDict.four = vector<int>(newTaskDict.three.size(), -1);

	if (!botsComm.empty()) {
		newTaskDict.five = minTimes;
		for (int i = 0; i < newTaskDict.four.size() - newTaskDict.five.size(); i++) {
			newTaskDict.five.push_back(0);
		}
	}
	else {
		int sizeFive = newTaskDict.five.size();
		for (int i = 0; i < newTaskDict.four.size() - sizeFive; i++) {
			newTaskDict.five.push_back(0);
		}
	}

	for (int tNew = 0; tNew < newTaskDict.one.size(); tNew++) {
		newTaskDict.four[tNew] = robots[k].particleGuess[newTaskDict.three[tNew]];
	}

	// add all tasks (goal)
	for (int i = 0; i < tasksToDo.size(); i++) {
		newTaskDict.one.push_back(robots[k].globalTaskLocs[tasksToDo[i]][0]);
		newTaskDict.two.push_back(robots[k].globalTaskLocs[tasksToDo[i]][1]);
		newTaskDict.three.push_back(tasksToDo[i]);
		newTaskDict.four.push_back(-1);
		newTaskDict.five.push_back(0);
	}

	/* -- Vornoi weighted clustering -- */
	vector<int> partition(newTaskDict.one.size(), 0);
	vector<double> cost(conBots.size());
	vector<Pose> new_loc(conBots.size());
	for (int pp = 0; pp < conBots.size(); pp++) {
		new_loc[pp] = robots[conBots[pp]].pose;
	}

	vector<double> timeForTasks(cost.size());
	for (int tNew = 0; tNew < newTaskDict.one.size(); tNew++) {
		int assIdx = computePartition(newTaskDict, timeForTasks, new_loc, cost, conBots, tNew, k);
		partition[tNew] = conBots[assIdx];
		cost[assIdx] += timeForTasks[assIdx];

		if (tNew <= newTaskDict.three.size()) {
			vector<int> notAssigned = conBots;
			my_setdiff(notAssigned, conBots[assIdx]);
			for (int na = 0; na < notAssigned.size(); na++) {
				robots[notAssigned[na]].dfwm.push_back(newTaskDict.three[tNew]);
				robots[notAssigned[na]].dfwm.push_back(conBots[assIdx]);
			}
		}
	}

	bool continue_plan = 0, full_replan = 1;
	int numMCTS_Iters = 4000;
	planningResults = compute.mcts_planning(robots, newTaskDict, partition, opBots, numMCTS_Iters, step, continue_plan, full_replan, mcts);


	for (int ops = 0; ops < opBots.size(); ops++) {
		Robot& robot = robots[opBots[ops]];
		//	//taskDict whichTasks;
		vector<Plan> whichTasks;
		update_robots(robot, planningResults[opBots[ops]], whichTasks, opBots, conBots, deadBots, ops);

		//	update_robots(robot, planningResults[ops], whichTasks, opBots, conBots, deadBots, ops);
			//// Remove entries in dfwm that are present in rzQueue
		for (int q : robot.rzQueue) {
			auto it = std::find(robot.dfwm.begin(), robot.dfwm.end(), q);
			if (it != robot.dfwm.end()) { // If the element is found
				robot.dfwm.erase(it); // Remove the element
			}
		}

		// Remove entries in dfwm that are present in commQueue
		for (int q : robot.commQueue) {
			auto it = std::find(robot.dfwm.begin(), robot.dfwm.end(), q);
			if (it != robot.dfwm.end()) { // If the element is found
				robot.dfwm.erase(it); // Remove the element
			}
		}

		//If assigned a robot that is dead, adjust plan to account for backtracking
		if (ifAnyGreaterVector(robot.deadBots, robot.commQueue, 0)) {
			insertPlan(robot, whichTasks);
		}
	}

	for (int r1 = 0; r1 < conBots.size(); r1++) {
		for (int r2 = 0; r2 < conBots.size(); r2++) {
			if (r1 != r2) {
				for (int p = 0; p < numParticles; p++) {
					robots[conBots[r1]].particles[conBots[r2]][p] = robots[conBots[r2]];
				}
			}
		}
	}
}