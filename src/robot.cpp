#include "robot.h"

using namespace std;
using namespace numbers;

void Robot::initRobots(int _id, Problem& problem) {
	ID = _id;
	dt = 0.1;
	decay = 0.2;
	numBots = problem.robotLocs.size();
	pose.x = problem.robotLocs[_id].x();
	pose.y = problem.robotLocs[_id].y();
	pose.theta = numbers::pi / 2;
	pose_sim = pose;
	goalLoc.x = 0; goalLoc.y = 0;
	//Sensor parameters
	double step = numbers::pi / 16;
	for (double value = -numbers::pi; value <= pi; value += step) {
		angles.push_back(value);
	}
	maxRange = 5; // sensor max range
	commRange = 10;
	// Motion Parameters
	max_v = 5; // max velocity
	numP = 1; // Number of particles per robot, we assume all robots have the same number*
	for (int i = 0; i < numBots; i++) {
		numParticles.emplace_back(numP);
		particleLastUpdate.emplace_back(0);
		robotLastUpdate.emplace_back(0);
		deadBots.emplace_back(0);
		repFlag.emplace_back(0);
	}
	state = "Do Tasks";
	depot = problem.depots;
	// task order
	taskQueue = problem.robotPlans[_id];
	taskQueue.pop_back(); // remove the depot
	originalPlan = taskQueue;
	globalTaskLocs = problem.tasks;

	// plan order
	for (int i = 0; i < taskQueue.size(); i++) { //-1 to remove the depot
		Plan tmpPlan;

		tmpPlan.x = globalTaskLocs[taskQueue[i]][0];
		tmpPlan.y = globalTaskLocs[taskQueue[i]][1];
		tmpPlan.tq1 = taskQueue[i];
		tmpPlan.tq2 = -1; tmpPlan.tq3 = 0;
		tmpPlan.id = i;
		plan.emplace_back(tmpPlan);
	}
	reachedNewParticle = true;
	goalLoc.x = numeric_limits<double>::max();
	goalLoc.x = numeric_limits<double>::max();
	particleFollow = 1;
	particleGuess = numParticles;
	connected_replan = 0;
	replan = 0;
	iamdifferent = false;
	at_final_pos = false;
	commError = 0.1;
	planID = 1;

	obsDist.push_back(5);  //initialized to the value from matlab.

}

void Robot::initParticles(vector<Robot>& _robots, int k) {
	
	_robots[k].particles.resize(numBots);
	for (int r = 0; r < numBots; r++) {
		_robots[k].particles[r].resize(numP);
		for (int p = 0; p < numP; p++) {
			_robots[k].particles[r][p] = _robots[r];
		}
	}
}

void Robot::initMap(Robot& _robot, Problem& problem) {
	//MAP map;
	_robot.map.size = calculateMapSize(problem.tasks);
	_robot.map.gridPoints = generateMeshgrid(map.size.second, map.size.first);

	vector<vector<int>> xObs, yObs; // Example obstacle locations
	vector<vector<int>> bdObs = generateBoundaryObstacles(map.size.second, map.size.first);

	_robot.map.obs.insert(map.obs.end(), xObs.begin(), xObs.end());
	_robot.map.obs.insert(map.obs.end(), bdObs.begin(), bdObs.end());

	// Assuming M0 is a 2D grid of doubles initialized to 0.5, similar to MATLAB's matrix
	_robot.map.M0 = vector<vector<double>>(map.size.second, vector<double>(map.size.first, 0.5));
}

// Private
pair<int, int> Robot::calculateMapSize(const vector<Eigen::Vector2d>& taskLoc) {
	double maxX = 0.0, maxY = 0.0;
	for (const auto& loc : taskLoc) {
		maxX = round(max(maxX, loc[0]));
		maxY = round(max(maxY, loc[1]));
	}
	return { static_cast<int>(maxX + 5), static_cast<int>(maxY + 5) };
}

vector<vector<int>> Robot::generateMeshgrid(int rows, int cols) {
	vector<vector<int>> gridPoints;
	for (int x = 1; x <= cols; ++x) {
		for (int y = 1; y <= rows; ++y) {
			gridPoints.push_back({ x, y });
		}
	}
	return gridPoints;
}

vector<vector<int>> Robot::generateBoundaryObstacles(int rows, int cols) {
	vector<vector<int>> bdObs;
	for (int i = 1; i <= cols; ++i) {
		bdObs.push_back({ i, 1 });
		bdObs.push_back({ i, rows });
	}
	for (int i = 2; i < rows; ++i) { // Avoid duplicating corners
		bdObs.push_back({ 1, i });
		bdObs.push_back({ cols, i });
	}
	return bdObs;
}