#pragma once
#include <numbers>
#include "../eigen-master/Eigen/Dense"
#include "readProblem.h"
#include "Node_EP.h"
#include "structures.h"

class Robot {
public:
	std::vector<double> obsDist, angles; //   angles
	std::vector<std::pair<int,int>> taskReached;
	MAP map; // Initial belief state of map
	std::vector<int>  originalPlan, numParticles, taskQueue, particleGuess, particleLastUpdate, robotLastUpdate, commQueue, rzQueue, dfwm, tasks_done, deadBots, repFlag;
	taskDict tasksAssigned;
	std::vector<Plan> plan;
	std::vector<Eigen::Vector2d> globalTaskLocs, depot;
	Pose pose, pose_sim; //
	double maxRange, commRange, max_v, dt, commError, decay; // Default values
	int ID, particleFollow, connected_replan, replan, planID; // Default ID set to -1 indicating not set
	std::string state; // Assuming "Idle" is a valid default state
	bool reachedNewParticle, iamdifferent = false, at_final_pos = false;
	Point goalLoc; // Default locations
	std::vector<std::vector<Robot>> particles;
	double goalReached = 1.0; // Threshold for reaching the goal
	std::shared_ptr<Node_EP> root;

	// Default constructor
	Robot() {}

	void initRobots(int _id, Problem& problem);
	void initParticles(std::vector<Robot>& _robots, int k);
	void initMap(Robot& _robot, Problem& problem);
	
	int numBots, numP;

private:
	std::pair<int, int> calculateMapSize(const std::vector<Eigen::Vector2d>& taskLoc);

	std::vector<std::vector<int>> generateMeshgrid(int rows, int cols);

	std::vector<std::vector<int>> generateBoundaryObstacles(int rows, int cols);

};