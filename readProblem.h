#include "eigen-master/Eigen/dense"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

class Problem {
	
public:
	Problem() {}
	void readConfigurations(const std::string& initConditionsTxt, const std::string& allocationsTxt);
	std::vector<std::vector<int>> robotPlans;
	std::vector<Eigen::Vector2d> robotLocs, tasks, depots; // Positions of robots,tasks, and depots
private:
	
};