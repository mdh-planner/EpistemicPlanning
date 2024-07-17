#include "readProblem.h"

using namespace std;

void Problem::readConfigurations(const string& initConditionsTxt, const string& allocationsTxt) {
	ifstream initFile(initConditionsTxt);
	string line;


	int flag = 0;
	while (getline(initFile, line)) {
		istringstream iss(line);
		double x, y;
		iss >> x >> y;

		if (x == -1 && y == -1) {
			flag++;
			continue;
		}

		if (flag == 0) {
			robotLocs.emplace_back(x, y);
		}

		if (flag == 1) {
			tasks.emplace_back(x, y);
		}

		if (flag == 2) {
			tasks.emplace_back(x, y);
			depots.emplace_back(x, y);
		}
	}


	// Parse the allocations file
	ifstream allocFile(allocationsTxt);

	size_t robotId = 0;

	while (getline(allocFile, line)) {
		istringstream iss(line);
		vector<int> plan((istream_iterator<int>(iss)), istream_iterator<int>());

		// Sort the tasks based on their order, keeping track of the original task numbers
		vector<pair<int, int>> sortedPlan; // First: original task number (0-indexed), Second: order
		for (int i = 0; i < plan.size(); ++i) {
			if (plan[i] > 0) { // task is part of the plan
				sortedPlan.emplace_back(i, plan[i]); // Adjust for 0-indexing
			}
		}

		// Sort by the order, which is the second element of the pair
		sort(sortedPlan.begin(), sortedPlan.end(), [](const pair<int, int>& a, const pair<int, int>& b) {
			return a.second < b.second;
		});
		robotPlans.resize(robotLocs.size());
		// Extract the sorted task numbers back into robotPlans
		robotPlans[robotId].clear();
		for (const auto& p : sortedPlan) {
			robotPlans[robotId].push_back(p.first); // Keep task IDs 0-indexed
		}

		robotId++;
	}
}

