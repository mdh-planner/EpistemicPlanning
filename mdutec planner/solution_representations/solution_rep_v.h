#pragma once
#include "base/City.h"
#include <vector>
#include <math.h>

class solution_rep_v {

public:
	solution_rep_v(std::vector<City>& inCities) {
		mCities = inCities;
		mAllDistances = malloc()
		generateDistances();
	}

private:
	void generateDistances() {
		int len = mCities.size();
		for (int _i = 0; _i < len; _i++) {
			std::pair<float, float> _city1 = mCities[_i].getLocation();
			for (int _j = 0; _j < len; _j++) {
				std::pair<float, float> _city2 = mCities(_j).getLocation();
				double _dist = calcDistance(_city1, _city2);
				mAllDistances[_i][_j] = _dist;
			}
		}
	}

	double calcDistance(
		std::pair<float, float> inLocation1, 
		std::pair<float, float> inLocation2) {

		return sqrt((inLocation1.first - inLocation2.first) ^ 2
			+ (inLocation1.second - inLocation2.second) ^ 2);
	}

private:
	char** mAllAgents;

	double** mAllDistances;

	std::vector<City> mCities;
};