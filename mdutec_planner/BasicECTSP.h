// BasicECTSP.h (add this declaration)
#pragma once


// BasicECTSP.cpp (add this definition)
// #include "BasicECTSP.h"
#include "base/Salesman.h"
#include "base/City.h"
#include "base/Depot.h"
#include "misc/CsvReader.h"
#include "heuristics/precedence.h"
#include "heuristics/gene_manipulation.h"
#include "main/DiscretePopulationBasedAlgoritm.h"
#include "misc/importData.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <iterator>
#include <emmintrin.h>
#include <immintrin.h>
#include <stdlib.h>


#ifndef POSE_H
#define POSE_H

typedef struct Pose {
	double x, y, theta;
};
#endif

// std::vector<int> runAlgorithm(int instance, std::vector<int>& tasks, std::vector<std::pair<double, double>>& taskLoc,
// 	std::vector<Pose>& startLoc, std::vector<Pose>& depots, std::vector<std::vector<int>>& prec);


std::vector<int> runAlgorithm(int instance, std::vector<int>& tasks, std::vector<std::pair<double, double>>& taskLoc,
	std::vector<Pose>& startLoc, std::vector<Pose>& depots, std::vector<std::vector<int>>& prec) {
	/*Update counter file*/
	int count = 0;
	std::ifstream infile("counter.txt"); infile >> count;
	std::ofstream count_file; count_file.open("counter.txt"); count_file << count + 1; count_file.close();

	/*----- Parse data for the planner -------------*/
	MissionData MD;
	// create auxiliary 

	for (long i = 0; i < startLoc.size(); i++) {
		float _speed = 5;
		vector<bool> _colors = { true };
		Salesperson _m(i, startLoc[i].x, startLoc[i].y, _speed, _colors);
		MD.salesUnit.push_back(_m);
	}

	//for (int ii = 0; ii < tasks.size(); ii++) {



	int N = tasks.size();

	std::vector<int> tempTaskIDs(N);
	std::iota(tempTaskIDs.begin(), tempTaskIDs.end(), 0);

	for (long i = 0; i < tempTaskIDs.size(); i++) {

		float _x = taskLoc[i].first;
		float _y = taskLoc[i].second;



		float _duration = 1;
		int _color = 1;
		int _precedenceCity = -1;
		for (int ii = 0; ii < prec.size(); ii++) {

			if (!prec[ii].empty()) {
				auto flag = std::find(prec[ii].begin(), prec[ii].end()-1, tasks[i]) != prec[ii].end()-1;

				if (flag) {
					_precedenceCity = i + 1;
				}
			}
		}
		City _city(tasks[i], _x, _y, _color, _duration, _precedenceCity);
		MD.cities.push_back(_city);
		//std::cout << _city.getPrintName() << "\t" << _x << "\t" << _y << "\t" << std::endl;
	}
	//}
	for (long i = 0; i < depots.size(); i++) {
		Depot _depot(i, depots[i].x, depots[i].y);
		MD.depots.push_back(_depot);
	}

	//ImportData::printLoadedData(MD);
	int maxRun = 1;

	/* We need to define Algorithm Parameters somewhere, for now they are all over the place */
	unsigned long populationSize = 201;
	std::vector<int> result, _result;

	std::cout << std::endl;
	std::cout << "=========================" << std::endl;
	std::cout << "=======Replanning!=======" << std::endl;
	std::cout << "=========================" << std::endl;
	std::cout << std::endl;

	for (int run = 0; run < maxRun; run++) {
		clock_t tStart2 = clock();

		mdutec::mdutecAlgo* algo = new DiscretePopulationBasedAlgoritm(populationSize, MD);

		result = algo->run(instance, run, MD.cities.size(), MD.salesUnit.size());

		printf("Run time: %.2fs\n", (double)(clock() - tStart2) / CLOCKS_PER_SEC);

		delete algo;  // Don't forget to delete the allocated memory
	}

	// prepare result
	_result.resize(result.size());
	vector<int> tasks2;
	for (int i = 0; i < tasks.size(); i++) {
		tasks2.insert(tasks2.end(), tasks.begin(), tasks.end());
	}
	for (int i = 0; i < result.size(); i++) {

		_result[i] = tasks2[result[i] - 1];
		std::cout << _result[i] << "," << std::flush;
	}
	std::cout << std::endl;
	return _result;

}