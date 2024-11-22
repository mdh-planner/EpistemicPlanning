#pragma once
#include "..\solution_representations\TSolution.h"

#define MIN(x,y) ((x) < (y) ? (x) : (y)) //calculate minimum between two values
/**
* Class implementing different distance measure
**/
class DistanceMetric {

public:
	/** Humming distance between two strings
	* /param inSolutionOne - RawSolution (int* representing the whole solution)
	* /param inSolutionTwo - RawSolution (int* representing the whole solution)
	* /param inNumberOfCities - number of cities in problem
	* /param inNumberOfAgents - number of agents in solutions
	* /return Integer representing Humming distance
	*/
	static int hummingDistance(SolutionStruct &inSolutionOne,
		SolutionStruct& inSolutionTwo) {

		 //Calculate distances between individual agents of solution
		int _result = 0;
		for (size_t i = 0; i < inSolutionOne.mNumberOfAgents; i++)
		{
			int _interAgentDist = DistanceMetric::hummingDistanceImpl(
				(inSolutionOne.mSolution + i * inSolutionOne.mNumberOfCities),
				(inSolutionTwo.mSolution + i * inSolutionOne.mNumberOfCities),
				inSolutionOne.mNumberOfCities);
			// TODO - Solve issue with agent having 0 path!
			_result += _interAgentDist;
		}

		return _result;
	};

	static int levinsteinDistance(SolutionStruct& inSolutionOne,
		SolutionStruct& inSolutionTwo) {

		//Calculate distances between individual agents of solution
		int _result = 0;

		for (size_t i = 0; i < inSolutionOne.mNumberOfAgents; i++)
		{
			std::vector<int> a1, a2;
			findAllocTasks(inSolutionOne, a1, i);
			findAllocTasks(inSolutionTwo, a2, i);

			int _interAgentDist = DistanceMetric::levinsteinDistanceImpl(a1, a2);
			_result += _interAgentDist;
		}

		return _result;
	}

private:
	/** Humming distance between two strings
	* /param inAgentOne - RawSolution (int* representing the path of first agent)
	* /param inAgentTwo - RawSolution (int* representing the path of second agent)
	* /param inNumberOfCities - number of cities in problem for traversing the arrays
	* /return Integer representing Humming distance
	*/
	static int hummingDistanceImpl(RawSolution inAgentOne,
		RawSolution inAgentTwo,
		int inNumberOfCities) {

		long _result = 0;
		for (int i = 0; i < inNumberOfCities; i++) {
			if (inAgentOne[i] > 0 || inAgentTwo[i] > 0) { // removes needless comparison of unvisited cities
				_result += (inAgentOne[i] == inAgentTwo[i]) ? 0 : 1;
			}
		}

		return _result;
	}

	static int levinsteinDistanceImpl(std::vector<int> & a1, std::vector<int> & a2) {

		// The implementation is done by the Javier's suggestion
		// https://www.tutorialspoint.com/cplusplus-program-to-implement-levenshtein-distance-computing-algorithm
		// and adapted to our problem.
		int l1 = a1.size();
		int l2 = a2.size();

		std::vector<std::vector<int> > dist(l1, std::vector<int>(l2)); 
		
		if (l1 == 0) {return l2;}
		else if (l2 == 0) {return l1;}

		for (int i = 0; i < l2; i++) {
			dist[0][i] = i;
		}
		for (int j = 0; j < l1; j++) {
			dist[j][0] = j;
		}

		for (int i = 1; i < l1; i++) {
			for (int j = 1; j < l2; j++) {

				int t = MIN((dist[i - 1][j] + 1), (dist[i][j - 1] + 1));

				if (a1[i - 1] == a2[j - 1]) {
					dist[i][j] = MIN(t, (dist[i - 1][j - 1]));
				}
				else {
					dist[i][j] = MIN(t, (dist[i - 1][j - 1] + 1));
				}
			}
		}

		// Plot resulting Matrix
		
		//for (auto a : dist) {
		//	for (auto b : a) {
		//		std::cout << std::setw(3) << b << std::flush;
		//	}
		//	std::cout << std::endl;
		//}

		//std::cout << "The Levinstein distance is:" << dist[l1-1][l2-1];

		return dist[l1-1][l2-1];
	}

	static void findAllocTasks(SolutionStruct& inSolutionStruct,
		std::vector<int>& tmpPath, int rndA) {

		size_t _offset = rndA * inSolutionStruct.mNumberOfCities;

		for (int i = _offset; i < _offset + inSolutionStruct.mNumberOfCities; i++) {
			if (inSolutionStruct.mSolution[i] > 0) {
				tmpPath.push_back(i - _offset);
			}
		}
	}

};

