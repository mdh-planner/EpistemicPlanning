#pragma once

#include <vector>
#include <algorithm>
#include <numeric>


/* Array of 1 x (n * m), for every agent there is an array of all cities.*/

typedef int* RawSolution;

/** Representation of ordered set of individual solutions for single agents
* This is the full Solution for the multi-agent version of the problem
*/
typedef RawSolution* RawSolutionPopulation;

/** Distance matrix between cities
* For [x,y] it returns distance between City with ID x and City with ID y
* NOTE: Potential for data saving due to either sparcity or doubled data
*/
typedef long** DistMat;

/** Array of precedences, index being the city ID and value being ID of the preceeding city
* NOTES:
*	1. No city will have ID 0, it will be used for no-precedence flag
*	2. City ID 1 will match to index 0 in the array
* Example: City 4 requires City 2 prior, City 6 requires City 3 prior
* Resulting array: 0 0 0 2 0 3 0
*/
typedef int* Precedences;

/** Matrix of Agent IDs (1st index) as row and City IDs as columns (2nd index)
* Example (3 agents, 6 cities):
*	Red color: City 2,4,6
*	Blue color: City 3,1,5
*	Agent 1: Red
*	Agent 2: Blue
*	Agent 3: Red Blue
* Matrix (3x6):
*	1  1* 0  1* 1  1*
*	1  0  0  0  1  0
*	1  1* 1* 1* 1  1*
* where 1/0 means that Agent at Row I can/can't access City at Column J
* and * is just for clarification that this comes from the color matching
*/
typedef bool** AgentCityMat;

/* Mapping of tasks to agents, i.e., for each agent we have a list of tasks that it is able to do; Rows -> agents; Columns - Tasks*/
typedef std::vector<int>* Tasks2Agents;

/* Mapping of agents to tasks, i.e., for each task we have a list of agents that are able to do that task; Rows -> Tasks; Columns - Agents */
typedef std::vector<int>* Agents2Tasks;

/* Agents Color Instersection*/
typedef std::vector<std::vector<std::vector<int>>> agentsColorIntersect;

/* Closest tasks to each task */
typedef std::vector<std::vector<std::vector<int>>> closestTask;

/* Closest depot to each task pair<distance to depot, id of the depot> */
typedef std::pair<long, int>* DepoSelection;

struct SolutionMetaData {
	int mSolutionId;
	double mCost;
	bool mValidSolution;
};


struct PrecalculatedDataStruct {
	DistMat mDistanceMatrix;
	DistMat mSourceDepotDistanceMatrix;
	DepoSelection mDestinationDepotDistanceMatrix;
	Precedences mPrecedenceMatrix;
	AgentCityMat mAgentCityMatrix;
	unsigned long mNumberOfCities;
	unsigned int mNumberOfColors;
	unsigned long mNumberOfAgents;
	std::vector<long> mCityDuration;
};

struct SolutionPopulationStruct {
	RawSolutionPopulation mPopulation;
	unsigned long mNumberOfCities;
	unsigned long mNumberOfAgents;
	unsigned long mNumberOfSolutionsInPopulation;

};

struct SolutionStruct {
	RawSolution mSolution;
	unsigned long mNumberOfCities;
	unsigned long mNumberOfAgents;
	unsigned long mGen;
	unsigned long mPop;
};

struct MissionData {
	std::vector<Salesperson> salesUnit;
	std::vector<City> cities;
	std::vector<Depot> depots;
};

long calc_solution_cost(SolutionStruct& inSolutionStruct,
	PrecalculatedDataStruct& inPrecalcDataStruct) {

	std::vector<long> _agentCosts(inSolutionStruct.mNumberOfAgents);

	for (int _j = 0; _j < inSolutionStruct.mNumberOfAgents; _j++) {

		// Extract path
		_agentCosts[_j] = 0;
		int _i_offset = _j * inSolutionStruct.mNumberOfCities;
		std::vector<int> _path(inSolutionStruct.mNumberOfCities, -1);
		int _pathLen = 0;

		for (int i = 0; i < inSolutionStruct.mNumberOfCities; i++) {
			if (inSolutionStruct.mSolution[i + _i_offset] > 0) {
				_path[inSolutionStruct.mSolution[i + _i_offset] - 1] = i;
				_pathLen++;
			}
		}

		//Add Edges to the overall cost
		if (_pathLen > 0) {

			_agentCosts[_j] += inPrecalcDataStruct.mSourceDepotDistanceMatrix[_j][_path[0]];

			for (int i = 0; i < _pathLen - 1; i++) {
				_agentCosts[_j] += inPrecalcDataStruct.mDistanceMatrix[_path[i]][_path[i + 1]];
			}

			_agentCosts[_j] += inPrecalcDataStruct.mDestinationDepotDistanceMatrix[_path[_pathLen - 1]].first;

			if (_agentCosts[_j] < 0) {
				std::cout << "wtf negative cost?" << std::endl;
			}

			//Add City Stay Duration to the overall cost
			for (int i = 0; i < _pathLen; i++) {
				_agentCosts[_j] += inPrecalcDataStruct.mCityDuration[_path[i]];
			}	
		}
		else {
			_agentCosts[_j] = 0; //should we add the cost of going from source to destination depot or not?
		}
	}

	// Here you can choose the obj function. Missing implementation;

	//return  std::accumulate(_agentCosts.begin(), _agentCosts.end(), 0);
	return  *max_element(_agentCosts.begin(), _agentCosts.end()) + 0.1 * std::accumulate(_agentCosts.begin(), _agentCosts.end(), 0);
}

class Solution
{
public:
	Solution(RawSolution& inSolutionPtr,
		int inNumberOfCities, int inNumberOfAgents,
		DistMat& inDistanceMatrixPtr)
	{
		mSolutionPtr = inSolutionPtr;
		mNumOfCities = inNumberOfCities;
		mNumOfAgents = inNumberOfAgents;
		mDistanceMatrixPtr = inDistanceMatrixPtr;
	}


	//not used
	long getSolutionCost()
	{
		long _cost = 0;
		for (int _j = 0; _j < mNumOfAgents; _j++)
		{
			// Offset to be used for accessing elements of mSolutionPtr
			int _i_offset = _j * mNumOfCities;
			for (int _i = 0; _i < mNumOfCities; _i++)
			{
				if (mSolutionPtr[_i + _i_offset] > 0)
					_cost += mDistanceMatrixPtr[_i][mSolutionPtr[_i + _i_offset] - 1];
			}
		}
		return _cost;
	}

private:
	// Member to hold ptr to data
	RawSolution mSolutionPtr;
	// Distance mat ptr
	DistMat mDistanceMatrixPtr;
	// Number of cities used in solution 
	int mNumOfCities;
	// Number of agents used in solution 
	int mNumOfAgents;
};