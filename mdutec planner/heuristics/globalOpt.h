#pragma once
#include "..\main\algorithm.h"
#include "gene_manipulation.h"
#include "..\heuristics\validations.h"


class GlobalOpt {
public:
	static void insertClosestXAgent(SolutionStruct& inSolutionStruct, PrecalculatedDataStruct& inPrecalcDataStruct, Agents2Tasks& inA2T, closestTask& cT) {

		int Agent, rndInsertTask, pcTask, direction = 0;
		std::vector<int> tmpPath;
		bool op2 = false;
		// Pick a random agent
		int rndA = getRandomIntegerInRange(0, static_cast<int>(inSolutionStruct.mNumberOfAgents - 1));

		// Find all tasks allocated to the picked agent
		LocalOpt::findAllocTasks(inSolutionStruct, tmpPath, rndA);

		// Pick a task to remove and randomly insert 
		if (!tmpPath.empty()) {
			int tmp1 = getRandomIntegerInRange(0, static_cast<int>(tmpPath.size()) - 1);
			rndInsertTask = tmpPath[tmp1];
		} else { return; }

		if (inPrecalcDataStruct.mPrecedenceMatrix[rndInsertTask] > 0) {
			pcTask = inPrecalcDataStruct.mPrecedenceMatrix[rndInsertTask] - 1;
			direction = 1; 
		}

		if (inPrecalcDataStruct.mPrecedenceMatrix[rndInsertTask + inSolutionStruct.mNumberOfCities] > 0) {	
			pcTask = inPrecalcDataStruct.mPrecedenceMatrix[rndInsertTask + inSolutionStruct.mNumberOfCities] - 1;
			direction = -1; 
		}

		auto dirPos = performOperation1(inSolutionStruct, cT, inPrecalcDataStruct, rndA, rndInsertTask, Agent);
		if (dirPos.first == -1) { return; }

		if (direction != 0) {
			performOperation2(inSolutionStruct, cT, inPrecalcDataStruct, Agent, pcTask, dirPos, rndA, direction);
		}
	}
private:

	static pair<int,int> performOperation1(SolutionStruct& inSolutionStruct, closestTask& cT, PrecalculatedDataStruct& inPrecalcDataStruct, int Agent, int Task, int &newAgent) {

		auto atPairV = findClosestNodeSearch(inSolutionStruct, cT, Agent, Task);
		//pair<int, int> atPair = atPairV[getRandomIntegerInRange(0, static_cast<int>(atPairV.size()) - 1)];
		int i = 0;
		
		for (; i < atPairV.size() - 1; i++) {
			auto rndf = getRandomRealInRange(0.0, 1.0);
			if (rndf > 0.5) {
				break;
			}
		}
		pair<int, int> atPair = atPairV[i];


		if (atPair.first == -1) { return make_pair(-1,-1); } // it means that the task cannot be moved anywhere else.

		int insertPos2 = findInsertionPos(inSolutionStruct, inPrecalcDataStruct, atPair);

		int insertPos = inSolutionStruct.mSolution[atPair.first * inSolutionStruct.mNumberOfCities + atPair.second] + insertPos2;
		removeCityFromSolution(inSolutionStruct, Agent + 1, Task + 1);
		insertCityToSolution(inSolutionStruct, atPair.first + 1, Task + 1, inSolutionStruct.mSolution[atPair.first * inSolutionStruct.mNumberOfCities + atPair.second] + insertPos2);
		newAgent = atPair.first;

		return make_pair(insertPos2,insertPos);
	}

	static bool performOperation2(SolutionStruct& inSolutionStruct, closestTask& cT, PrecalculatedDataStruct& inPrecalcDataStruct, int Agent, int Task, pair<int,int> dirPos, int toRemoveAgent, int direction) {

		pair<int, int> atPair(Agent, -1);
		int _offset = Agent * inSolutionStruct.mNumberOfCities;
		int insertTaskAtPath;

		//if direction 1 we need to insert the city after insertPos; if direction 0 we need to insert the city before the insertPos;
		if (dirPos.second == 1 && direction == 1) { // inserts task at the beginning of the plan
			insertTaskAtPath = 1;
		} else {
			for (int i = 0; i < cT[Task][Agent].size(); i++) {
				int id = inSolutionStruct.mSolution[cT[Task][Agent][i] + _offset];
				if (direction == 1) {
					if (id > 0 && id < dirPos.second) {
						atPair.second = cT[Task][Agent][i];
						break;
					}
				}
				else {
					if (id > 0 && id > dirPos.second) {
						atPair.second = cT[Task][Agent][i];
						break;
					}
				}
			}

			if (atPair.second == -1) { // inserts task at the end of the plan
				auto idm1 = find(inSolutionStruct.mSolution + _offset, inSolutionStruct.mSolution + _offset + inSolutionStruct.mNumberOfCities, dirPos.second + 1);
				if (idm1 == inSolutionStruct.mSolution + _offset + inSolutionStruct.mNumberOfCities) {
					insertTaskAtPath = dirPos.second + 1;
				}
			}// THERE MIGHT BE PROBLEMS IF WE TRY TO INSERT INTO AGENTS WITH NO TASKS OR 1 TASK IN THE PLAN
			else {
				int insertPos2 = findInsertionPos(inSolutionStruct, inPrecalcDataStruct, atPair);
				insertTaskAtPath = inSolutionStruct.mSolution[atPair.first * inSolutionStruct.mNumberOfCities + atPair.second] + insertPos2;
			}
			
		}

		removeCityFromSolution(inSolutionStruct, toRemoveAgent + 1, Task + 1);
		insertCityToSolution(inSolutionStruct, atPair.first + 1, Task + 1, insertTaskAtPath);

		return true;
	}

	static vector<pair<int, int>> findClosestNodeSearch(SolutionStruct& inSolutionStruct, closestTask& cT, int Agent, int Task) {

		vector<pair<int, int>> _result;
		int N = 5; // arbitrary number;

		for (int i = 0; i < cT[Task][Agent].size(); i++) {
			for (int j = 0; j < inSolutionStruct.mNumberOfAgents; j++) {
				//cout << "i " << i << " j " << j << endl;
				if (j != Agent && !cT[Task][j].empty()) {
					int id = inSolutionStruct.mSolution[cT[Task][Agent][i] + inSolutionStruct.mNumberOfCities * j];
					if (id > 0 && _result.size() < N) {
						_result.push_back(make_pair(j, cT[Task][Agent][i]));
					}
					else if (_result.size() == N) { return _result; }
				}
			}
		}
		
		if (_result.empty()) {
			_result.push_back(make_pair(-1, -1));
		}

		return _result;
	}


	static int findInsertionPos(SolutionStruct& inSolutionStruct, PrecalculatedDataStruct& inPrecalcDataStruct, pair<int, int>& atPair) {

		int _offset = inSolutionStruct.mNumberOfCities * atPair.first;
		pair<double, double> dist;
		int idxp1 = -1, idxm1 = -1;
		int p1 = inSolutionStruct.mSolution[_offset + atPair.second] + 1;
		int m1 = inSolutionStruct.mSolution[_offset + atPair.second] - 1;

		if (inSolutionStruct.mSolution[_offset + atPair.second] == 1) {
			dist.second = inPrecalcDataStruct.mSourceDepotDistanceMatrix[atPair.first][atPair.second];
		}
		else {
			auto idm1 = find(inSolutionStruct.mSolution + _offset, inSolutionStruct.mSolution + _offset + inSolutionStruct.mNumberOfCities, m1);
			idxm1 = distance(inSolutionStruct.mSolution + _offset, idm1);
			dist.second = inPrecalcDataStruct.mDistanceMatrix[atPair.second][idxm1];
		}

		auto idp1 = find(inSolutionStruct.mSolution + _offset, inSolutionStruct.mSolution + _offset + inSolutionStruct.mNumberOfCities, p1);

		if (idp1 == inSolutionStruct.mSolution + _offset + inSolutionStruct.mNumberOfCities + 1) {
			dist.first = inPrecalcDataStruct.mDestinationDepotDistanceMatrix[atPair.second].first;
		}
		else {
			idxp1 = distance(inSolutionStruct.mSolution + _offset, idp1);
			dist.first = inPrecalcDataStruct.mDistanceMatrix[atPair.second][idxp1];
		}

		return (dist.first < dist.second) ? 1 : 0;
	}
};