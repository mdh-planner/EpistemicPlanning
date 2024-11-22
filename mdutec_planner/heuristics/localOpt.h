#pragma once
#include "../main/mdutecalgo.h"
#include "gene_manipulation.h"
#include "../heuristics/validations.h"

class LocalOpt {

public:

	static void insertTask(SolutionStruct& inSolutionStruct, PrecalculatedDataStruct& inPrecalcDataStruct, Agents2Tasks& inA2T) {

		// Pick a random agent
		int rndA = getRandomIntegerInRange(0, static_cast<int>(inSolutionStruct.mNumberOfAgents - 1));

		// Find all tasks allocated to the picked agent
		std::vector<int> tmpPath, dummy;
		findAllocTasks(inSolutionStruct, tmpPath, rndA);

		// Pick a task to remove and randomly insert 
		int rndInsertTask;
		if (!tmpPath.empty()) {
			int tmp1 = getRandomIntegerInRange(0, static_cast<int>(tmpPath.size()) - 1);
			rndInsertTask = tmpPath[tmp1];
		} else {return;}

		removeCityFromSolution(inSolutionStruct, rndA + 1, rndInsertTask + 1);

		// Local Insertion if there are precedence constraints
		if (inPrecalcDataStruct.mPrecedenceMatrix[rndInsertTask] > 0) {

				// inserts task randomly to the same agent
			insertCityToSolutionAfterCityId(inSolutionStruct,
				rndA + 1, rndInsertTask + 1, inPrecalcDataStruct.mPrecedenceMatrix[rndInsertTask], dummy); //fix this dummy thing, it is unnacessary.

			return;
		}

		if (inPrecalcDataStruct.mPrecedenceMatrix[rndInsertTask + inSolutionStruct.mNumberOfCities] > 0) {
			
			// inserts task randomly to the same agent
			insertCityToSolutionBeforeCityId(inSolutionStruct,
				rndA + 1, rndInsertTask + 1, inPrecalcDataStruct.mPrecedenceMatrix[rndInsertTask + inSolutionStruct.mNumberOfCities], dummy); //fix this dummy thing, it is unnacessary.
	
			return;
		}

		int range = inA2T[rndInsertTask].size();
		int rndAgentInsertTask = getRandomIntegerInRange(0, range - 1);
		int rndAgent = inA2T[rndInsertTask][rndAgentInsertTask];

		insertCityToSolutionRandomly(inSolutionStruct, rndAgent, rndInsertTask + 1);
	}


	static void swapTasks(SolutionStruct& inSolutionStruct, PrecalculatedDataStruct& inPrecalcDataStruct, agentsColorIntersect& inACI, std::vector<City>& inCityData) {

		std::vector<int> tmpPath1, tmpPath2;
		std::pair<int, int> inCitiesToSwap, inAgents;

		// Pick the first random agent
		inAgents.first = getRandomIntegerInRange(0, static_cast<int>(inSolutionStruct.mNumberOfAgents - 1));

		// Pick the second random agent
		inAgents.second = getRandomIntegerInRange(0, static_cast<int>(inSolutionStruct.mNumberOfAgents - 1));

		if (inACI[inAgents.first][inAgents.second].empty()) { return; }

		// Find all tasks allocated to the picked agents that follow the color intersection scheme	
		findAllocatedColormapTasks(inSolutionStruct, tmpPath1, inAgents.first, inACI[inAgents.first][inAgents.second], inCityData);
		findAllocatedColormapTasks(inSolutionStruct, tmpPath2, inAgents.second, inACI[inAgents.first][inAgents.second], inCityData);

		// Pick a task to swap
		if (!tmpPath1.empty() && !tmpPath2.empty()) {
			inCitiesToSwap.first = tmpPath1[getRandomIntegerInRange(0, static_cast<int>(tmpPath1.size()) - 1)];
			inCitiesToSwap.second = tmpPath2[getRandomIntegerInRange(0, static_cast<int>(tmpPath2.size()) - 1)];
		} else {return;}

		taskSwappingInSolution(inSolutionStruct, inCitiesToSwap, inAgents);
		findAllTasks2Swap(inSolutionStruct, inPrecalcDataStruct, inCitiesToSwap, inAgents);

		std::swap(inCitiesToSwap.first, inCitiesToSwap.second);
		std::swap(inAgents.first, inAgents.second);
		findAllTasks2Swap(inSolutionStruct, inPrecalcDataStruct, inCitiesToSwap, inAgents);
	}

private:

	static void findAllocatedColormapTasks(SolutionStruct& inSolutionStruct,
		std::vector<int>& tmpPath, int rndA, std::vector<int>& intersect, std::vector<City>& inCityData) {

		size_t _offset = rndA * inSolutionStruct.mNumberOfCities;

		for (int i = _offset; i < _offset + inSolutionStruct.mNumberOfCities; i++) {
			if (inSolutionStruct.mSolution[i] > 0) {
				for (int j = 0; j < intersect.size(); j++) {
					if (inCityData[i - _offset].getColor() == intersect[j] + 1) {
						tmpPath.push_back(i - _offset);
					}
				}
			}
		}
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

	static void findAllTasks2Swap(SolutionStruct& inSolutionStruct,
		PrecalculatedDataStruct& inPrecalcDataStruct,
		std::pair<int, int>& inCitiesToSwap,
		std::pair<int, int>& inAgents) {

		std::vector<int> dummy;

		//if there is a city that needs to precede city "inCitiesToSwap.first"
		if (inPrecalcDataStruct.mPrecedenceMatrix[inCitiesToSwap.first] > 0) {

			if (inSolutionStruct.mSolution[inAgents.first * inSolutionStruct.mNumberOfCities + inPrecalcDataStruct.mPrecedenceMatrix[inCitiesToSwap.first] - 1] >
				inSolutionStruct.mSolution[inAgents.first * inSolutionStruct.mNumberOfCities + inCitiesToSwap.first]) {
				/*	debug_print_raw_solution(inSolutionStruct.mSolution,
						inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
					debug_print_solution_agents(inSolutionStruct.mSolution,
						inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/

						// removes the task
				removeCityFromSolution(inSolutionStruct, inAgents.first + 1, inPrecalcDataStruct.mPrecedenceMatrix[inCitiesToSwap.first]);
				/*		debug_print_raw_solution(inSolutionStruct.mSolution,
							inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
						debug_print_solution_agents(inSolutionStruct.mSolution,
							inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/

							// inserts task randomly to the same agent as the city it precedes
				insertCityToSolutionBeforeCityId(inSolutionStruct, inAgents.second + 1, inPrecalcDataStruct.mPrecedenceMatrix[inCitiesToSwap.first],
					inCitiesToSwap.first + 1, dummy); //fix this dummy thing, it is unnecessary.
				//debug_print_solution_agents(inSolutionStruct.mSolution,
				//	inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
				//validateCityAgentColor(inSolutionStruct, inPrecalcDataStruct);
			}
		}
		//if there is a city that needs to succeed city "inCitiesToSwap.first"
		if (inPrecalcDataStruct.mPrecedenceMatrix[inCitiesToSwap.first + inSolutionStruct.mNumberOfCities] > 0) {

			if (inSolutionStruct.mSolution[inAgents.second * inSolutionStruct.mNumberOfCities + inPrecalcDataStruct.mPrecedenceMatrix[inCitiesToSwap.first + inSolutionStruct.mNumberOfCities] - 1] <
				inSolutionStruct.mSolution[inAgents.second * inSolutionStruct.mNumberOfCities + inCitiesToSwap.first]) {

				/*debug_print_solution_agents(inSolutionStruct.mSolution,
					inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/
					// removes the task
				removeCityFromSolution(inSolutionStruct, inAgents.first + 1, inPrecalcDataStruct.mPrecedenceMatrix[inCitiesToSwap.first + inSolutionStruct.mNumberOfCities]);
				/*debug_print_solution_agents(inSolutionStruct.mSolution,
					inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/
					// inserts task randomly to the same agent as the city it succeeds
				insertCityToSolutionAfterCityId(inSolutionStruct,
					inAgents.second + 1, inPrecalcDataStruct.mPrecedenceMatrix[inCitiesToSwap.first + inSolutionStruct.mNumberOfCities], inCitiesToSwap.first + 1, dummy); //fix this dummy thing, it is unnecessary.
			/*debug_print_solution_agents(inSolutionStruct.mSolution,
					inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/
					//validateCityAgentColor(inSolutionStruct, inPrecalcDataStruct);
			}
		}
	}

	friend class GlobalOpt;
};