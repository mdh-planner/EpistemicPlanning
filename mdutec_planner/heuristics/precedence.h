#pragma once
#include "../main/mdutecalgo.h"
#include "../solution_representations/TSolution.h"
#include "../misc/randnumgen.h"
#include "gene_manipulation.h"
#include <random>

bool precedenceCheck(SolutionStruct& inSolutionStruct,
	PrecalculatedDataStruct& inPrecalcDataStruct,
	bool inEarlyFail = false)
{		
	
	bool allChecksPassed = true;
	/*debug_print_solution_agents(inSolutionStruct.mSolution, inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
	debug_print_raw_solution(inSolutionStruct.mSolution, inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
	*/
	for (int i = 0; i < inSolutionStruct.mNumberOfCities; i++)
	{
		if (inPrecalcDataStruct.mPrecedenceMatrix[i] > 0) {

			int _cityIdToPrecedeOffset = inPrecalcDataStruct.mPrecedenceMatrix[i] - 1 - i;
			int _agentId = 1;
			for (int j = i; j < inSolutionStruct.mNumberOfCities * inSolutionStruct.mNumberOfAgents; j += inSolutionStruct.mNumberOfCities, _agentId++) {

				/*if (inSolutionStruct.mSolution[j] > 0
					&& inSolutionStruct.mSolution[_cityIdToPrecedeOffset + j] > 0
					&& inSolutionStruct.mSolution[_cityIdToPrecedeOffset + j] < inSolutionStruct.mSolution[j])
				{*/
				/*debug_print_solution_agents(inSolutionStruct.mSolution, inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
				debug_print_raw_solution(inSolutionStruct.mSolution, inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);

				std::cout << inSolutionStruct.mSolution[j] << "\t" << inSolutionStruct.mSolution[_cityIdToPrecedeOffset + j] << std::endl;
				*/
				if (inSolutionStruct.mSolution[j] > 0
					&& inSolutionStruct.mSolution[_cityIdToPrecedeOffset + j] > 0
					&& inSolutionStruct.mSolution[j] - inSolutionStruct.mSolution[_cityIdToPrecedeOffset + j] == 1 )
				{
					allChecksPassed = true;
					break;
				}
				else
				{
					allChecksPassed = false;

					/* NE ZNAM CEMU OVO SLUZI PA SAM ZAKOMENTARISAO :) */

					//if(inEarlyFail)
					//	return false;
				}
			}

			if (!allChecksPassed) {
				//debug_print_solution_agents(inSolutionStruct.mSolution, inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
				//std::cout << "Failed check with City ID " << i + 1 << " and preceding City ID " << inPrecalcDataStruct.mPrecedenceMatrix[i] << std::endl;
				return false;
			}
		}
	}

	// THIS IS NOT WORKING PROPERLY. IT CAN RETURN TRUE EVEN IF THERE ARE FAILS. 
	// fix it some times, it is not urgent
	// As no issues were found, return true. 
	return allChecksPassed;
}

void precedenceReparation(SolutionStruct& inSolutionStruct,
	PrecalculatedDataStruct& inPrecalcDataStruct,
	std::vector<int>& inTaskCountPerAgent) {

	for (int cityIndex = 0; cityIndex < inSolutionStruct.mNumberOfCities; cityIndex++) {
		//debug_print_solution_agents(inSolutionStruct.mSolution, 9, 1);
		if (inPrecalcDataStruct.mPrecedenceMatrix[cityIndex] > 0) {
			
			//std::cout << "City ID " << cityIndex+1 << " has preceding City ID " << inPrecalcDataStruct.mPrecedenceMatrix[cityIndex] << std::endl;
			/*debug_print_raw_solution(inSolutionStruct.mSolution,
				inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/
			
			int _cityIdToPrecede = inPrecalcDataStruct.mPrecedenceMatrix[cityIndex];
			int _cityIdToPrecedeOffset = (_cityIdToPrecede - 1) - cityIndex;

			//std::cout << "Preceding City ID offset is " << _cityIdToPrecedeOffset << std::endl;

			int _agentId = 1; // Done by Vuk, why 1?
			//int _agentId = 0; // Set to 0 by Branko.

			// Pairs <agentNumber, indexOfCityInSolution> of cities in inPrecalcDataStruct.mPrecedenceMatrix
			std::pair<int, int> _cityBefore;
			std::pair<int, int> _cityAfter;
			_cityAfter.first = -1;
			_cityBefore.first = -1;
			// Flag indicate double passage through cities
			bool _cityInPrecedenceIsTwiceInSolution = false;
			for (int j = cityIndex; j < inSolutionStruct.mNumberOfCities * inSolutionStruct.mNumberOfAgents; j += inSolutionStruct.mNumberOfCities, _agentId++)
			{
				// If agent contains correction to be made
				if (inSolutionStruct.mSolution[j] > 0)
				{
					if (_cityBefore.first == -1)
					{
						_cityBefore.first = _agentId;
						_cityBefore.second = j;
					}
					else
					{
						_cityInPrecedenceIsTwiceInSolution = true;
						std::cout << "ERROR: Found a city (" << cityIndex + 1 << ") twice in the solution." << std::endl;
					}
				}
				
				
				if (inSolutionStruct.mSolution[_cityIdToPrecedeOffset + j] > 0)
				{
					if (_cityAfter.first == -1)
					{
						_cityAfter.first = _agentId;
						_cityAfter.second = _cityIdToPrecedeOffset + j;
					}
					else {
						_cityInPrecedenceIsTwiceInSolution = true;
						std::cout << "ERROR: Found a city (" << inPrecalcDataStruct.mPrecedenceMatrix[cityIndex] << ") twice in the solution." << std::endl;
					}
				}
			}

			if (_cityInPrecedenceIsTwiceInSolution) {
				std::cout << "ERROR FOUND NO PRECENDENCE FIXES WILL BE APPLIED FOR CITY " << cityIndex + 1 << " NEEDS CITY " << inPrecalcDataStruct.mPrecedenceMatrix[cityIndex] << " TO BE PRIOR." << std::endl;
			}
			else {
				// If both parts of precedence condition were found
				if (_cityBefore.first > 0 && _cityAfter.first > 0)
				{
					// Case 1 (as in A Genetic Algorithm Approach to Multi-Agent Mission Planning Problems)
					// Both cities are in the same agent but potentially wrong order
					if (_cityBefore.first == _cityAfter.first)
					{
						// need to swap
						// NOTE: Can reduce with IF above to single IF
						/*if (inSolutionStruct.mSolution[_cityBefore.second] < inSolutionStruct.mSolution[_cityAfter.second])
						{*/
						if (inSolutionStruct.mSolution[_cityBefore.second] < inSolutionStruct.mSolution[_cityAfter.second] ||
							inSolutionStruct.mSolution[_cityBefore.second] - inSolutionStruct.mSolution[_cityAfter.second] != 1)
						{
							//int _pathIndOfCity = inSolutionStruct.mSolution[_cityBefore.second];


							//inSolutionStruct.mSolution[_cityBefore.second] = inSolutionStruct.mSolution[_cityAfter.second];
							///*debug_print_solution_agents(inSolutionStruct.mSolution,
							//	inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/
							//inSolutionStruct.mSolution[_cityAfter.second] = _pathIndOfCity;

							/*debug_print_solution_agents(inSolutionStruct.mSolution,
							inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);

							debug_print_raw_solution(inSolutionStruct.mSolution,
								inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/
							removeCityFromSolution(inSolutionStruct, _cityAfter.first, _cityIdToPrecede);
							inTaskCountPerAgent[_cityAfter.first - 1]--;

							/*debug_print_solution_agents(inSolutionStruct.mSolution,
								inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
							debug_print_raw_solution(inSolutionStruct.mSolution,
								inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/
							insertCityToSolutionBeforeCityId2(inSolutionStruct, _cityBefore.first, _cityIdToPrecede, cityIndex + 1,
								inTaskCountPerAgent);
							inTaskCountPerAgent[_cityBefore.first - 1]++; 

							/*debug_print_solution_agents(inSolutionStruct.mSolution,
								inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
							debug_print_raw_solution(inSolutionStruct.mSolution,
								inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/

						}

					}
					else
						// Case 2 -> Tasks are in different agents
					{
						auto _isSuitableForTask = [&](int inAgentId, int inCityIndInWholeSolution) {
							int cityId = inCityIndInWholeSolution % inSolutionStruct.mNumberOfCities;
							return inPrecalcDataStruct.mAgentCityMatrix[inAgentId - 1][cityId];
						};

						enum eSuitableAgent { eSA_AGENT1, eSA_AGENT2, eSA_MOVE1, eSA_MOVE_BOTH, eSA_NONE };

						bool _beforeSuitableForBoth =
							_isSuitableForTask(_cityBefore.first, _cityAfter.second);
						bool _afterSuitableForBoth =
							_isSuitableForTask(_cityAfter.first, _cityBefore.second);

						auto _suitableDecisioning = [_beforeSuitableForBoth, _afterSuitableForBoth]() {
							if (_beforeSuitableForBoth && _afterSuitableForBoth)
								return eSA_MOVE1;
							if (_beforeSuitableForBoth && !_afterSuitableForBoth)
								return eSA_AGENT1;
							if (!_beforeSuitableForBoth && _afterSuitableForBoth)
								return eSA_AGENT2;
							if (!_beforeSuitableForBoth && !_afterSuitableForBoth)
								return eSA_MOVE_BOTH;
							return eSA_NONE;
						};

						auto _moveBothToAgent1 = [&]() {
							/*debug_print_solution_agents(inSolutionStruct.mSolution,
								inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/

							removeCityFromSolution(inSolutionStruct, _cityAfter.first, _cityIdToPrecede);
							inTaskCountPerAgent[_cityAfter.first - 1]--;

							/*debug_print_solution_agents(inSolutionStruct.mSolution,
								inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/

							insertCityToSolutionBeforeCityId(inSolutionStruct, _cityBefore.first, _cityIdToPrecede, cityIndex + 1,
								inTaskCountPerAgent);
							inTaskCountPerAgent[_cityBefore.first - 1]++;

							/*debug_print_solution_agents(inSolutionStruct.mSolution,
								inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/

						};
						auto _moveBothToAgent2 = [&]() {

							/*debug_print_solution_agents(inSolutionStruct.mSolution,
								inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
							debug_print_raw_solution(inSolutionStruct.mSolution,
								inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/
							removeCityFromSolution(inSolutionStruct, _cityBefore.first, cityIndex + 1);
							inTaskCountPerAgent[_cityBefore.first - 1]--; // this is not working properly
							/*debug_print_solution_agents(inSolutionStruct.mSolution,
								inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/

							insertCityToSolutionAfterCityId(inSolutionStruct, _cityAfter.first, cityIndex + 1, _cityIdToPrecede,
								inTaskCountPerAgent);
							inTaskCountPerAgent[_cityBefore.first - 1]++; // this is not working properly
							
						/*	debug_print_solution_agents(inSolutionStruct.mSolution,
							inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
							debug_print_raw_solution(inSolutionStruct.mSolution,
								inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/

						};
						auto _moveBothToAgentN = [&](int N) {

							/*		debug_print_solution_agents(inSolutionStruct.mSolution,
										inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
									debug_print_raw_solution(inSolutionStruct.mSolution,
										inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/
							removeCityFromSolution(inSolutionStruct, _cityBefore.first, cityIndex + 1);

							inTaskCountPerAgent[_cityBefore.first - 1]--; // this is not working properly

							removeCityFromSolution(inSolutionStruct, _cityAfter.first, _cityIdToPrecede);

							inTaskCountPerAgent[_cityAfter.first - 1]--; // this is not working properly
					/*		debug_print_solution_agents(inSolutionStruct.mSolution,
								inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/

							insertCityToSolutionAfterCityId(inSolutionStruct, N, cityIndex + 1, _cityIdToPrecede, inTaskCountPerAgent);
							inTaskCountPerAgent[_cityBefore.first - 1]++; // this is not working properly
							insertCityToSolutionBeforeCityId(inSolutionStruct, N, _cityIdToPrecede, cityIndex + 1, inTaskCountPerAgent);
							inTaskCountPerAgent[_cityBefore.first - 1]++; // this is not working properly
							//debug_print_solution_agents(inSolutionStruct.mSolution,
							//	inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
							//debug_print_raw_solution(inSolutionStruct.mSolution,
							//	inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);

						};
						std::vector<int> agentList;
						auto _case = _suitableDecisioning();
						//std::cout << "Suitable case has been found." << std::endl;
						switch (_case)
						{
							// Subcase (2) -> agent before is the only one suitable
						case eSA_AGENT1:
							//	std::cout << "eSA_AGENT1 case in progress." << std::endl;
							_moveBothToAgent1();
							//	std::cout << "eSA_AGENT1 case done." << std::endl;
							break;
							// Subcase (2) -> agent after is the only one suitable
						case eSA_AGENT2:
							//std::cout << "eSA_AGENT2 case in progress." << std::endl;
							_moveBothToAgent2();
							//std::cout << "eSA_AGENT2 case done." << std::endl;
							break;
							// Subcase (1)
						case eSA_MOVE1:
							//std::cout << "eSA_BOTH case in progress." << std::endl;
							//if (dist(mt) >= 0.5)
							if (getRandomRealInRange(0.0, 1.0) >= 0.5)
							{
								// We use Agent 1 (i.e. _cityBefore.first)
								_moveBothToAgent1();
							}
							else
							{
								// We use Agent 2 (i.e. _cityAfter.first)
								_moveBothToAgent2();
							}
							break;
						case eSA_MOVE_BOTH:
							// NOT TESTED!
							agentList.clear();
							agentList.reserve(inSolutionStruct.mNumberOfAgents);
							for (int agId = 0; agId < inSolutionStruct.mNumberOfAgents; agId++) {

								if (_isSuitableForTask(agId, _cityAfter.second) && _isSuitableForTask(agId, _cityBefore.second)) {
									agentList.push_back(agId);
								}
							}
						
							_moveBothToAgentN(agentList[getRandomIntegerInRange(0, static_cast<int>(agentList.size()))]);

							
							//std::cout << "NEEDS IMPLEMENTATION\n\n" << std::endl;
							break;
						case eSA_NONE:
							std::cout << "eSA_NONE case found.\n\nNEEDS IMPLEMENTATION\n\n" << std::endl;
							break;
						default:
							break;
						}
					}
				}
			}
			/*debug_print_solution_agents(inSolutionStruct.mSolution,
				inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/
		}
	}
	/*debug_print_solution_agents(inSolutionStruct.mSolution,
		inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);*/
}
