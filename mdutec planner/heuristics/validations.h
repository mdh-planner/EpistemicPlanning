#pragma once
#include "..\main\algorithm.h"
#include "..\solution_representations\TSolution.h"
#include "precedence.h"


/** Validates that agents of RawSolution have been
	visiting cities wrt to the precedence constraints
* /param inSolutionStruct.mSolution - RawSolution to be tested
* /param inSolutionStruct.mNumberOfCities - number of cities in problem
* /param inSolutionStruct.mNumberOfAgents - number of agents in solutions
* /param inPrecedenceConditions - City presendence conditions
* /return True if validation passed
*/
bool validateCityprecedences(SolutionStruct& inSolutionStruct,
	PrecalculatedDataStruct& inPrecalcDataStruct)
{
	//debug_print_solution_agents(inSolutionStruct.mSolution,
		//inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
	const bool early_fail = true;
	return precedenceCheck(inSolutionStruct,
		inPrecalcDataStruct,
		early_fail);
}

/** Validates that agents of RawSolution have been
	visiting cities they are allowed to
* /param inSolutionStruct.mSolution - RawSolution to be tested
* /param inSolutionStruct.mNumberOfCities - number of cities in problem
* /param inSolutionStruct.mNumberOfAgents - number of agents in solutions
* /param inAgentCityMatrix - AgentCityMat describing agents
							permissions to enter a city
* /return True if validation passed
*/
bool validateCityAgentColor(SolutionStruct& inSolutionStruct,
	PrecalculatedDataStruct& inPrecalcDataStruct)
{
	// for debugging/logging
	std::vector<std::vector<int>> _agentViolations;
	bool _failed = false;

	for (int _i = 0; _i < inSolutionStruct.mNumberOfAgents; _i++)
	{
		bool* agent_access_code = inPrecalcDataStruct.mAgentCityMatrix[_i];
		int j_offset = _i * inSolutionStruct.mNumberOfCities;
		// debug/logging subvector initialization
		_agentViolations.push_back({});

		for (int _j = 0; _j < inSolutionStruct.mNumberOfCities; _j++)
		{
			if (inSolutionStruct.mSolution[_j + j_offset] > 0 && !agent_access_code[_j])
			{
				_agentViolations[_i].push_back(_j + 1);
				_failed = true;
			}
		}
	}

	// Output issues:

	if (_failed) {

		debug_print_raw_solution(inSolutionStruct.mSolution,
			inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
		debug_print_solution_agents(inSolutionStruct.mSolution,
			inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);

		for (int i = 0; i < inSolutionStruct.mNumberOfAgents; i++)
		{
			std::cout << "Agent " << i + 1 << " present in following forbidden cities: ";
			for (int j = 0; j < _agentViolations[i].size(); j++)
			{
				std::cout << _agentViolations[i][j] << ", ";
			}
			std::cout << std::endl;
		}
	}

	return _failed;
}

/** Validates that RawSolution covered all cities in the problem
* /param inSolutionStruct.mSolution - RawSolution to be tested
* /param inSolutionStruct.mNumberOfCities - number of cities in problem
* /param inSolutionStruct.mNumberOfAgents - number of agents in solutions
* /return True if validation passed
*/
bool validateAllCitiesVisited(SolutionStruct& inSolutionStruct)
{
	//for (int _i = 0; _i < inSolutionStruct.mNumberOfCities; _i++)
	//{
	//	bool _cityVisited = false;
	//	for (int _j = 0; _j < inSolutionStruct.mNumberOfAgents; _j += inSolutionStruct.mNumberOfCities)
	//	{
	//		_cityVisited |= (inSolutionStruct.mSolution[_i + _j] > 0);
	//	}
	//	if (!_cityVisited)
	//	{
	//		std::cout << "Not all cities have been visited!!!" << endl;
	//		return false;
	//	}
	//}
	//return true;


	for (int _i = 0; _i < inSolutionStruct.mNumberOfCities; _i++)
	{
		int counter = 0;
		for (int _j = 0; _j < inSolutionStruct.mNumberOfAgents * inSolutionStruct.mNumberOfCities; _j += inSolutionStruct.mNumberOfCities)
		{
			if ((inSolutionStruct.mSolution[_i + _j] > 0)) {
				counter++;
			}

		}
		if (counter != 1)
		{
			std::cout << "Not all cities have been visited!!!" << endl;

			debug_print_raw_solution(inSolutionStruct.mSolution,
				inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);
			debug_print_solution_agents(inSolutionStruct.mSolution,
				inSolutionStruct.mNumberOfCities, inSolutionStruct.mNumberOfAgents);

			return false;
		}
	}
	return true;
}


/** Validates that agents of RawSolution have been
	visiting cities wrt to the precedence constraints
* /param inSolutionStruct.mSolution - RawSolution to be tested
* /param inSolutionStruct.mNumberOfCities - number of cities in problem
* /param inSolutionStruct.mNumberOfAgents - number of agents in solutions
* /param inPrecedenceConditions - City presendence conditions
* /param inAgentCityMatrix - AgentCityMat describing agents
							permissions to enter a city
* /return True if all validations pass
*/
bool validate(SolutionStruct& inSolutionStruct,
	PrecalculatedDataStruct& inPrecalcDataStruct)
{
	bool vPC = validateCityprecedences(inSolutionStruct, inPrecalcDataStruct);
	bool vColor = validateCityAgentColor(inSolutionStruct, inPrecalcDataStruct);
	bool vVisited =	validateAllCitiesVisited(inSolutionStruct);
	return (vPC && vColor && vVisited);
}
