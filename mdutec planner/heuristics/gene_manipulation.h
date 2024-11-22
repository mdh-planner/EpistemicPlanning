#pragma once
#include "..\solution_representations\TSolution.h"
#include "precedence.h"
#include "..\misc\debugging.h"
#include <vector>


bool insertCityToSolution(SolutionStruct& inSolutionStruct,
	int inAgentId,
	int inCityIdToInsert, int inPositionOnPathToInsertAt)
{
	int _indexOfAgent = inAgentId - 1;
	if (_indexOfAgent < 0 || _indexOfAgent > inSolutionStruct.mNumberOfAgents)
	{
		// Failure due to invalid agent id
		return false;
	}

	// Index is 0-based (City ID 1 => index 0)
	int _indexOfCityToInsert = inCityIdToInsert - 1;
	//if (inSolutionPtr[_indexOfAgent * inNumberOfCities + _indexOfCityToInsert] > 0)
	//{
	//	std::cout << "A city exists at the place of planned insertion!" << std::endl;
	//	// City has already been added to the path -> NO PLANNED HANDLING
	//	return false;
	//}

	// Used to know if the city is added at the end
	int _agentPathLength = 0;
	// Iterate over cities in agent to discover path
	for (int j = 0; j < inSolutionStruct.mNumberOfCities; j++)
	{
		int globalIndex = j + _indexOfAgent * inSolutionStruct.mNumberOfCities;
		if (inSolutionStruct.mSolution[globalIndex] >= inPositionOnPathToInsertAt)
		{
			inSolutionStruct.mSolution[globalIndex]++;
		}
		// Update agent path length as max value found
		_agentPathLength = inSolutionStruct.mSolution[globalIndex] > _agentPathLength ?
			inSolutionStruct.mSolution[globalIndex] : _agentPathLength;
	}

	if (inPositionOnPathToInsertAt == 0) {

		std::cout << "CRAAP" << std::endl;
	}

	// If path reached the position of the trip length -> add
	if (_agentPathLength >= inPositionOnPathToInsertAt - 1)
	{
		inSolutionStruct.mSolution[_indexOfAgent * inSolutionStruct.mNumberOfCities + _indexOfCityToInsert] = inPositionOnPathToInsertAt;
	}
	else
	{
		// Path of the agent never reached the required length for insertion
		return false;
	}
	//debug_print_raw_solution(inSolutionPtr,
	//	inNumberOfCities, inNumberOfAgents);
	//debug_print_solution_agents(inSolutionPtr,
	//	inNumberOfCities, inNumberOfAgents);
	// No-error exit
	return true;
}

bool insertCityToSolutionBeforeCityId(SolutionStruct& inSolutionStruct,
	int inAgentId,
	int inCityIdToInsert, int inCityIdToInsertBefore,
	std::vector<int>& inTaskCountPerAgent)
{
	//debug_print_raw_solution(inSolutionPtr, inNumberOfCities, inNumberOfAgents);
	int _indexOfAgent = inAgentId - 1;
	int _i_offset = _indexOfAgent * inSolutionStruct.mNumberOfCities;
	if (_indexOfAgent < 0 || _indexOfAgent > inSolutionStruct.mNumberOfAgents)
	{
		// Failure due to invalid agent id
		return false;
	}

	int _indexOfCityToInsertBefore = inCityIdToInsertBefore - 1;
	// Index is 0-based (City ID 1 => index 0)
	int _indexOfCityToInsert = inCityIdToInsert - 1;
	if (inSolutionStruct.mSolution[_indexOfAgent * inSolutionStruct.mNumberOfCities + _indexOfCityToInsert] > 0
		|| inSolutionStruct.mSolution[_indexOfAgent * inSolutionStruct.mNumberOfCities + _indexOfCityToInsertBefore] == 0)
	{
		// City has already been added to the path -> NO PLANNED HANDLING
		// OR City before is not on the path at all
		return false;
	}


	// Randomly select the position in the path to insert the task/city
	int _indexInPathToInsertCity = getRandomIntegerInRange(1, inSolutionStruct.mSolution[_i_offset + inCityIdToInsertBefore - 1]);

	return insertCityToSolution(inSolutionStruct, inAgentId, inCityIdToInsert, _indexInPathToInsertCity);
}

bool insertCityToSolutionBeforeCityId2(SolutionStruct& inSolutionStruct,
	int inAgentId,
	int inCityIdToInsert, int inCityIdToInsertBefore,
	std::vector<int>& inTaskCountPerAgent) // IT inserts city immediately before the city, no randomness
{
	//debug_print_raw_solution(inSolutionPtr, inNumberOfCities, inNumberOfAgents);
	int _indexOfAgent = inAgentId - 1;
	int _i_offset = _indexOfAgent * inSolutionStruct.mNumberOfCities;
	if (_indexOfAgent < 0 || _indexOfAgent > inSolutionStruct.mNumberOfAgents)
	{
		// Failure due to invalid agent id
		return false;
	}

	int _indexOfCityToInsertBefore = inCityIdToInsertBefore - 1;
	// Index is 0-based (City ID 1 => index 0)
	int _indexOfCityToInsert = inCityIdToInsert - 1;
	if (inSolutionStruct.mSolution[_indexOfAgent * inSolutionStruct.mNumberOfCities + _indexOfCityToInsert] > 0
		|| inSolutionStruct.mSolution[_indexOfAgent * inSolutionStruct.mNumberOfCities + _indexOfCityToInsertBefore] == 0)
	{
		// City has already been added to the path -> NO PLANNED HANDLING
		// OR City before is not on the path at all
		return false;
	}


	// Randomly select the position in the path to insert the task/city
	int _indexInPathToInsertCity = inSolutionStruct.mSolution[_i_offset + inCityIdToInsertBefore - 1];

	return insertCityToSolution(inSolutionStruct, inAgentId, inCityIdToInsert, _indexInPathToInsertCity);
}


bool insertCityToSolutionAfterCityId(SolutionStruct& inSolutionStruct,
	int inAgentId,
	int inCityIdToInsert, int inCityIdToInsertAfter,
	std::vector<int>& inTaskCountPerAgent)
{
	//debug_print_raw_solution(inSolutionPtr, inNumberOfCities, inNumberOfAgents);
	int _indexOfAgent = inAgentId - 1;
	if (_indexOfAgent < 0 || _indexOfAgent > inSolutionStruct.mNumberOfAgents)
	{
		// Failure due to invalid agent id
		return false;
	}

	int _indexOfCityToInsertAfter = _indexOfAgent * inSolutionStruct.mNumberOfCities + inCityIdToInsertAfter - 1;
	// Index is 0-based (City ID 1 => index 0)
	int _indexOfCityToInsert = _indexOfAgent * inSolutionStruct.mNumberOfCities + inCityIdToInsert - 1;
	if (inSolutionStruct.mSolution[_indexOfCityToInsert] > 0
		|| inSolutionStruct.mSolution[_indexOfCityToInsertAfter] == 0)
	{
		// City has already been added to the path -> NO PLANNED HANDLING
		// OR City before is not on the path at all
		return false;
	}

	//debug_print_raw_solution(inSolutionPtr,
	//	inNumberOfCities, inNumberOfAgents);
	//debug_print_solution_agents(inSolutionPtr,
	//	inNumberOfCities, inNumberOfAgents);

	// Randomly select the position in the path to insert the task/city
	auto max = std::max_element(inSolutionStruct.mSolution + (_indexOfAgent * inSolutionStruct.mNumberOfCities), inSolutionStruct.mSolution + (_indexOfAgent + 1) * inSolutionStruct.mNumberOfCities);
	int _indexInPathToInsertCity = getRandomIntegerInRange(inSolutionStruct.mSolution[_indexOfCityToInsertAfter] + 1, *max+1);

	return insertCityToSolution(inSolutionStruct, inAgentId, inCityIdToInsert, _indexInPathToInsertCity);
}

bool insertCityToSolutionRandomly(SolutionStruct& inSolutionStruct,
	int inAgentId,
	int inCityIdToInsert)
{
	/*debug_print_raw_solution(inSolutionPtr,
		inNumberOfCities, inNumberOfAgents);
	debug_print_solution_agents(inSolutionPtr,
		inNumberOfCities, inNumberOfAgents);*/

	//debug_print_raw_solution(inSolutionPtr, inNumberOfCities, inNumberOfAgents);
	int _indexOfAgent = inAgentId - 1;
	if (_indexOfAgent < 0 || _indexOfAgent > inSolutionStruct.mNumberOfAgents)
	{
		std::cout << "Failed to insert task due to invalid agent id " << std::endl;
		return false;
	}

	int _indexOfCityToInsertAfter = _indexOfAgent * inSolutionStruct.mNumberOfCities;
	// Index is 0-based (City ID 1 => index 0)
	int _indexOfCityToInsert = _indexOfAgent * inSolutionStruct.mNumberOfCities + inCityIdToInsert - 1;
	if (inSolutionStruct.mSolution[_indexOfCityToInsert] > 0
		|| inSolutionStruct.mSolution[_indexOfCityToInsertAfter] == 0)
	{
		// City has already been added to the path -> NO PLANNED HANDLING
		// OR City before is not on the path at all
		return false;
	}

	// Randomly select the position in the path to insert the task/city
	auto max = std::max_element(inSolutionStruct.mSolution + (_indexOfAgent * inSolutionStruct.mNumberOfCities), inSolutionStruct.mSolution + (_indexOfAgent + 1) * inSolutionStruct.mNumberOfCities - 1);
	int maxRange = *max + 1;

	if (*max == -1) {
		maxRange = 1;
	}

	int _indexInPathToInsertCity = getRandomIntegerInRange(1, maxRange);

	return insertCityToSolution(inSolutionStruct, inAgentId, inCityIdToInsert, _indexInPathToInsertCity);
}

bool removeCityFromSolution(SolutionStruct& inSolutionStruct,
	int inAgentId,
	int inCityIdToRemove)
{
	int _indexOfAgent = inAgentId - 1;
	if (_indexOfAgent < 0 || _indexOfAgent > inSolutionStruct.mNumberOfAgents)
	{
		// Failure due to invalid agent id
		return false;
	}

	// Index is 0-based (City ID 1 => index 0)
	int _indexOfCityToRemove = inCityIdToRemove - 1;
	int _cityPositionInPath = inSolutionStruct.mSolution[_indexOfAgent * inSolutionStruct.mNumberOfCities + _indexOfCityToRemove];
	if (_cityPositionInPath == -1)
	{
		// City is not part of agent's path
		return false;
	}

	// Iterate over cities in agent to discover path
	for (int j = 0; j < inSolutionStruct.mNumberOfCities; j++)
	{
		int globalIndex = j + _indexOfAgent * inSolutionStruct.mNumberOfCities;
		if (inSolutionStruct.mSolution[globalIndex] > _cityPositionInPath)
		{
			inSolutionStruct.mSolution[globalIndex]--;
		}
	}
	//debug_print_raw_solution(inSolutionPtr,
	//	inNumberOfCities, inNumberOfAgents);
	inSolutionStruct.mSolution[_indexOfAgent * inSolutionStruct.mNumberOfCities + _indexOfCityToRemove] = -1;
	
	// No-error exit
	return true;
}

void taskSwappingInSolution(SolutionStruct& inSolutionStruct,
	std::pair<int, int> inCitiesToSwap, std::pair<int,int> & inAgents) {

	if (inAgents.first == inAgents.second) {
		int _tmp = inSolutionStruct.mSolution[inAgents.first * inSolutionStruct.mNumberOfCities + inCitiesToSwap.first];
		inSolutionStruct.mSolution[inAgents.first * inSolutionStruct.mNumberOfCities + inCitiesToSwap.first] = inSolutionStruct.mSolution[inAgents.first * inSolutionStruct.mNumberOfCities + inCitiesToSwap.second];
		inSolutionStruct.mSolution[inAgents.first * inSolutionStruct.mNumberOfCities + inCitiesToSwap.second] = _tmp;
	}
	else {

		inSolutionStruct.mSolution[inAgents.first * inSolutionStruct.mNumberOfCities + inCitiesToSwap.second] = inSolutionStruct.mSolution[inAgents.first * inSolutionStruct.mNumberOfCities + inCitiesToSwap.first];
		inSolutionStruct.mSolution[inAgents.first * inSolutionStruct.mNumberOfCities + inCitiesToSwap.first] = -1;
		
		inSolutionStruct.mSolution[inAgents.second * inSolutionStruct.mNumberOfCities + inCitiesToSwap.first] = inSolutionStruct.mSolution[inAgents.second * inSolutionStruct.mNumberOfCities + inCitiesToSwap.second];
		inSolutionStruct.mSolution[inAgents.second * inSolutionStruct.mNumberOfCities + inCitiesToSwap.second] = -1;

	}
}