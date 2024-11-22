#pragma once
#include "../solution_representations/TSolution.h"

void debug_print_raw_solution(RawSolution& inSolPtr,
	int inNumberOfCities, int inNumberOfAgents)
{
	std::cout << std::endl;
	std::cout << "Debug printing raw solution..." << std::endl;
	std::cout << std::endl;
	for (int i = 0; i < inNumberOfAgents * inNumberOfCities; i++)
	{
		std::cout << inSolPtr[i] << "," << ((i + 1) % inNumberOfCities == 0 ? " | " : "");
	}
	std::cout << std::endl;
}

void debug_print_dist_matrix(DistMat& inDistMat, 
	int inNumberOfCities)
{
	std::cout << "Debug printing Distance matrix..." << std::endl;
	for (int i = 0; i < inNumberOfCities; i++)
	{
		for (int j = 0; j < inNumberOfCities; j++)
		{
			std::cout << inDistMat[i][j] << "\t";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}

void debug_print_agent_dist_matrix(DistMat& inDistMat,
	int inNumberOfCities, int inNumberOfAgents)
{
	std::cout << "Debug printing Agent-City Distance matrix..." << std::endl;
	for (int i = 0; i < inNumberOfAgents; i++)
	{
		for (int j = 0; j < inNumberOfCities; j++)
		{
			std::cout << inDistMat[i][j] << "\t";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}

void debug_print_depot_dist_matrix(DepoSelection& inDistMat,
	int inNumberOfCities)
{
	std::cout << "Debug printing City-Depot Distance matrix..." << std::endl;
	for (int i = 0; i < inNumberOfCities; i++)
	{
		
		
			std::cout << inDistMat[i].first << "\t";
		
		std::cout << std::endl;
	}
	std::cout << std::endl;
}


void debug_print_path_cost(PrecalculatedDataStruct & mPrecalcDataStruct, RawSolution& inSolPtr,
	int inNumberOfCities, int inNumberOfAgents)
{
	std::cout << "Debug printing path cost.." << std::endl;
	
	std::vector<int> total_path;
	for (int ia = 0; ia < inNumberOfAgents; ia++)
	{
		std::cout << "Agent " << ia + 1 << ": ";
		std::vector<int> _agentPath(inNumberOfCities, 0);
		int _pathLen = 0;
		for (int ic = 0; ic < inNumberOfCities; ic++)
		{
			int _ind = ia * inNumberOfCities + ic;
			//std::cout << inSolPtr[_ind] << " ";
			if (inSolPtr[_ind] > 0)
			{
				//std::cout << _ind << "   " << inSolPtr[_ind] - 1 << "  " << ic + 1 << std::endl;

				_agentPath[inSolPtr[_ind] - 1] = ic + 1;
				_pathLen++;
			}
		}
		//std::cout << std::endl << "\tPath: ";
		if (_pathLen > 0) {
			std::cout << mPrecalcDataStruct.mSourceDepotDistanceMatrix[ia][_agentPath[0]-1] << "\t";
			for (int i = 0; i < _pathLen - 1; i++)
			{
				std::cout << mPrecalcDataStruct.mDistanceMatrix[_agentPath[i] - 1][_agentPath[i + 1] - 1] << "\t";
			}

			std::cout << mPrecalcDataStruct.mDestinationDepotDistanceMatrix[_agentPath[_pathLen-1]-1].first << "\t";

			std::cout << std::endl;
		}

		//Add City Stay Duration to the overall cost
		long _cost = 0;
		for (int i = 0; i < _pathLen; i++) {
			if (_agentPath[i] >= 0) {
				_cost += mPrecalcDataStruct.mCityDuration[_agentPath[i]-1];
				std::cout << mPrecalcDataStruct.mCityDuration[_agentPath[i]-1] << std::endl;
			}
			else
				break;
		}

		copy(_agentPath.begin(), _agentPath.end(), back_inserter(total_path));
		/* Print output to be copied to my other planner for checking */
	}

	std::cout << std::endl;
}

void debug_print_precedences(Precedences& inPrec,
	int inNumberOfCities)
{
	std::cout << "Debug printing precedences..." << std::endl;
	for (int i = 0; i < inNumberOfCities+1; i++)
	{
		if (inPrec[i] > 0)
		{
			std::cout << inPrec[i] << "->" << i + 1 << "  ";
		}
	}
	std::cout << std::endl;
}

void debug_print_solution_agents(RawSolution& inSolPtr,
	int inNumberOfCities, int inNumberOfAgents)
{
	std::cout << std::endl;
	std::cout << "Debug printing agents in solution..." << std::endl;
	std::cout << std::endl;
	std::vector<int> total_path;
	for (int ia = 0; ia < inNumberOfAgents; ia++)
	{
		std::cout << "Agent " << ia + 1 << ": ";
		std::vector<int> _agentPath(inNumberOfCities, 0);
		int _pathLen = 0;
		for (int ic = 0; ic < inNumberOfCities; ic++)
		{
			int _ind = ia * inNumberOfCities + ic;
			//std::cout << inSolPtr[_ind] << " ";
			if (inSolPtr[_ind] > 0)
			{
			//std::cout << _ind << "   " << inSolPtr[_ind] - 1 << "  " << ic + 1 << std::endl;
				
				_agentPath[inSolPtr[_ind] - 1] = ic + 1;
				_pathLen++;
			}
		}
		//std::cout << std::endl << "\tPath: ";
		for (int i = 0; i < _pathLen; i++)
		{
			std::string _s = (i + 1 < _pathLen) ? "," : "";
			std::cout << _agentPath[i] << _s;
		}
		std::cout << std::endl;

		
		copy(_agentPath.begin(),_agentPath.end(),back_inserter(total_path));
		/* Print output to be copied to my other planner for checking */
	}

	//for (int i = 0; i < total_path.size(); i++) {
	//	if (i % inNumberOfCities == 0) {
	//		std::cout << "-1," << std::flush;
	//	}
	//	if (total_path[i] > 0) {
	//		std::cout << total_path[i]-1 << "," << std::flush;
	//	}
	//}

	//std::cout << std::endl;

	//for (int i = 0; i < total_path.size(); i++) {
	//	if (i % inNumberOfCities == 0) {
	//		std::cout << "1," << std::flush;
	//	}
	//	if (total_path[i] > 0) {
	//		std::cout << "14," << std::flush;
	//	}
	//}

	//std::cout << std::endl;
}

std::vector<int> returnSolution(RawSolution& inSolPtr,
	int inNumberOfCities, int inNumberOfAgents)
{
	std::vector<int> total_path;
	for (int ia = 0; ia < inNumberOfAgents; ia++)
	{
		
		std::vector<int> _agentPath(inNumberOfCities, 0);
		int _pathLen = 0;
		for (int ic = 0; ic < inNumberOfCities; ic++)
		{
			int _ind = ia * inNumberOfCities + ic;
			//std::cout << inSolPtr[_ind] << " ";
			if (inSolPtr[_ind] > 0)
			{
				//std::cout << _ind << "   " << inSolPtr[_ind] - 1 << "  " << ic + 1 << std::endl;

				_agentPath[inSolPtr[_ind] - 1] = ic + 1;
				_pathLen++;
			}
		}
		//std::cout << std::endl << "\tPath: ";
		/*for (int i = 0; i < _pathLen; i++)
		{
			std::string _s = (i + 1 < _pathLen) ? "," : "";
			std::cout << _agentPath[i] << _s;
		}
		std::cout << std::endl;*/


		copy(_agentPath.begin(), _agentPath.end(), back_inserter(total_path));
	}
	return total_path;
}

void debug_print_solution_population(RawSolutionPopulation& inSolutionPopulationPtr,
	int inNumberOfCities, int inNumberOfAgents, int inNumOfSolutions)
{
	std::cout << "Debug printing solutions..." << std::endl << std::endl;
	for (int i = 0; i < inNumOfSolutions; i++)
	{
		debug_print_raw_solution(inSolutionPopulationPtr[i], inNumberOfCities, inNumberOfAgents);
	}
	std::cout << std::endl;
}