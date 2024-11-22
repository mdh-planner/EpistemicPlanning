// BasicECTSP.cpp : Defines the entry point for the application.
//

#include <fstream>
#include <iostream>
#include <sstream>
#include <iterator>

#include <vector>

#include <emmintrin.h>
#include <immintrin.h>
#include <malloc.h>

#include "../heuristics/precedence.h"
#include "../heuristics/measures.h"
#include "../heuristics/gene_manipulation.h"

#include "../misc/debugging.h"

using namespace std;


// Task swapping test
int test_task_swapping()
{
	//std::cout << std::endl << "Task swapping test" << std::endl;
	//int Nc = 6;
	//int Na = 2;
	//std::vector<int> solV = { 1,0,0,0,0,2, 0,1,4,2,3,0 };
	//RawSolution sol = solV.data();
	//std::cout << "Original: " << std::endl;
	//debug_print_solution_agents(sol, Nc, Na);
	//taskSwappingInSolution(sol, Nc, Na, std::make_pair(5, 6));
	//std::cout << std::endl << "Swapped 5 and 6: " << std::endl;
	//debug_print_solution_agents(sol, Nc, Na);
	//taskSwappingInSolution(sol, Nc, Na, std::make_pair(1, 5));
	//std::cout << std::endl << "Swapped again 1 and 5: " << std::endl;
	//debug_print_solution_agents(sol, Nc, Na);
	return 1;
}

// Task insertion/removing test
int test_task_insertion_removing()
{
	/*std::cout << std::endl << "Task insertion/removing test " << std::endl;
	int Nc = 6;
	int Na = 2;
	std::vector<int> solV = { 1,0,0,0,0,2, 0,1,4,2,3,0 };
	RawSolution sol = solV.data();
	std::cout << "Original: " << std::endl;
	debug_print_solution_agents(sol, Nc, Na);
	if (!insertCityToSolution(sol, Nc, Na, 1, 4, 3))
	{
		std::cout << "ERROR INSERTING" << std::endl;
		debug_print_solution_agents(sol, Nc, Na);
		return -1;
	}
	std::cout << std::endl << "Added city 4 to place 3 in agent 1 path:\n" << std::endl;
	debug_print_solution_agents(sol, Nc, Na);
	if (!removeCityFromSolution(sol, Nc, Na, 2, 4))
	{
		std::cout << "ERROR REMOVING" << std::endl;
		debug_print_solution_agents(sol, Nc, Na);
		return -1;
	}
	std::cout << std::endl << "Removed city 4 in agent 2 path:\n " << std::endl;
	debug_print_solution_agents(sol, Nc, Na);*/
	return 1;
}

// Precedence check test
//int test_precedence_check()
//{
//	int Nc = 6;
//	int Na = 3;
//	std::vector<int> solV = { 0,3,1,2,5,4, 0, 2, 1, 4, 3, 5, 0,4,5,1,2,3 };
//	RawSolution sol = solV.data();
//	std::vector<int> precCondV = { 0, 0, 0, 2, 0, 3 };
//	Precedences prec = precCondV.data();
//	std::cout << std::endl << std::endl;
//	debug_print_solution_agents(sol, Nc, Na);
//	if (!precedenceCheck(sol, Nc, Na, prec))
//	{
//		std::cout << "failed precedence check!" << std::endl;
//		return -1;
//
//	}
//	std::cout << "True!" << std::endl;
//	return 1;
//}

// Precedence reparation
//int test_precedence_reparation()
//{
//	std::cout << std::endl << "Precedence reparation test" << std::endl;
//	int Nc = 6;
//	int Na = 2;
//	std::vector<int> solV = { 0,3,1,2,5,4, 0,2,1,4,3,5 };
//	RawSolution sol = solV.data();
//	std::vector<int> precCondV = { 0, 0, 0, 2, 0, 3 };
//	Precedences prec = precCondV.data();
//	std::cout << "Original: " << std::endl;
//	debug_print_solution_agents(sol, Nc, Na);
//	bool** _agMat;
//	precedenceReparation(sol, Nc, Na, prec, _agMat);
//	if (!precedenceCheck(sol, Nc, Na, prec))
//	{
//		std::cout << "Failed the precedence check!" << std::endl;
//		debug_print_solution_agents(sol, Nc, Na);
//		debug_print_precedences(prec, Nc);
//		return -1;
//	}
//	debug_print_solution_agents(sol, Nc, Na);
//	return 1;
//}

// Task insertion before/after city test
//int test_task_insertion_before_after()
//{
//	std::cout << std::endl << " Task insertion before/after city test" << std::endl;
//	int Nc = 6;
//	int Na = 2;
//	std::vector<int> solV = { 1,0,0,0,0,2, 0,1,4,2,3,0 };
//	RawSolution sol = solV.data();
//	auto _print = [&]() {
//		for (int ia = 0; ia < Na; ia++)
//		{
//			std::cout << "Agent " << ia + 1 << ": ";
//			for (int ic = 0; ic < Nc; ic++)
//			{
//				std::cout << solV[ia * Nc + ic] << ",";
//			}
//			std::cout << std::endl;
//		}
//	};
//	std::cout << "Original: " << std::endl;
//	_print();
//	if (!insertCityToSolutionAfterCityId(sol, Nc, Na, 2, 1, 4))
//	{
//		std::cout << "ERROR INSERTING" << std::endl;
//		return -1;
//	}
//	if (!insertCityToSolutionBeforeCityId(sol, Nc, Na, 1, 3, 1))
//	{
//		std::cout << "ERROR INSERTING" << std::endl;
//		return -1;
//	}
//	_print();
//	return 1;
//}

//
//// Task swapping test
//int test_humming_distance()
//{
//	std::cout << std::endl << "Task swapping test" << std::endl;
//	int Nc = 6;
//	int Na = 2;
//	int Np = 4;
//	DistMat dist_mat = new long*[Nc];
//	for (int i = 0; i < Nc; i++)
//	{
//		dist_mat[i] = new long[Nc];
//	}
//	for (int i = 0; i < Nc; i++)
//		for (int j = 0; j < Nc; j++)
//			dist_mat[i][j] = 1;
//
//	std::vector<int> solV = { 1,0,0,0,0,2, 0,1,4,2,3,0 };
//	std::vector<int> solC = { 1,0,3,0,0,2, 0,1,0,2,3,0 };
//	RawSolution sol = solV.data();
//	RawSolutionPopulation pop1 = new RawSolution[Np];
//	for (int i = 0; i < Np; i++)
//	{
//		if (i > 1)
//			//pop1[i] = solC.data();
//		pop1[i] = new int[Nc * Na];
//		else
//			pop1[i] = solV.data();
//	}
//	RawSolution _bestSolution = pop1[0];
//	std::vector<SolutionMetaData> _solutionMetadata;
//	for (int i = 0; i < Np; i++)
//	{
//		SolutionMetaData _metadata;
//		_metadata.mSolutionId = i;
//		_metadata.mCost = calc_solution_cost(pop1[i], Nc, Na, dist_mat);
//		_solutionMetadata.push_back(_metadata);
//	}
//	auto _lambdaSort = [](const SolutionMetaData& a, const SolutionMetaData& b)
//	{
//		return a.mCost < b.mCost;
//	};
//	std::sort(_solutionMetadata.begin(), _solutionMetadata.end(), _lambdaSort);
//
//	for (int i = 0; i < _solutionMetadata.size(); i++)
//	{
//		std::cout << _solutionMetadata[i].mSolutionId << ": " << _solutionMetadata[i].mCost << " ";
//		debug_print_raw_solution(pop1[_solutionMetadata[i].mSolutionId], Nc, Na);
//		std::cout << std::endl;
//	}
//	std::cout << std::endl;
//
//	for (int i = 1; i < Np; i++)
//	{
//		long _res = Humming::hummingDistance(_bestSolution, pop1[i],
//			Nc, Na);
//		std::cout << "Humming dist for sol ID " << i << " is " << _res << std::endl;
//	}
//	
//	return 1;
//}