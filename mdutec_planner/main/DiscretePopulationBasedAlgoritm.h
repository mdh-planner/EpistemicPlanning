#pragma once
#include <random>
#include "mdutecalgo.h"
#include "../heuristics/kopt.h"
#include "../heuristics/localOpt.h"
#include "../heuristics/globalOpt.h"
#include "../heuristics/measures.h"
#include "../heuristics/validations.h"
#include "../heuristics/precedence.h"
#include "../misc/randnumgen.h"
#include <algorithm> 

struct Bat {
	int mBatID;
	// Bat pulse rate ->
	double r;
	// Bat velocity ->
	double v;
	// Bat loudness -> 
	double A;
};

class DiscretePopulationBasedAlgoritm : public mdutec::mdutecAlgo {

public:
	DiscretePopulationBasedAlgoritm(unsigned long inNumberOfPopulation, MissionData& MD) {

		inCityData = MD.cities;
		inAgentData = MD.salesUnit;
		inDepotData = MD.depots;

		mNumberOfSolutionsInPopulation = inNumberOfPopulation;
		inNumberOfAgents = inAgentData.size();
		inNumberOfCities = inCityData.size();

		mPrecalcDataStruct.mDistanceMatrix = calculateDistances(inCityData, inAgentData);
		mPrecalcDataStruct.mPrecedenceMatrix = calculatePrecedences(inCityData);
		mPrecalcDataStruct.mAgentCityMatrix = calculateAgentCityMatrix(inCityData, inAgentData);
		mPrecalcDataStruct.mSourceDepotDistanceMatrix = calculateSourceDepotsMatrix(inCityData, inAgentData);
		mPrecalcDataStruct.mDestinationDepotDistanceMatrix = calculateDestinationDepotsMatrix(inCityData, inAgentData, inDepotData);
		mA2T = calculateAgents2Tasks(inCityData, inAgentData);
		mT2A = calculateTasks2Agents(inCityData, inAgentData);
		mACI = calculateColorInstersection(inAgentData);
		mPrecalcDataStruct.mNumberOfAgents = inNumberOfAgents;
		mPrecalcDataStruct.mNumberOfCities = inNumberOfCities;

		//debug_print_precedences(mPrecalcDataStruct.mPrecedenceMatrix, inNumberOfCities);
		/*debug_print_agent_dist_matrix(mPrecalcDataStruct.mSourceDepotDistanceMatrix, inNumberOfCities, inNumberOfAgents);
		debug_print_dist_matrix(mPrecalcDataStruct.mDistanceMatrix, inNumberOfCities);
		debug_print_depot_dist_matrix(mPrecalcDataStruct.mDestinationDepotDistanceMatrix, inNumberOfCities);*/
		for (auto i : inCityData) { mPrecalcDataStruct.mCityDuration.push_back(i.getDurationOfStay()); }

		cT = calculateClosestTasks(inCityData, inAgentData);
		//sort cT
		for (int i = 0; i < inCityData.size(); i++) {
			// Sort based on distance
			auto _lambdaSort = [&](const int& a, const int& b) { return mPrecalcDataStruct.mDistanceMatrix[i][a] < mPrecalcDataStruct.mDistanceMatrix[i][b]; };
			for (int j = 0; j < inAgentData.size(); j++) {
				std::sort(cT[i][j].begin(), cT[i][j].end(), _lambdaSort);
			}
		}

	}

	bool readInData()
	{
		return true;
	}

protected:

	virtual RawSolutionPopulation generateInitialCandidateSolutions(DepoSelection& ddepotmat) override {
		ddepotmat = mPrecalcDataStruct.mDestinationDepotDistanceMatrix;

		mPopulationOfSolutions = new RawSolution[mNumberOfSolutionsInPopulation];
		vector<int> numTasksPerAgent(inNumberOfAgents);

		for (size_t i = 0; i < mNumberOfSolutionsInPopulation; i++) {
			mPopulationOfSolutions[i] = new int[inNumberOfAgents * inNumberOfCities];
			for (size_t j = 0; j < inNumberOfCities * inNumberOfAgents; j++) {

				//initializing values to -1
				mPopulationOfSolutions[i][j] = -1;
			}
		}

		//for (size_t i = 0; i < mNumberOfSolutionsInPopulation/2; i++) {

		//	std::fill(numTasksPerAgent.begin(), numTasksPerAgent.end(), 1);
		//	mTaskCountPerAgent.push_back({});
		//	vector<int> lastAssigned(inNumberOfAgents);
		//	std::fill(lastAssigned.begin(), lastAssigned.end(), -1);

		//	vector<int> listOfTasks(inNumberOfCities);
		//	std::iota(listOfTasks.begin(), listOfTasks.end(), 0);
		//	std::shuffle(listOfTasks.begin(), listOfTasks.end(), std::random_device());

		//	for (size_t j = 0; j < inNumberOfCities; j++) {

		//		vector<std::pair<int, double>> agentFitness;
		//		vector<double> temp;
		//		double totalDist = 0;

		//		for (int k = 0; k < inNumberOfAgents; k++) {
		//			if (inAgentData[k].hasColor(inCityData[listOfTasks[j]].getColor())) {
		//				double dist = 0;
		//				if (lastAssigned[k] == -1) {
		//					dist = mPrecalcDataStruct.mSourceDepotDistanceMatrix[k][listOfTasks[j]];
		//				}
		//				else {
		//					dist = mPrecalcDataStruct.mDistanceMatrix[lastAssigned[k]][listOfTasks[j]];
		//				}

		//				totalDist += dist;
		//				temp.push_back(dist);
		//				agentFitness.push_back(make_pair(k, 0));

		//			}
		//		}

		//		auto _lambdaSort = [](const std::pair<int, double>& a, const std::pair<int, double>& b) { return a.second < b.second; };
		//		std::sort(agentFitness.begin(), agentFitness.end(), _lambdaSort);
		//		std::sort(temp.begin(), temp.end());
		///*		int ii = 0;

		//		for (auto riter = temp.rbegin(); riter != temp.rend(); riter++) {
		//			if (ii == 0) {
		//				agentFitness[ii].second = *riter / totalDist;
		//			}
		//			else {
		//				agentFitness[ii].second = agentFitness[ii - 1].second + *riter / totalDist;
		//			}
		//			ii++;
		//		}

		//		auto p = getRandomRealInRange(0.0, 1.0);

		//		int rndA = histc(p, agentFitness);*/

		//		int rndA = agentFitness[0].first;

		//		lastAssigned[rndA] = listOfTasks[j];
		//		mPopulationOfSolutions[i][(rndA * inNumberOfCities) + listOfTasks[j]] = numTasksPerAgent[rndA];
		//		numTasksPerAgent[rndA]++;
		//	}


		//	mTaskCountPerAgent[i] = numTasksPerAgent;
		//}

		for (size_t i = 0; i < mNumberOfSolutionsInPopulation; i++) {

			std::fill(numTasksPerAgent.begin(), numTasksPerAgent.end(), 1);
			mTaskCountPerAgent.push_back({});
			vector<int> lastAssigned(inNumberOfAgents);
			std::fill(lastAssigned.begin(), lastAssigned.end(), -1);

			vector<int> listOfTasks(inNumberOfCities);
			std::iota(listOfTasks.begin(), listOfTasks.end(), 0);
			std::shuffle(listOfTasks.begin(), listOfTasks.end(), std::random_device());

			for (size_t j = 0; j < inNumberOfCities; j++) {

				vector<std::pair<int, double>> agentFitness;
				vector<double> temp;
				double totalDist = 0;

				for (int k = 0; k < inNumberOfAgents; k++) {
					if (inAgentData[k].hasColor(inCityData[listOfTasks[j]].getColor())) {
						double dist = 0;
						if (lastAssigned[k] == -1) {
							dist = mPrecalcDataStruct.mSourceDepotDistanceMatrix[k][listOfTasks[j]] * 1.2;
						}
						else {
							dist = mPrecalcDataStruct.mDistanceMatrix[lastAssigned[k]][listOfTasks[j]];
						}


						dist += mPrecalcDataStruct.mDestinationDepotDistanceMatrix[listOfTasks[j]].first;

						totalDist += dist;
						temp.push_back(dist);
						agentFitness.push_back(make_pair(k, 0));

					}
				}

				auto _lambdaSort = [](const std::pair<int, double>& a, const std::pair<int, double>& b) { return a.second < b.second; };
				std::sort(agentFitness.begin(), agentFitness.end(), _lambdaSort);
				std::sort(temp.begin(), temp.end());
				/*		int ii = 0;

						for (auto riter = temp.rbegin(); riter != temp.rend(); riter++) {
							if (ii == 0) {
								agentFitness[ii].second = *riter / totalDist;
							}
							else {
								agentFitness[ii].second = agentFitness[ii - 1].second + *riter / totalDist;
							}
							ii++;
						}

						auto p = getRandomRealInRange(0.0, 1.0);

						int rndA = histc(p, agentFitness);*/

				int rndA = agentFitness[0].first;

				lastAssigned[rndA] = listOfTasks[j];
				mPopulationOfSolutions[i][(rndA * inNumberOfCities) + listOfTasks[j]] = numTasksPerAgent[rndA];
				numTasksPerAgent[rndA]++;
			}


			mTaskCountPerAgent[i] = numTasksPerAgent;
		}
		//	//std::cout << "ini pop" << std::endl;
		//	//debug_print_raw_solution(mPopulationOfSolutions[i], inNumberOfCities, inNumberOfAgents);
		//	//debug_print_solution_agents(mPopulationOfSolutions[i], inNumberOfCities, inNumberOfAgents);
		//}

		//for (size_t i = 0; i < mNumberOfSolutionsInPopulation; i++) {


		//	std::fill(numTasksPerAgent.begin(), numTasksPerAgent.end(), 1);
		//	mTaskCountPerAgent.push_back({});

		//	for (size_t j = 0; j < inNumberOfCities; j++) {


		//		//Randomly selects a task for an agent with appropriate color

		//		// do this in a smarter way
		//		int rndA = getRandomIntegerInRange(0, static_cast<int>(inNumberOfAgents - 1));

		//		while (!(inAgentData[rndA].hasColor(inCityData[j].getColor())))
		//		{
		//			rndA = getRandomIntegerInRange(0, static_cast<int>(inNumberOfAgents - 1));

		//			if (rndA == -1) {
		//				continue;
		//			}
		//		}

		//		if (rndA == -1) {
		//			continue;
		//		}

		//		mPopulationOfSolutions[i][(rndA * inNumberOfCities) + j] = numTasksPerAgent[rndA];
		//		numTasksPerAgent[rndA]++;
		//	}


		//	mTaskCountPerAgent[i] = numTasksPerAgent;

		//	//std::cout << "ini pop" << std::endl;
		//	//debug_print_raw_solution(mPopulationOfSolutions[i], inNumberOfCities, inNumberOfAgents);
		//	//debug_print_solution_agents(mPopulationOfSolutions[i], inNumberOfCities, inNumberOfAgents);
		//}

		return mPopulationOfSolutions;
	}

	virtual void makeNewCandidateSolutions(RawSolutionPopulation& inSolutionPopulation) override {

//		debug_print_path_cost(mPrecalcDataStruct, _bestSolutionStruct.mSolution, inNumberOfCities, inNumberOfAgents);

		_bestSolutionStruct.mSolution = inSolutionPopulation[mBestSolutionId];
		_bestSolutionStruct.mNumberOfCities = inNumberOfCities;
		_bestSolutionStruct.mNumberOfAgents = inNumberOfAgents;
		mGeneration++;
		_bestSolutionStruct.mGen = mGeneration;
		int _size = inNumberOfAgents * inNumberOfCities;
		vector<int> bestSol(_size);

		//debug_print_solution_agents(inSolutionPopulation[mBestSolutionId], inNumberOfCities, inNumberOfAgents);

		for (int cpy = 0; cpy < _size; cpy++) {
			bestSol[cpy] = inSolutionPopulation[mBestSolutionId][cpy];
		}
		//std::copy(inSolutionPopulation[mBestSolutionId], inSolutionPopulation[mBestSolutionId] + _size, back_inserter(bestSol));

		//debug_print_solution_agents(_bestSolutionStruct.mSolution, inNumberOfCities, inNumberOfAgents);
		//for (unsigned int i = 0; i <  1; i++)
		for (unsigned int i = 0; i < mNumberOfSolutionsInPopulation-1; i++)
		{

		

			SolutionStruct _solutionStruct;
			_solutionStruct.mSolution = inSolutionPopulation[i];
			_solutionStruct.mNumberOfCities = inNumberOfCities;
			_solutionStruct.mNumberOfAgents = inNumberOfAgents;
			_solutionStruct.mPop = i;

			//if (i == mBestSolutionId) {
			//	//debug_print_solution_agents(inSolutionPopulation[i], inNumberOfCities, inNumberOfAgents);
			//	//debug_print_path_cost(mPrecalcDataStruct, _solutionStruct.mSolution, inNumberOfCities, inNumberOfAgents);
			//	continue;
			//}

			int _dist = DistanceMetric::hummingDistance(_bestSolutionStruct, _solutionStruct);
			//std::cout << i << std::endl;

		/*	int _dist = DistanceMetric::levinsteinDistance(_bestSolutionStruct,
				_solutionStruct);*/

			if (_dist == 0) {
				//continue;
				_dist = 1;
			}

			unsigned int Vt = getRandomIntegerInRange(1, _dist);

			// is this ok?!
			Bat tmp; tmp.v = Vt;
			mPopulationBatBehavior.push_back(tmp);

			
			
			// Takes a task from one agent and inserts it in others agent path (can be inserted within the same agent)
			for (int mut = 0; mut < 3; mut++) {
				LocalOpt::insertTask(_solutionStruct, mPrecalcDataStruct, mA2T);
				GlobalOpt::insertClosestXAgent(_solutionStruct, mPrecalcDataStruct, mA2T, cT);
			}
			for (int mut = 0; mut < 3; mut++) {
				LocalOpt::swapTasks(_solutionStruct, mPrecalcDataStruct, mACI, inCityData);
				
			}
			
			if (Vt > inNumberOfCities / 4) { // tune this
				// 3-OPT
				ThreeOpt::runLocal(_solutionStruct, mPrecalcDataStruct);
			}
			else {
				// 2 OPT
				
				TwoOpt::runLocal(_solutionStruct, mPrecalcDataStruct);
			}
			
			
		}

		//debug_print_solution_agents(inSolutionPopulation[mNumberOfSolutionsInPopulation - 1], inNumberOfCities, inNumberOfAgents);

		//inSolutionPopulation[mNumberOfSolutionsInPopulation - 1] = _bestSolutionStruct.mSolution;
		//std::copy(bestSol.begin(), bestSol.end(), inSolutionPopulation[mNumberOfSolutionsInPopulation - 1]);
		for (int cpy = 0; cpy < _size; cpy++) {
			inSolutionPopulation[mNumberOfSolutionsInPopulation - 1][cpy] = bestSol[cpy];
		}


		//debug_print_solution_agents(inSolutionPopulation[mNumberOfSolutionsInPopulation - 1], inNumberOfCities, inNumberOfAgents);
	}

	virtual bool evaluateCandidateSolutions(RawSolutionPopulation& inSolutionPopulation, std::vector<SolutionMetaData>& inSolutionMetadata) {

		inSolutionMetadata.clear();

		// Calculate cost and other metadata metrics of solutions in population
		for (unsigned int i = 0; i < mNumberOfSolutionsInPopulation; i++)
		{

			SolutionStruct _solutionStruct; // why do we reinitialize this here?
			_solutionStruct.mSolution = inSolutionPopulation[i];
			_solutionStruct.mNumberOfCities = inNumberOfCities;
			_solutionStruct.mNumberOfAgents = inNumberOfAgents;
			_solutionStruct.mPop = i;

			SolutionMetaData _metadata;
			_metadata.mSolutionId = i;

			_metadata.mCost = calc_solution_cost(_solutionStruct, mPrecalcDataStruct);

			//_metadata.mValidSolution = validate(_solutionStruct, mPrecalcDataStruct); //Saving time. there shouldn't be any problems anymore :)

			inSolutionMetadata.push_back(_metadata);

		}

		// Sort and filter to get the best solution
		auto _lambdaSort = [](const SolutionMetaData& a, const SolutionMetaData& b) { return a.mCost < b.mCost; };
		std::sort(inSolutionMetadata.begin(), inSolutionMetadata.end(), _lambdaSort);

		// Store best solution ID for next iteration
		mBestSolutionId = inSolutionMetadata[0].mSolutionId;

		

		return true;
	}

	virtual void precedencePopulationReparation(RawSolutionPopulation& inSolutionPopulation) {
		for (size_t i = 0; i < mNumberOfSolutionsInPopulation; i++) {

			SolutionStruct _solutionStruct;
			_solutionStruct.mSolution = inSolutionPopulation[i];
			_solutionStruct.mNumberOfCities = inNumberOfCities;
			_solutionStruct.mNumberOfAgents = inNumberOfAgents;

			while (!precedenceCheck(_solutionStruct, mPrecalcDataStruct, true)) {
				precedenceReparation(_solutionStruct, mPrecalcDataStruct, mTaskCountPerAgent[i]);
			}
			//bool test = precedenceCheck(_solutionStruct, mPrecalcDataStruct, true);

		/*	if (!test) {
				std::cout << "i: " << i << std::endl;
				debug_print_solution_agents(inSolutionPopulation[i],
					inNumberOfCities, inNumberOfAgents);
			}*/

		}

	}

	virtual void showBestSolution(RawSolutionPopulation& inSolutionPopulation) {
		debug_print_solution_agents(inSolutionPopulation[mBestSolutionId], inNumberOfCities, inNumberOfAgents);
	}

	virtual void importSolution(RawSolutionPopulation& inSolutionPopulation) {

		RawSolution inPop = new int[inNumberOfAgents * inNumberOfCities];

		for (size_t j = 0; j < inNumberOfCities * inNumberOfAgents; j++) {
			inPop[j] = -1; //initializing values to -1
		}

		string MIPstart = "C:\\Users\\bmc01\\Desktop\\Tecnalia\\Data\\74\\Instance_3\\0_plan.txt";

		ifstream infile;
		infile.exceptions(ifstream::failbit | ifstream::badbit);
		infile.open(MIPstart.c_str());

		string str;

		char* line, * pch, * saveptr1;

		for (int ag = 0; ag < inNumberOfAgents; ag++) {

			int offset = ag * inNumberOfCities;

			getline(infile, str);
			line = strdup(str.c_str());
			pch = strtok_r(line, ",", &saveptr1);
			int counter = 1;

			if (atoi(pch) > inNumberOfCities) { continue; }

			inPop[atoi(pch) + offset - 1] = counter;

			while (true) {
				pch = strtok_r(NULL, ",", &saveptr1);

				if (atoi(pch) > inNumberOfCities) {break;}

				counter++;
				inPop[atoi(pch) + offset -1] = counter;

			}
		}

		inSolutionPopulation[0] = inPop;

		//debug_print_raw_solution(inPop, inNumberOfCities, inNumberOfAgents);
		//debug_print_solution_agents(inPop, inNumberOfCities, inNumberOfAgents);
	}
private:

	static int histc(double p, vector<pair<int, double>>& agentFitness) {
		int binNum = 0;

		if (0 <= p && p < agentFitness[binNum].second)
			return agentFitness[binNum].first;


		for (; binNum < agentFitness.size() - 1; ++binNum) {
			if (agentFitness[binNum].second <= p && p < agentFitness[binNum + 1].second) {
				return agentFitness[binNum].first;
			}
		}

	}

	unsigned long mNumberOfSolutionsInPopulation;
	unsigned long inNumberOfCities;
	unsigned int mNumberOfColors;
	unsigned long inNumberOfAgents;
	long mGeneration = -1;

	std::vector<City>  inCityData;
	std::vector<Salesperson> inAgentData;
	std::vector<Depot> inDepotData;

	SolutionStruct _bestSolutionStruct;

	DistMat mDistanceMatrix;
	Precedences mPrecedenceMatrix;
	AgentCityMat mAgentCityMatrix;
	PrecalculatedDataStruct mPrecalcDataStruct;

	std::vector<Bat> mPopulationBatBehavior;
	RawSolutionPopulation mPopulationOfSolutions;
	Agents2Tasks mA2T;
	Tasks2Agents mT2A;
	agentsColorIntersect mACI;

	closestTask cT;

	std::vector<std::vector<int>> mTaskCountPerAgent;

	// Data 
	int mBestSolutionId;
};