#pragma once

// #include "plog\Log.h"
#include "../misc/debugging.h"
#include <vector>
#include <math.h>
// #include "plog/Initializers/RollingFileInitializer.h"
#include "../solution_representations/TSolution.h"
#include "../logging/logger.h"

#define OPTIMIZE
namespace mdutec
{
	class mdutecAlgo
	{
	public:
		mdutecAlgo(std::string inPathToLoggingDirectory = "D:\\LOGS\\ECTSP")
		{
			// plog::init(plog::warning, inPathToLoggingDirectory, 1000000, 5);
		}

		std::vector<int> run(int instance, int run, int nTasks, int mRobots)
		{

			// initialize ecdf logging vector
			clock_t t = clock();
			double runTime = 0;
			vector<pair<double, double>> eCDF;
			DepoSelection ddepotmat;

			// BLOG logger(instance, run);

			// Generate initial candidate solution
			RawSolutionPopulation _candidateSolutions = generateInitialCandidateSolutions(ddepotmat);

			// importSolution(_candidateSolutions);

			precedencePopulationReparation(_candidateSolutions);
			/*_candidateSolutions[0][0] = 1;
			_candidateSolutions[0][1] = 2;
			_candidateSolutions[0][2] = 3;
			_candidateSolutions[0][3] = 4;
			_candidateSolutions[0][4] = 5;

			_candidateSolutions[1][0] = 1;
			_candidateSolutions[1][1] = 4;
			_candidateSolutions[1][2] = 5;
			_candidateSolutions[1][3] = 2;
			_candidateSolutions[1][4] = 3;

			debug_print_solution_agents(_candidateSolutions[0], 5, 1);
			debug_print_solution_agents(_candidateSolutions[1], 5, 1);*/

			/*	if (nTasks == 2) {
					_candidateSolutions[0][0] = 1;
					_candidateSolutions[0][1] = 2;


					_candidateSolutions[1][0] = 2;
					_candidateSolutions[1][1] = 1;

				}*/

			mFinished = evaluateCandidateSolutions(_candidateSolutions, inSolutionMetadata);

			// eCDF.push_back(make_pair(inSolutionMetadata[0].mCost, (double)(clock() - t) / CLOCKS_PER_SEC));

#ifdef OPTIMIZE
			while (mFinished && mCleanFinish)
			{

				// std::cout << "Gen number: " << mNumOfIterations << " Best Solution: " << inSolutionMetadata[0].mCost << std::endl;
				// std::cout << inSolutionMetadata[0].mCost << std::endl;
				// debug_print_solution_agents(_candidateSolutions[inSolutionMetadata[0].mSolutionId],30, 2);
				// RawSolutionPopulation _candidateSolutions = generateInitialCandidateSolutions(ddepotmat);

				makeNewCandidateSolutions(_candidateSolutions); // Generate new candidate solution
				precedencePopulationReparation(_candidateSolutions);
				mFinished = evaluateCandidateSolutions(_candidateSolutions, inSolutionMetadata); // Evaluate the candidate solution

				//
				runTime = (double)(clock() - t) / CLOCKS_PER_SEC;
				// eCDF.push_back(make_pair(inSolutionMetadata[0].mCost, runTime));

				if (runTime > 2)
				{
					mFinished = false;
				}

				// // Stopping criterion is num of iterations
				// if (mNumOfIterations > 100) {
				//	mFinished = false;

				//}
				mNumOfIterations++;
			}
#endif

			// logger.logCDF(eCDF);
			// logger.orderedPlanLog(_candidateSolutions[inSolutionMetadata[0].mSolutionId], nTasks, mRobots, ddepotmat);
			// showBestSolution(_candidateSolutions);

			// Log
			/*std::cout << "Finished run of the Algorithm with candidate solution status: " << std::endl;
			std::cout << "Clean finish: " << std::endl;*/

			// Return inverse error state showing if no error has been found
			return returnSolution(_candidateSolutions[inSolutionMetadata[0].mSolutionId], nTasks, mRobots);
		}

	protected:
		virtual RawSolutionPopulation generateInitialCandidateSolutions(DepoSelection &ddepotmat) = 0;

		virtual bool evaluateCandidateSolutions(RawSolutionPopulation &, std::vector<SolutionMetaData> &) = 0;

		virtual void makeNewCandidateSolutions(RawSolutionPopulation &) = 0;

		virtual void precedencePopulationReparation(RawSolutionPopulation &) = 0;

		virtual void showBestSolution(RawSolutionPopulation &) = 0;

		virtual void importSolution(RawSolutionPopulation &) = 0; // import solution from a file, mostly used for debugging, verification and comparison.

		/// Protected functions for utility
		virtual DistMat calculateDistances(std::vector<City> &inCityData, std::vector<Salesperson> &inAgentData)
		{
			size_t _size = inCityData.size();
			DistMat _result = new long *[_size];
			for (size_t i = 0; i < inCityData.size(); i++)
			{
				_result[i] = new long[_size];
				auto ith_city = inCityData[i].getLocation();
				for (size_t j = 0; j < inCityData.size(); j++)
				{
					auto jth_city = inCityData[j].getLocation();
					_result[i][j] = sqrt(
						(ith_city.first - jth_city.first) * (ith_city.first - jth_city.first) + (ith_city.second - jth_city.second) * (ith_city.second - jth_city.second)); // / inAgentData[0].getSpeed(); // We divide by speed. For now all speeds are the same so no problem. But if this changes this needs to be adjusted.
				}
			}
			return _result;
		}

		virtual Precedences calculatePrecedences(std::vector<City> &inCityData)
		{
			///  EXPECTING CITY UUID TO GO FROM 1 TO X
			size_t _size = inCityData.size();
			Precedences _result = new int[_size * 2];

			for (size_t i = 0; i < _size; i++)
			{
				_result[i] = 0;
			}

			// This means that i+1 must precede inCityData[i].getPrecedenceCondition()
			for (size_t i = 0; i < _size; i++)
			{
				if (inCityData[i].getPrecedenceCondition() > 0)
				{
					// cout << inCityData[i].getPrecedenceCondition() << "   " << i + 1 << endl;
					_result[inCityData[i].getPrecedenceCondition()] = i + 1;
				}

				_result[i + inCityData.size()] = inCityData[i].getPrecedenceCondition() + 1;
			}

			// This means that i+1 must precede inCityData[i].getPrecedenceCondition()
			// for (size_t i = 0; i < _size; i++) {
			//	if (inCityData[i].getPrecedenceCondition() > 0) {
			//		//cout << inCityData[i].getPrecedenceCondition() << "   " << i << endl;
			//		_result[inCityData[i].getPrecedenceCondition()] = i;
			//	}
			//	//cout << "=" << i + inCityData.size()-1 << "   " << inCityData[i].getPrecedenceCondition() << endl;
			//	_result[i + inCityData.size() - 1] = inCityData[i].getPrecedenceCondition();
			//}
			/*	std::cout << std::endl;
				for (size_t i = 0; i < _size; i++) {
					std::cout << std::setw(3) << i << std::flush;
				}*/
			/*std::cout << std::endl;
			std::cout << "Precedence Constraints: " << std::endl;

			for (size_t i = 0; i < _size; i++) {
				if (_result[i] > 0) {
					std::cout << _result[_size + _result[i] - 1] << " <- " << _result[i] << std::endl;;
				}

			}
			std::cout << std::endl;*/

			// for (size_t i = _size; i < _size * 2; i++) {
			//	std::cout << std::setw(3) << _result[i] << std::flush;
			// }
			// std::cout << std::endl;
			return _result;
		}

		virtual AgentCityMat calculateAgentCityMatrix(std::vector<City> &inCityData,
													  std::vector<Salesperson> &inAgentData)
		{
			///  EXPECTING CITY UUID TO GO FROM 1 TO X
			size_t _size = inCityData.size();
			AgentCityMat _result = new bool *[inAgentData.size()];
			for (size_t i = 0; i < inAgentData.size(); i++)
			{
				_result[i] = new bool[_size];
				for (size_t j = 0; j < inCityData.size(); j++)
				{
					_result[i][j] = inAgentData[i].hasColor(inCityData[j].getColor());
					// std::cout << _result[i][j] << " " << std::flush;
				}
				std::cout << std::endl;
			}
			return _result;
		}

		virtual DistMat calculateSourceDepotsMatrix(std::vector<City> &inCityData, std::vector<Salesperson> &inAgentData)
		{

			DistMat _result = new long *[inAgentData.size()];

			for (size_t i = 0; i < inAgentData.size(); i++)
			{
				_result[i] = new long[inCityData.size()];
				auto ith_city = inAgentData[i].getStartLocation();

				for (size_t j = 0; j < inCityData.size(); j++)
				{
					auto jth_city = inCityData[j].getLocation();

					_result[i][j] = sqrt((ith_city.first - jth_city.first) * (ith_city.first - jth_city.first) + (ith_city.second - jth_city.second) * (ith_city.second - jth_city.second)); // / inAgentData[0].getSpeed(); // We divide by speed. For now all speeds are the same so no problem. But if this changes this needs to be adjusted.

					// std::cout << _result[i][j] << "    " << std::flush;
				}
				// std::cout << std::endl;
			}

			return _result;
		}

		virtual DepoSelection calculateDestinationDepotsMatrix(std::vector<City> &inCityData, std::vector<Salesperson> &inAgentData, std::vector<Depot> &inDepotData)
		{

			DepoSelection _result = new std::pair<long, int>[inCityData.size()];

			for (size_t i = 0; i < inCityData.size(); i++)
			{

				auto ith_city = inCityData[i].getLocation();
				_result[i].first = LONG_MAX;
				_result[i].second = -1;

				for (size_t j = 0; j < inDepotData.size(); j++)
				{
					auto jth_city = inDepotData[j].getLocation();
					auto _dist = sqrt(
						(ith_city.first - jth_city.first) * (ith_city.first - jth_city.first) + (ith_city.second - jth_city.second) * (ith_city.second - jth_city.second)); // / inAgentData[0].getSpeed(); // We divide by speed. For now all speeds are the same so no problem. But if this changes this needs to be adjusted.
					if (_dist < _result[i].first)
					{
						_result[i].first = _dist;
						_result[i].second = j;
					}
				}
			}

			return _result;
		}

		virtual Tasks2Agents calculateTasks2Agents(std::vector<City> &inCityData,
												   std::vector<Salesperson> &inAgentData)
		{
			///  EXPECTING CITY UUID TO GO FROM 1 TO X

			Tasks2Agents _result = new std::vector<int>[inAgentData.size()];

			for (size_t i = 0; i < inAgentData.size(); i++)
			{
				for (size_t j = 0; j < inCityData.size(); j++)
				{
					if (inAgentData[i].hasColor(inCityData[j].getColor()))
					{
						_result[i].push_back(j + 1);
						// std::cout << j + 1 << " " << std::flush;
					}
				}
				// std::cout << std::endl;
			}
			return _result;
		}

		virtual Agents2Tasks calculateAgents2Tasks(std::vector<City> &inCityData,
												   std::vector<Salesperson> &inAgentData)
		{

			Agents2Tasks _result = new std::vector<int>[inCityData.size()];

			for (size_t i = 0; i < inCityData.size(); i++)
			{
				for (size_t j = 0; j < inAgentData.size(); j++)
				{
					if (inAgentData[j].hasColor(inCityData[i].getColor()))
					{
						_result[i].push_back(j + 1);
						// std::cout << j + 1 << " " << std::flush;
					}
				}
				// std::cout << std::endl;
			}
			return _result;
		}

		/* Vuce sredi ovo da bude kako treba :) */
		virtual agentsColorIntersect calculateColorInstersection(std::vector<Salesperson> &inAgentData)
		{

			agentsColorIntersect aCI(inAgentData.size());

			for (size_t i = 0; i < inAgentData.size(); i++)
			{
				aCI[i].resize(inAgentData.size());

				for (size_t j = 0; j < inAgentData.size(); j++)
				{
					for (int k = 0; k < inAgentData[i].getColors().size(); k++)
					{
						if (inAgentData[i].getColors()[k] && inAgentData[j].getColors()[k])
						{
							aCI[i][j].push_back(k);
						}
					}
				}
			}

			return aCI;
		}

		virtual closestTask calculateClosestTasks(std::vector<City> &inCityData,
												  std::vector<Salesperson> &inAgentData)
		{

			closestTask _result(inCityData.size());

			for (size_t i = 0; i < inCityData.size(); i++)
			{
				_result[i].resize(inAgentData.size());
				for (size_t j = 0; j < inAgentData.size(); j++)
				{

					if (inAgentData[j].hasColor(inCityData[i].getColor()))
					{

						for (size_t jj = 0; jj < inCityData.size(); jj++)
						{

							if (inAgentData[j].hasColor(inCityData[jj].getColor()) && i != jj)
							{
								_result[i][j].push_back(jj);
							}
						}
					}
				}
			}

			return _result;
		}

	protected:
		// Flag for erroneous exit from algorithm
		bool mCleanFinish = true;
		// Flag for invalid setup -> used to prevent running
		bool mInvalidSetup = false;
		// Trackers for the optimization process
		long mNumOfIterations = 0;

	private:
		bool mFinished = false;
		std::pair<long, long> debugVal;
		std::vector<SolutionMetaData> inSolutionMetadata;
	};
}