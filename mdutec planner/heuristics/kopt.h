#pragma once
#include "..\main\algorithm.h"


class PathManipulation {

public:
	static void decodePath(SolutionStruct& inSolutionStruct, vector<vector<int>>& _agentPath) {

		_agentPath.resize(inSolutionStruct.mNumberOfAgents);
		
		for (int ia = 0; ia < inSolutionStruct.mNumberOfAgents; ia++) {

			bool emptyFlag = true;
			_agentPath[ia].resize(2*inSolutionStruct.mNumberOfCities);
			int length = 0;

			for (int ic = 0; ic < inSolutionStruct.mNumberOfCities; ic++) {

				int _ind = ia * inSolutionStruct.mNumberOfCities + ic;

				if (inSolutionStruct.mSolution[_ind] > 0) {
					_agentPath[ia][inSolutionStruct.mSolution[_ind] - 1] = ic + 1;
					length++; emptyFlag = false;
				}
			}

			if (!emptyFlag) {
				_agentPath[ia].insert(_agentPath[ia].begin(), _agentPath[ia][0]); // not very efficient way to do this.
				length++;
				_agentPath[ia][length] = _agentPath[ia][length - 1];
				length++;
			}
			_agentPath[ia].resize(length);
		}
	}

	static void encodePath(SolutionStruct& inSolutionStruct, vector<int> _path, int agent) {

		int _offSet = agent * inSolutionStruct.mNumberOfCities;
		std::fill(&inSolutionStruct.mSolution[_offSet], &inSolutionStruct.mSolution[_offSet + inSolutionStruct.mNumberOfCities], -1);

		for (int i = 0; i < _path.size() - 2; i++) {
			inSolutionStruct.mSolution[_path[i + 1] + _offSet - 1] = i + 1;	
		}
	}

	static long calcAgentsCost(vector<int> _path, PrecalculatedDataStruct& inPrecalcDataStruct, const int & a) {

		long cost = 0;
		int i = 1;

		cost += inPrecalcDataStruct.mSourceDepotDistanceMatrix[a][_path[1] - 1];
		for (; i < _path.size() - 2; i++) {
			cost += inPrecalcDataStruct.mDistanceMatrix[_path[i] - 1][_path[i + 1] - 1];
		}
		cost += inPrecalcDataStruct.mDestinationDepotDistanceMatrix[_path[i] - 1].first;

		return cost;
	}

	static bool twoOptPrecCheck(vector<int>& path,
		Precedences& inPrecedenceConditions, int i, int k) {

		for (; k > i; k--) {
			if (inPrecedenceConditions[path[k] - 1] > 0) {
				int _cityIdToPrecedeOffset = inPrecedenceConditions[path[k] - 1];
				for (int j = k - 1; j >= i; j--) {
					if (path[j] == _cityIdToPrecedeOffset)
						return false;
				}
			}
		}
		return true;
	}

	static bool heuristicPrecCheck(vector<int>& path,
		Precedences& inPrecedenceConditions, int i, int k) {

		if (inPrecedenceConditions[path[k] - 1] > 0) {
			int _cityIdToPrecedeOffset = inPrecedenceConditions[path[k] - 1];
			for (int j = k - 1; j >= i; j--) {
				if (path[j] == _cityIdToPrecedeOffset)
					return false;
			}
		}

		return true;
	}
};

/**
* Implementation of the 2-opt algorithm for heuristical
* local optimization of the graph path
*/
class TwoOpt
{

public:
	static void runLocal(SolutionStruct& inSolutionStruct, PrecalculatedDataStruct& inPrecalcDataStruct) {

		std::vector<vector<int>> decodedPath;
		PathManipulation::decodePath(inSolutionStruct, decodedPath);

		for (int a = 0; a < inSolutionStruct.mNumberOfAgents; a++) {

			if (decodedPath[a].empty()) { continue; }

			bool improve = true, improved = false;

			while (improve && decodedPath[a].size() > 2) {
				improve = false;
				for (int i = 0; i < decodedPath[a].size() - 2; i++) { 
					for (int k = i + 1; k < decodedPath[a].size() - 1; k++) {
						long _pathDifference = calculateDiff(decodedPath[a], inPrecalcDataStruct, i, k, a);
						if (_pathDifference > 0) {
							if (PathManipulation::twoOptPrecCheck(decodedPath[a], inPrecalcDataStruct.mPrecedenceMatrix, i, k)) {
								std::reverse(decodedPath[a].begin() + i + 1, decodedPath[a].begin() + k + 1);
								improve = true; improved = true;
								break;
							}
							else{
						
								long _pDiff = calcPathChange(decodedPath[a], inPrecalcDataStruct, i, k, a);
								
								if (_pDiff > 0 && PathManipulation::heuristicPrecCheck(decodedPath[a], inPrecalcDataStruct.mPrecedenceMatrix, i, k)) {
									//this can be done more efficiently (don't do it in decoded path, path do it after encoding), this is just for testing
									int tmp = decodedPath[a][k];
									decodedPath[a].erase(decodedPath[a].begin() + k);						
									decodedPath[a].insert(decodedPath[a].begin() + i + 1, tmp);
									improve = true; improved = true;
									break;
								}
							}
						}
					}
				}
			}
			if (improved)
				PathManipulation::encodePath(inSolutionStruct, decodedPath[a], a); // encodes the solution back into the population;
		}
	}

private:

	static long calculateDiff(std::vector<int>& path,
		PrecalculatedDataStruct& inPrecalcDataStruct,
		const int& i, const int& k, const int& a) {

		long i_to_ip1, k_to_kp1, i_to_k, ip1_to_kp1;

		if (i == 0) {
			i_to_ip1 = inPrecalcDataStruct.mSourceDepotDistanceMatrix[a][path[i + 1] - 1];
			i_to_k = inPrecalcDataStruct.mSourceDepotDistanceMatrix[a][path[k] - 1];
		}
		else {
			i_to_ip1 = inPrecalcDataStruct.mDistanceMatrix[path[i] - 1][path[i + 1] - 1];
			i_to_k = inPrecalcDataStruct.mDistanceMatrix[path[i] - 1][path[k] - 1];
		}
			
		if (k == path.size() - 2) {
			k_to_kp1 = inPrecalcDataStruct.mDestinationDepotDistanceMatrix[path[k] - 1].first;
			ip1_to_kp1 = inPrecalcDataStruct.mDestinationDepotDistanceMatrix[path[i + 1] - 1].first;
		}
		else {
			k_to_kp1 = inPrecalcDataStruct.mDistanceMatrix[path[k] - 1][path[k + 1] - 1];
			ip1_to_kp1 = inPrecalcDataStruct.mDistanceMatrix[path[i + 1] - 1][path[k + 1] - 1];
		}

		long _pathDifference = i_to_ip1 + k_to_kp1 - i_to_k - ip1_to_kp1;

		//long _pathDifference = inPrecalcDataStruct.mDistanceMatrix[path[i - 1] - 1][path[i] - 1]
		//	+ inPrecalcDataStruct.mDistanceMatrix[path[k] - 1][path[k + 1] - 1]
		//	- inPrecalcDataStruct.mDistanceMatrix[path[i - 1] - 1][path[k] - 1]
		//	- inPrecalcDataStruct.mDistanceMatrix[path[i] - 1][path[k + 1] - 1];

		return _pathDifference;
	}

	static long calcPathChange(std::vector<int>& path,
		PrecalculatedDataStruct& inPrecalcDataStruct,
		const int& i, const int& k, const int& a) {

		long i_to_ip1, i_to_k, k_to_ip1, km1_to_kp1, km1_to_k, k_to_kp1;

		if (i == 0) {
			i_to_ip1 = inPrecalcDataStruct.mSourceDepotDistanceMatrix[a][path[i + 1] - 1];
			i_to_k = inPrecalcDataStruct.mSourceDepotDistanceMatrix[a][path[k] - 1];
		}
		else {
			i_to_ip1 = inPrecalcDataStruct.mDistanceMatrix[path[i] - 1][path[i + 1] - 1];
			i_to_k = inPrecalcDataStruct.mDistanceMatrix[path[i] - 1][path[k] - 1];
		}

		if (k == path.size() - 2) {
			k_to_kp1 = inPrecalcDataStruct.mDestinationDepotDistanceMatrix[path[k] - 1].first;
			km1_to_kp1 = inPrecalcDataStruct.mDestinationDepotDistanceMatrix[path[k - 1] - 1].first;

		}
		else {
			k_to_kp1 = inPrecalcDataStruct.mDistanceMatrix[path[k] - 1][path[k + 1] - 1];
			km1_to_kp1 = inPrecalcDataStruct.mDistanceMatrix[path[k - 1] - 1][path[k + 1] - 1];
		}

		k_to_ip1 = inPrecalcDataStruct.mDistanceMatrix[path[k] - 1][path[i + 1] - 1];
		km1_to_k = inPrecalcDataStruct.mDistanceMatrix[path[k - 1] - 1][path[k] - 1];


		long _pathDifference = (km1_to_k + k_to_kp1 + i_to_ip1) - (i_to_k + k_to_ip1 + km1_to_kp1);

		return _pathDifference;
	}
};


class ThreeOpt {

public:
	static void runLocal(SolutionStruct& inSolutionStruct, PrecalculatedDataStruct& inPrecalcDataStruct) {

		std::vector<vector<int>> decodedPath;
		std::vector<long> d(5);
		std::vector<pair<long, int>> sortedMoveList;

		PathManipulation::decodePath(inSolutionStruct, decodedPath);

		for (int a = 0; a < inSolutionStruct.mNumberOfAgents; a++) {

			if (decodedPath[a].empty()) { continue; }

			bool improve = true, improved = false;

			while (improve && decodedPath[a].size() > 4) {
				improve = false;	
				for (int i = 0; i < decodedPath[a].size() - 5; i++) { 
					for (int j = i + 2; j < decodedPath[a].size() - 3; j++) {
						for (int k = j + 2; k < decodedPath[a].size() - 1; k++) {

							sortedMoveList.clear();
							calculateDistances(decodedPath[a], d, inPrecalcDataStruct, i, j, k, a);
							chooseMove(decodedPath[a], d, sortedMoveList);

							if (sortedMoveList.empty()) { continue; 
							} else if (doSwap(decodedPath[a], d, inPrecalcDataStruct.mPrecedenceMatrix, sortedMoveList, i, j, k)) {
								improve = true; improved = true;
								break;
							}
						}
					}
				}
				if (improved) {
					PathManipulation::encodePath(inSolutionStruct, decodedPath[a], a); // encodes the solution back into the population;
					//break; // no repetititions, it goes once through the path
				}
			}	
		}
	}

private:
	static void calculateDistances(std::vector<int>& path,
		std::vector<long>& d, PrecalculatedDataStruct& inPrecalcDataStruct,
		const int& i, const int& j, const int& k, const int& a) {

		//edge distance calculations. Hardcoded for the 4 cases of 3opt.
		long i_to_ip1, i_to_j, i_to_k, i_to_jp1, k_to_kp1, jp1_to_kp1, j_to_kp1, ip1_to_kp1;

		if (i == 0) {
			i_to_ip1 = inPrecalcDataStruct.mSourceDepotDistanceMatrix[a][path[i + 1] - 1];
			i_to_j = inPrecalcDataStruct.mSourceDepotDistanceMatrix[a][path[j] - 1];
			i_to_k = inPrecalcDataStruct.mSourceDepotDistanceMatrix[a][path[k] - 1];
			i_to_jp1 = inPrecalcDataStruct.mSourceDepotDistanceMatrix[a][path[j + 1] - 1];
		}
		else {
			i_to_ip1 = inPrecalcDataStruct.mDistanceMatrix[path[i] - 1][path[i + 1] - 1];
			i_to_j = inPrecalcDataStruct.mDistanceMatrix[path[i] - 1][path[j] - 1];
			i_to_k = inPrecalcDataStruct.mDistanceMatrix[path[i] - 1][path[k] - 1];
			i_to_jp1 = inPrecalcDataStruct.mDistanceMatrix[path[i] - 1][path[j + 1] - 1];
		}

		if (k == path.size() - 2) {
			k_to_kp1 = inPrecalcDataStruct.mDestinationDepotDistanceMatrix[path[k] - 1].first;
			jp1_to_kp1 = inPrecalcDataStruct.mDestinationDepotDistanceMatrix[path[j + 1] - 1].first;
			j_to_kp1 = inPrecalcDataStruct.mDestinationDepotDistanceMatrix[path[j] - 1].first;
			ip1_to_kp1 = inPrecalcDataStruct.mDestinationDepotDistanceMatrix[path[i + 1] - 1].first;
		}
		else {
			k_to_kp1 = inPrecalcDataStruct.mDistanceMatrix[path[k] - 1][path[k + 1] - 1];
			jp1_to_kp1 = inPrecalcDataStruct.mDistanceMatrix[path[j + 1] - 1][path[k + 1] - 1];
			j_to_kp1 = inPrecalcDataStruct.mDistanceMatrix[path[j] - 1][path[k + 1] - 1];
			ip1_to_kp1 = inPrecalcDataStruct.mDistanceMatrix[path[i + 1] - 1][path[k + 1] - 1];
		}

		d[0] = i_to_ip1
			+ inPrecalcDataStruct.mDistanceMatrix[path[j] - 1][path[j + 1] - 1]
			+ k_to_kp1;

		d[1] = i_to_j
			+ inPrecalcDataStruct.mDistanceMatrix[path[i + 1] - 1][path[k] - 1]
			+ jp1_to_kp1;

		d[2] = i_to_k
			+ inPrecalcDataStruct.mDistanceMatrix[path[j + 1] - 1][path[i + 1] - 1]
			+ j_to_kp1;

		d[3] = i_to_jp1
			+ inPrecalcDataStruct.mDistanceMatrix[path[k] - 1][path[j] - 1]
			+ ip1_to_kp1;

		d[4] = i_to_jp1
			+ inPrecalcDataStruct.mDistanceMatrix[path[k] - 1][path[i + 1] - 1]
			+ j_to_kp1;
	}

	static void chooseMove(std::vector<int>& path,
		std::vector<long>& d, std::vector<pair<long, int>>& sortedMoveList) {

		sortedMoveList.reserve(4);

		for (int i = 1; i < d.size(); i++) {
			if (d[0] - d[i] > 0)
				sortedMoveList.push_back(make_pair(d[0] - d[i], i));
		}

		std::sort(sortedMoveList.begin(), sortedMoveList.end(), [](auto& left, auto& right) {
			return left.first < right.first;
		});

	}

	static bool doSwap(std::vector<int>& path,
		std::vector<long>& d, Precedences& inPrecedenceConditions,
		std::vector<pair<long, int>>& sortedMoveList, int i, int j, int k) {

		// pick the first move from the list and do prec check, if it is ok do it, otherwise move to the next move in the list.
		int _case = 0;
		for (int ii = 0; ii < sortedMoveList.size(); ii++) {
			if (threeOptPrecCheck(path, inPrecedenceConditions, sortedMoveList[ii].second, i, j, k)) {
				_case = sortedMoveList[ii].second;  break;
			}
		}

		switch (_case) {
		case 1:
			std::reverse(path.begin() + i + 1, path.begin() + j + 1);
			std::reverse(path.begin() + j + 1, path.begin() + k + 1);
			return true;
		case 2:
			std::reverse(path.begin() + i + 1, path.begin() + k + 1);
			std::reverse(path.begin() + i + (k - j) + 1, path.begin() + k + 1);
			return true;
		case 3:
			std::reverse(path.begin() + i + 1, path.begin() + k + 1);
			std::reverse(path.begin() + i + 1, path.begin() + i + (k - j) + 1);
			return true;
		case 4:
			std::reverse(path.begin() + i + 1, path.begin() + k + 1);
			std::reverse(path.begin() + i + 1, path.begin() + i + (k - j) + 1);
			std::reverse(path.begin() + i + (k - j) + 1, path.begin() + k + 1);
			return true;
		default:
			return false;
		}
	}

	static bool threeOptPrecCheck(std::vector<int>& path, Precedences& inPrecedenceConditions, int _case, int i, int j, int k) {

		bool check1 = false, check2 = false, check3 = false;
		switch (_case)
		{
		case 1:
			check1 = PathManipulation::twoOptPrecCheck(path, inPrecedenceConditions, i + 1, j);
			check2 = PathManipulation::twoOptPrecCheck(path, inPrecedenceConditions, j + 1, k);
			if (check1 && check2)
				return true;
			else
				return false;
		case 2:
			check1 = PathManipulation::twoOptPrecCheck(path, inPrecedenceConditions, i + 1, k);
			check2 = PathManipulation::twoOptPrecCheck(path, inPrecedenceConditions, i + 1, j);
			if (check1 && check2)
				return true;
			else
				return false;
		case 3:
			check1 = PathManipulation::twoOptPrecCheck(path, inPrecedenceConditions, i + 1, k);
			check2 = PathManipulation::twoOptPrecCheck(path, inPrecedenceConditions, i + 1, j);
			if (check1 && check2)
				return true;
			else
				return false;
		case 4:
			check1 = PathManipulation::twoOptPrecCheck(path, inPrecedenceConditions, i + 1, k);
			check2 = PathManipulation::twoOptPrecCheck(path, inPrecedenceConditions, i + 1, j);
			check3 = PathManipulation::twoOptPrecCheck(path, inPrecedenceConditions, j + 1, k);
			if (check1 && check2 && check3)
				return true;
			else
				return false;
		default:
			return false;
		}
	}
};