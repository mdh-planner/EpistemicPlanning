#include <iostream>
#include <string>
#include <cstdio>
#include <ctime>
#include <stdexcept>
#include <sstream>
#include <fstream>
#include <experimental/filesystem>

using namespace std;
namespace filesys = std::experimental::filesystem;

class BLOG
{
public:

	BLOG(int _instance, int nRun) {

		_nRun = nRun;
		instance = _instance;
		string directory_path(dir_path);
		bool resDir = checkIfDirectory(directory_path);

		if (!resDir) {
			createDir("", dir_path);
		}

		path = pathCreator(instance);
		instanceSettings();
	}

	void logCDF(vector<pair<double,double>> &ecdf)
	{
		// LOG eCDF, i.e., log best cost on every generation

		string path2 = path + to_string(_nRun) + "_eCDF.txt";
		ofstream myfile2(path2, ios_base::app | ios_base::out);

		for (int ecdfRun = 0; ecdfRun < ecdf.size(); ecdfRun++) {
			myfile2 << ecdf[ecdfRun].first << "\t" << ecdf[ecdfRun].second << endl;
		}

		myfile2.close();
	}

	void orderedPlanLog(RawSolution & sol, int mNumberOfCities, int mNumberOfAgents, DepoSelection &ddepotmat) {

		string path3 = path + to_string(_nRun) + "_plan.txt";
		ofstream myfile(path3, ios_base::app | ios_base::out);
		
		for (int ia = 0; ia < mNumberOfAgents; ia++)
		{
			myfile << flush;
			std::vector<int> _agentPath(mNumberOfCities, 0);
			int _pathLen = 0;
			for (int ic = 0; ic < mNumberOfCities; ic++)
			{
				int _ind = ia * mNumberOfCities + ic;
				//std::cout << inSolPtr[_ind] << " ";
				if (sol[_ind] > 0)
				{
					//std::cout << _ind << "   " << inSolPtr[_ind] - 1 << "  " << ic + 1 << std::endl;

					_agentPath[sol[_ind] - 1] = ic + 1;
					_pathLen++;
				}
			}
			//std::cout << std::endl << "\tPath: ";
			int i = 0;
			for (; i < _pathLen; i++)
			{
				//std::string _s = (i + 1 < _pathLen) ? "," : "";
				myfile << _agentPath[i] << ",";
			}

			if (i > 0) {
				myfile << ddepotmat[_agentPath[i - 1] - 1].second + mNumberOfCities + 1 << std::endl;
			} else {
				myfile << ddepotmat[_agentPath[0]].second + mNumberOfCities + 1 << std::endl;
			}
		}
		
		
	}

private:
	char dir_path[14] = "Test Results\\";
	RawSolution sol;
	string path;
	int _nRun; 
	int instance;

	void createDir(const char* name, char* dir_path) {

		//strcat(dir_path, path);
		strcat(dir_path, name);
		filesys::path dir(dir_path);

		if (filesys::create_directory(dir)) {
			cout << endl;
			std::cout << "Directory has been Successfully created" << "\n";
			cout << endl;
		}
	}

	void createDirS(string bla) {
		//strcat(dir_path, path);
		//strcat(dir_path, name);
		filesys::path dir(bla);

		if (filesys::create_directory(dir)) {
			cout << endl;
			std::cout << "Directory has been Successfully created" << "\n";
			cout << endl;
		}
	}

	bool checkIfDirectory(std::string& filePath)
	{
		try {
			// Create a Path object from given path string
			filesys::path pathObj(filePath);
			// Check if path exists and is of a directory file
			if (filesys::exists(pathObj) && filesys::is_directory(pathObj))
				return true;
		}
		catch (filesys::filesystem_error& e)
		{
			std::cerr << e.what() << std::endl;
		}
		return false;
	}

	string pathCreator(int instance) {

		int count;
		ifstream infile("counter.txt");
		infile >> count;

		string path = dir_path + to_string(count) + "\\";

		if (!checkIfDirectory(path)) {
			createDirS(path);
		}

		path += "Instance_" + to_string(instance);

		if (!checkIfDirectory(path)) {
			createDirS(path);
		}

		return path + "\\";
	};

	void instanceSettings() {


	//	ofstream myfile(path + "instanceSettings.txt", ios_base::app | ios_base::out);

	//	myfile << "Number_of_tasks " << ga->model->V << endl;
	//	myfile << "Number_of_runs " << _nRun << endl;
	//	myfile << "Population_size " << ga->POPULATION_SIZE << endl;
	//	myfile << "Number_of_generations " << ga->_maxGens << endl;
	//	myfile << "Chromosome_length " << ga->INDIVIDUAL_SIZE << endl;
	//	myfile << "Crossover_probability " << ga->pXOVER << endl;
	//	myfile << "Mutation_probability " << ga->pMUTATION << endl;
	//	myfile << "Kept_individuals " << ga->pELITE << endl;
	//	myfile << "Crossover_type " << ga->XOVER_TYPE << endl;
	//	//myfile << "Benchmark_Seed " << ga.seed << endl;
	//	//myfile << "Instance_Seed " << instanceSeed << endl;
	//	//myfile << "Run_Seed	" << flush;
	//	myfile.close();

	}

};