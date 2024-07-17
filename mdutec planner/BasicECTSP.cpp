// BasicECTSP.cpp : Defines the entry point for the application.
#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING 1
#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers

#include <fstream>
#include <iostream>
#include <sstream>
#include <iterator>
#include <emmintrin.h>
#include <immintrin.h>
#include <malloc.h>

#include "BasicECTSP.h"
#include "base/Salesman.h"
#include "base/City.h"
#include "base/Depot.h"
#include "misc/CsvReader.h"
#include "heuristics/precedence.h"
#include "heuristics/gene_manipulation.h"
//#include "tests/tests.h"
#include "main/DiscretePopulationBasedAlgoritm.h"
#include "misc/importData.h"

int main() {
	
	/*Update counter file*/
	int count = 0;
	ifstream infile("counter.txt"); infile >> count;
	ofstream count_file; count_file.open("counter.txt"); count_file << count + 1; count_file.close();
	/*------------------*/
	int startInstance = 0; int endInstance = 1;
	std::cout << "Select start instance: ";
	std::cin >> startInstance;
	std::cout << "Select end instance: ";
	std::cin >> endInstance;

	for (int instance = startInstance; instance < endInstance; instance++) {
		/*int _selectedDatasetIndex = 0;*/
		//std::cout << "Select index of dataset you wish to run: ";
		//std::cin >> _selectedDatasetIndex;
		//std::cout << std::string("You have selected dataset ") + std::to_string(_selectedDatasetIndex) + std::string("!") << std::endl;
		std::string _datasetExtension = std::string("_") + std::to_string(instance) + std::string(".csv");

		// ImportData from files
		MissionData MD = ImportData::loadData(_datasetExtension);
		ImportData::printLoadedData(MD);
		int maxRun = 30;


		/* We need to define Algorithm Parameters somewhere, for now they are all over the place */
		unsigned long populationSize = 201;

		for (int run = 0; run < maxRun; run++) {
			clock_t tStart2 = clock();

			mdutec::Algorithm* algo = new DiscretePopulationBasedAlgoritm(populationSize, MD);

			algo->run(instance, run, MD.cities.size(), MD.salesUnit.size());

			std::cout << std::endl;
			printf("Run time: %.2fs\n", (double)(clock() - tStart2) / CLOCKS_PER_SEC);
		}
	}
	return 0;
}