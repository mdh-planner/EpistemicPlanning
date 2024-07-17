# CMakeList.txt : CMake project for BasicECTSP, include source and define
# project specific logic here.
project(BasicECTSP)
#
cmake_minimum_required (VERSION 3.8)

include_directories("logging")

# Add source to this project's executable.
add_executable (BasicECTSP "BasicECTSP.cpp" "BasicECTSP.h" "base/Salesman.h" "base/types.h" "misc/CsvReader.h" "base/City.h" "solution_representations/LinkedListArray.h" "solution_representations/solution_rep_v.h" "main/algorithm.h" "solution_representations/TSolution.h" "main/DiscretePopulationBasedAlgoritm.h" "heuristics/kopt.h" "heuristics/precedence.h" "heuristics/measures.h" "heuristics/validations.h" "heuristics/gene_manipulation.h" "misc/debugging.h" "misc/randnumgen.h" "heuristics/localOpt.h" "base/Depot.h" "misc/ImportData.h" )

# TODO: Add tests and install targets if needed.

