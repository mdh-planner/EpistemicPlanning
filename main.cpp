#include "eigen-master/Eigen/dense"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "algorithm.h"
//#include "debugger.h"

 ///////Function to remove a file
bool remove_file(const std::string& filename) {
    if (std::remove(filename.c_str()) != 0) {
        std::cerr << "Error removing file: " << filename << std::endl;
        return false;
    }
    else {
       // std::cout << "File removed successfully: " << filename << std::endl;
        return true;
    }
}

// Function to copy a file
bool copy_file(const std::string& source, const std::string& destination) {
    std::ifstream src(source, std::ios::binary);
    std::ofstream dest(destination, std::ios::binary);

    if (!src.is_open()) {
        std::cerr << "Error opening source file: " << source << std::endl;
        return false;
    }

    if (!dest.is_open()) {
        std::cerr << "Error opening destination file: " << destination << std::endl;
        return false;
    }

    dest << src.rdbuf();

    if (!dest.good()) {
        std::cerr << "Error occurred while copying file." << std::endl;
        return false;
    }

    //std::cout << "File copied successfully from " << source << " to " << destination << std::endl;
    return true;
}

int main() {
	// Initialization parameters from MATLAB code

	std::string filename = "3bots";
	std::string seedDir = "C:/Users/bmc01/Desktop/SMC 2024/glocal-main/IROS24/examples";
	std::string init = seedDir + "/instance" + filename + ".txt";
	std::string alloc = seedDir + "/allocation" + filename + ".txt";

    std::string file_to_remove = "C:\\Users\\bmc01\\Desktop\\SMC 2024\\glocal-main\\IROS24\\rnd_value2.txt";
    std::string source_file = "C:\\Users\\bmc01\\Desktop\\SMC 2024\\glocal-main\\IROS24\\rnd_value.txt";
    std::string file_to_remove2 = "C:\\Users\\bmc01\\Desktop\\SMC 2024\\glocal-main\\IROS24\\datasample2.txt";
    std::string source_file2 = "C:\\Users\\bmc01\\Desktop\\SMC 2024\\glocal-main\\IROS24\\datasample.txt";


    // Remove the file
    remove_file(file_to_remove);
        // If file removed successfully, copy the source file to the destination
    copy_file(source_file, file_to_remove);
    
    // Remove the file
    remove_file(file_to_remove2);
    // If file removed successfully, copy the source file to the destination
    copy_file(source_file2, file_to_remove2);

	Algorithm alg(init, alloc);
	clock_t tStart = clock();
	alg.run();
	std::cout << "Total runtime:  " << (double)(clock() - tStart) / CLOCKS_PER_SEC << std::endl;
	return 0;
}




