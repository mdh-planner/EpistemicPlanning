#pragma once
#include "robot.h"
#include <filesystem>

class LOG {

public:
	std::vector<std::vector<Pose>> robotPose;
	std::vector<std::vector<std::vector<std::vector<Pose>>>> particlePose;
	std::vector<std::vector<Point>> goalLoc;

	std::string toString(const Pose& pose) const {
		// Return a string representation of Pose (e.g., "x y theta")
		// This is a placeholder implementation
		return std::to_string(pose.x) + " " + std::to_string(pose.y) + " " + std::to_string(pose.theta);
	}

	void writePose2File(std::vector<Robot>& robots) {

		int fileIndex = 1;
		std::string basefilename = "robotPose";
		std::string filename = basefilename + std::to_string(fileIndex) + ".txt";

		// Check if the file exists and create a new filename if it does
		while (std::filesystem::exists(filename)) {
			filename = basefilename + std::to_string(fileIndex) + ".txt";
			fileIndex++;
		}

		std::ofstream file(filename);

		if (!file.is_open()) {
			std::cerr << "Failed to open file: robotPose.txt" << std::endl;
			return;
		}

		size_t rows = robotPose[0].size();
		size_t columns = robotPose.size();
		std::vector<int> cnt(columns);

		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < columns; j++) {

				file << toString(robotPose[j][i]) << " ";

				if (cnt[j] < robots[j].taskReached.size()) {
					if (robots[j].taskReached[cnt[j]].first - 1 == i) {
						file << robots[j].taskReached[cnt[j]].second << " ";
						cnt[j]++;
					}
					else {
						file << -1 << " ";
					}
				}
				else {
					file << -1 << " ";
				}

				double x = goalLoc[j][i].x;
				double y = goalLoc[j][i].y;

				file << std::to_string(x) << " " << std::to_string(y) << " ";

			}
			file << "\n"; // Separate poses of different robots with a blank line
		}

		file.close();

	};

	void writeParticle2File() {
		int fileIndex = 1;
		std::string basefilename = "particlePose";
		std::string filename = basefilename + std::to_string(fileIndex) + ".txt";

		// Check if the file exists and create a new filename if it does
		while (std::filesystem::exists(filename)) {
			filename = basefilename + std::to_string(fileIndex) + ".txt";
			fileIndex++;
		}

		std::ofstream file(filename);

		if (!file.is_open()) {
			std::cerr << "Failed to open file: robotPose.txt" << std::endl;
			return;
		}

		size_t iter = particlePose[0][0][0].size();
		size_t robots = particlePose.size();
		size_t robotParticles = particlePose[0].size();
		size_t particles = particlePose[0][0].size();

		for (int i = 0; i < iter; i++) {
			for (int j = 0; j < robots; j++) {
				for (int k = 0; k < robotParticles; k++) {
					for (int p = 0; p < particlePose[j][k].size(); p++) {
						//std::cout << "i " << i << " j " << j << " k " << k << " p " << p << std::endl;
						file << toString(particlePose[j][k][p][i]) << " ";
					}
				}
			}
			file << "\n"; // Separate poses of different robots with a blank line
		}


		file.close();
	}



	void writeGoalLoc2File();
private:

};