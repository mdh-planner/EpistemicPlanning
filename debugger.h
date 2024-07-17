#pragma once

class debugger {
public:
	static void displayRobotInfo(const std::vector<Robot>& robots) {
		std::cout << "Displaying robot configurations and task locations:\n";

		for (const auto& robot : robots) {
			std::cout << "Robot ID: " << robot.ID << "\n";
			std::cout << "Depot: (" << robot.depot[0].x() << ", " << robot.depot[0].y() << ")\n";
			std::cout << "Tasks: ";
			for (const auto& taskId : robot.taskQueue) {
				// Correcting for zero-based indexing throughout

				auto taskPosition = robot.globalTaskLocs[taskId]; // Directly use taskId for 0-indexing
				std::cout << taskId << " (" << taskPosition(0) << ", " << taskPosition(1) << "), ";

			}
			std::cout << "\n\n";
		}

		std::cout << "Task Locations:\n";
		for (int i = 0; i < robots[0].globalTaskLocs.size(); ++i) {
			// Adjusting task display to be consistent with zero-based indexing
			std::cout << "Task " << i << ": (" << robots[0].globalTaskLocs[i].x() << ", " << robots[0].globalTaskLocs[i].y() << ")\n";
		}
	}
};

