#pragma once
#include <iostream>
#include <vector>

struct Point {
	double x, y;
	// Constructor
	Point(double x = -1.0, double y = -1.0) : x(x), y(y) {}

	// Define the subtraction operator for Point
	Point operator-(const Eigen::Vector2d& other) const {
		return { x - other.x(), y - other.y() }; // Subtract corresponding components
	}

	// Overload the division operator for division by a scalar
	Point operator/(double scalar) const {
		if (scalar == 0) {
			std::cout << "Division by zero." << std::endl;
		}
		return Point(x / scalar, y / scalar);
	}

	// Overload the multiplication operator for multiplication by a scalar
	Point operator*(double scalar) const {
		return Point(x * scalar, y * scalar);
	}

	// Scalar multiplication operator
	void operator*=(double scalar) {
		x *= scalar;
		y *= scalar;
	}
	// Calculate the Euclidean norm of the point
	double norm() const {
		return sqrt(x * x + y * y);
	}

	bool empty() const {
		if (x != -1.0 && y != -1.0) {
			return false;
		}

		return true;
	}

};

#ifndef POSE_H
#define POSE_H

typedef struct Pose {
	double x, y, theta;
};

#endif // MY_STRUCT_H



struct MAP {
	std::pair<int, int> size;
	std::vector<std::vector<int>> gridPoints;
	std::vector<std::vector<int>> obs;
	std::vector<std::vector<double>> M0;
};

struct Plan {
	float x, y;
	int tq1, tq2;
	
	double tq3;
	int	id;
	int order; // 1 order is important / 0 order is not important
	// Overload the equality operator
	bool operator==(const Plan& other) const {
		return x == other.x && y == other.y;
	}
};

//Dictionary for task assignmentand dynamic tsp reward
// Dict is current location(1:2), robot index(3), particle(4), min rz time(5 - when applicable > 0
struct taskDict {
	std::vector<double> one; //newTaskLocs x
	std::vector<double> two; //newTaskLocs y
	std::vector<int> three; //newTaskBots;
	std::vector<int> four; // particles
	std::vector<double> five; //min rz time
};

struct PResult {
	std::vector<std::shared_ptr<Node_EP>> root;
	std::vector<Pose> tour_states;
	std::vector<int> best_tour;
	std::vector<taskDict> tasksAssigned;
};