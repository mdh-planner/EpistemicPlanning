#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

class Node_EP {
public:
    int city;
    std::vector<int> tour;
    std::vector<int> visited;
    int visits = 0;
    double value = 0.0;
    std::vector<std::shared_ptr<Node_EP>> children;
    std::shared_ptr<Node_EP> parent;
    // Assume 'state' type to be int for simplification
    int state;

    Node_EP() {}

    void initGraph(int _city, const std::vector<int>& _tour, std::shared_ptr<Node_EP> _parent) {
        city = _city;
        tour = _tour;
        parent = _parent;
        visited = tour;
        visited.push_back(city);
        sort(visited.begin(), visited.end());
        visited.erase(unique(visited.begin(), visited.end()), visited.end());
    }

    bool is_fully_expanded(int total_cities) {
        return children.size() == (total_cities - tour.size());
    }

    std::shared_ptr<Node_EP> best_child(double exploration_param) {
        std::vector<double> ucb_values(children.size(), 0.0);
        for (size_t i = 0; i < children.size(); ++i) {
            auto& child = children[i];
            double exploitation = child->value / (child->visits + 1e-10);
            double exploration = exploration_param * sqrt(log(visits + 1e-10) / (child->visits + 1e-10));
            ucb_values[i] = exploitation + exploration;
        }
        /*for (auto a : ucb_values) {
            std::cout << a << "\t";
        }
        std::cout << std::endl;*/

        auto bestIt = max_element(ucb_values.begin(), ucb_values.end());
        return children[distance(ucb_values.begin(), bestIt)];
    }
};

