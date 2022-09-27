
#include <cstdint>
#include <ros/package.h>
#include <ros/ros.h>

#include "or_tools_catkin/or_interface.h"

namespace or_tools_catkin {
using namespace operations_research;

OrInterface::OrInterface(int num_nodes) :
        num_vehicles_(1),
        depot_{num_nodes - 2} // TODO(stlucas): change to num_nodes -2 at some point!
{
    ROS_ASSERT(num_nodes > 1);

    search_params_ = DefaultRoutingSearchParameters();
    search_params_.set_first_solution_strategy(
            FirstSolutionStrategy::PATH_CHEAPEST_ARC); //May adapt this!
}

bool OrInterface::loadGTSP(std::vector <std::vector<int>> &adjacency_matrix,
                       std::vector <std::vector<int>> &clusters) {

    int num_nodes = adjacency_matrix.size();
    clusters_ = clusters;
    // Noon and bean transform: (currently implemented)
    // https://www.researchgate.net/publication/265366022_An_Efficient_Transformation_Of_The_Generalized_Traveling_Salesman_Problem

    // Improved (maybe simpler): Generalized network design problems
    // https://www.degruyter.com/document/doi/10.1515/9783110267686.60/html?lang=de

    //int m = INT_MAX / 4; // / Precision;

    long int total_sum = 0;
    for(std::vector<int>& row : adjacency_matrix) {
        for(int& element: row){
            total_sum += element;
        }
    }
    int m = total_sum;

    std::vector<int> row(num_nodes, -1);
    std::vector<std::vector<int>> cluster_matrix(num_nodes, row);
    // NOTES: this matrix represents a lookup if and to which cluster a node belongs

    // TODO(stlucas): fix the cluster assignment (maybe review the TSP to GTSP transform)

    // create lookup to check for cluster membership
    for (int cluster_index = 0; cluster_index < clusters.size(); cluster_index++) { // per cluster
        std::vector<int>& current_cluster =clusters[cluster_index];
        for(int j = 0; j < current_cluster.size(); j++) { // iterate over each node element
            int node_index = current_cluster[j];
            cluster_lookup_[node_index] = cluster_index; //for node index, assign cluster index

            // mark intracluster edges
            for(int k = j+1; k < current_cluster.size(); k++){
              cluster_matrix[node_index][current_cluster[k]] = cluster_index;
              cluster_matrix[current_cluster[k]][node_index] = cluster_index;
             }
        }
    }

    // creating circular zero costs, back-propagating costs (creating P')
    for(std::vector<int>& current_cluster : clusters) {
        // keep tmp of adjacency_matrix[current_cluster[0]][k] since it gets overwritten
        std::vector<int> tmp(num_nodes);
        for(int k = 0; k < num_nodes; k++){
            tmp[k] = adjacency_matrix[current_cluster.front()][k];
        }

        for(int i = 0; i <  current_cluster.size() - 1; i++) {
            // moving "leaving cost one to the left"
            for(int k = 0; k < num_nodes; k++) {
                if(cluster_matrix[current_cluster[i+1]][k] == -1){ // if its an external edge, put it one back
                    adjacency_matrix[current_cluster[i]][k] = adjacency_matrix[current_cluster[i+1]][k];
                    // TODO: fix the overwriting of the first swap in the list?!
                }
          }
          // assigning the direct circle;
          adjacency_matrix[current_cluster[i]][current_cluster[i + 1]] = 0;
        }
        adjacency_matrix[current_cluster.back()][current_cluster.front()] = 0;

        // do the last shift, front to back for all k (using head used before)
        for(int k = 0; k < num_nodes; k++) {
            if(cluster_matrix[current_cluster.front()][k] == -1){
                adjacency_matrix[current_cluster.back()][k] = tmp[k];
            }
        }
    }

    //assigning M to inter-cluster connections (creating P'')
    for (int i = 0; i < num_nodes; i++) {
        for (int j = 0; j < num_nodes; j++) {
            if (cluster_matrix[i][j] == -1) {
              adjacency_matrix[i][j] += m;
            }
        }
    }

    ROS_INFO("Transform is done");

    // Loading underlying TSP
    return loadTSP(adjacency_matrix);
}

bool OrInterface::loadTSP(std::vector <std::vector<int>> &adjacency_matrix) {
    adjacency_matrix_ = adjacency_matrix;
    // TODO introcude better test!
    return true;
}

bool OrInterface::solve() {
    ROS_ASSERT(!adjacency_matrix_.empty());

    RoutingIndexManager manager(adjacency_matrix_.size(), 1,
                                depot_);
    RoutingModel routing(manager);

    const int transit_callback_index = routing.RegisterTransitCallback(
            [this, &manager](int64_t from_index,
                             int64_t to_index) -> int64_t {
                // Convert from routing variable Index to distance matrix NodeIndex.
                auto from_node = manager.IndexToNode(from_index).value();
                auto to_node = manager.IndexToNode(to_index).value();
                return adjacency_matrix_[from_node][to_node];
            });
    // NOTE: manager has to be passed by reference strictly!
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

    RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
    searchParameters.set_first_solution_strategy(
            FirstSolutionStrategy::PATH_CHEAPEST_ARC);

    const Assignment *solution = routing.SolveWithParameters(search_params_);

    extractSolution(*solution, manager, routing);
    return true;
}

void OrInterface::extractSolution(const Assignment &solution,
                                  RoutingIndexManager manager,
                                  RoutingModel &routing) {
    path_nodes_.clear();

    ROS_INFO_STREAM("Objective: " << solution.ObjectiveValue());

    // get start for vehicle 0
    int64_t distance{0};
    int64_t index = routing.Start(0);
    std::stringstream route;

    // Extract full route
    while (routing.IsEnd(index) == false) {
        route << manager.IndexToNode(index).value() << " -> ";
        path_nodes_.push_back(index);
        int64_t previous_index = index;
        index = solution.Value(routing.NextVar(index));

        distance += routing.GetArcCostForVehicle(previous_index, index, int64_t{0});
    }
    ROS_INFO_STREAM(route.str() << manager.IndexToNode(index).value());
    ROS_INFO_STREAM("Distance of the route: " << distance << "m \n");
    ROS_INFO_STREAM("Problem solved in " << routing.solver()->wall_time() << "ms");
}

std::vector<int> OrInterface::getTSPSolution() {
    return path_nodes_;
}

std::vector<int> OrInterface::getGTSPSolution() {
    // TODO(stlucas): filter in-cluster tours for the GTSP

    // detect first element of a cluster and drop the rest!
    std::vector<int>::iterator iter = path_nodes_.begin();

    for(iter; iter < path_nodes_.end(); iter++){
        if(cluster_lookup_.find(*iter) != cluster_lookup_.end()){

            int cluster_index = cluster_lookup_[*iter];
            int cluster_size = clusters_[cluster_index].size();
            path_nodes_.erase(iter + 1, iter + cluster_size);
        }
    }

    return path_nodes_;
}

}  // namespace or_interface_catkin
