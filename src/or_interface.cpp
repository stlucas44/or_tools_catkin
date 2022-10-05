
#include <cstdint>
#include <ros/package.h>
#include <ros/ros.h>

#include "or_tools_catkin/or_interface.h"

namespace or_tools_catkin {
using namespace operations_research;

OrInterface::OrInterface(int num_nodes) :
        num_nodes_(num_nodes),
        num_vehicles_(1),
        depot_{num_nodes - 2} // always starting at the second last element
{
    ROS_ASSERT(num_nodes_ > 1);

    search_params_ = DefaultRoutingSearchParameters();
    //search_params_.set_first_solution_strategy(
    //        FirstSolutionStrategy::PATH_CHEAPEST_ARC); //May adapt this!

    //search_params_.set_local_search_metaheuristic(
    //    LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
    //search_params_.set_log_search(true);


    search_params_.mutable_time_limit()->set_seconds(100);
    // TODO: check performance against guided local search
    // resource: https://github.com/google/or-tools/blob/stable/ortools/constraint_solver/routing.h

}

bool OrInterface::loadGTSP(std::vector <std::vector<int>> adjacency_matrix,
                           std::vector <std::vector<int>> &clusters) {
    ROS_ASSERT(adjacency_matrix.size() == num_nodes_);
    if(clusters.size() == 0){
        ROS_ERROR(" No clusters specified");
        return false;
    }

    //printVectorVector(adjacency_matrix);
    ROS_INFO("Clusters: ");
    printVectorVector(clusters);

    for(std::vector<int>& cluster : clusters){
        if(cluster.size() == 0){
            ROS_ERROR("empty cluster!");
            return false;
        }
    }

    clusters_ = clusters;

    // Noon and bean transform: (currently implemented)
    // https://www.researchgate.net/publication/265366022_An_Efficient_Transformation_Of_The_Generalized_Traveling_Salesman_Problem

    // Improved (maybe simpler): Generalized network design problems
    // https://www.degruyter.com/document/doi/10.1515/9783110267686.60/html?lang=de

    //int m = INT_MAX / 4; // / Precision;

    int total_sum = 0;
    for(std::vector<int>& row : adjacency_matrix) {
        for(int& element: row){
            total_sum += element;
        }
    }
    m_ = total_sum;

    // cluster membership lookup
    std::vector<int> row(num_nodes_, -1);
    std::vector<std::vector<int>> cluster_matrix(num_nodes_, row);

    // fill lookup
    for (int cluster_index = 0; cluster_index < clusters_.size(); cluster_index++) { // per cluster
        std::vector<int>& current_cluster =clusters_[cluster_index];
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

    // step 1
    // creating circular zero costs, back-propagating costs (creating P')
    for(std::vector<int>& current_cluster : clusters_) {
        // copy all costs of the cluster start to keep t
        std::vector<int> tmp(num_nodes_);
        for(int k = 0; k < num_nodes_; k++){
            tmp[k] = adjacency_matrix[current_cluster.front()][k];
        }

        for(int i = 1; i <  current_cluster.size(); i++) {
            // moving "leaving cost one to the left"
            for(int k = 0; k < num_nodes_; k++) {
                if(cluster_matrix[current_cluster[i]][k] == -1){ // if its an external edge, put it one back
                    adjacency_matrix[current_cluster[i-1]][k] = adjacency_matrix[current_cluster[i]][k];
                    // TODO: What about the zero cost circular path?
                }
          }
          // assigning the direct circle;
          adjacency_matrix[current_cluster[i-1]][current_cluster[i]] = 0;
        }
        adjacency_matrix[current_cluster.back()][current_cluster.front()] = 0;
        // do the last shift, front to back for all k (using head used before)
        for(int k = 0; k < num_nodes_; k++) {
            if(cluster_matrix[current_cluster.front()][k] == -1){
                adjacency_matrix[current_cluster.back()][k] = tmp[k];
            }
        }
    }
    // step 2
    //assigning M to inter-cluster connections (creating P'')
    for (int i = 0; i < num_nodes_; i++) {
        for (int j = 0; j < num_nodes_; j++) {
            if (cluster_matrix[i][j] == -1 &&
                adjacency_matrix[i][j] < INT_MAX) {
              adjacency_matrix[i][j] += m_;
            }
        }
    }

    // Loading underlying TSP
    return loadTSP(adjacency_matrix);
}

bool OrInterface::loadTSP(std::vector <std::vector<int>> &adjacency_matrix) {
    adjacency_matrix_ = adjacency_matrix;
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

    const Assignment *solution = routing.SolveWithParameters(search_params_);

    if(routing.status() == 1 &&
        solution != nullptr){
      extractSolution(*solution, manager, routing);
      return true;
    }
    else if(solution == nullptr){
      ROS_ERROR("solution == nullptr");
      ROS_ERROR("OR status: %i", routing.status());
      return false;
    }

    else if(!solution->Empty()){
      ROS_ERROR("solution is empty");
      return false;
    }
    else{
      ROS_ERROR("solution failed with unkown reason tsp failed");
      return false;
    }
}

void OrInterface::extractSolution(const Assignment &solution,
                                  const RoutingIndexManager &manager,
                                  const RoutingModel &routing) {
    path_nodes_.clear();

    ROS_INFO_STREAM("Objective: " << solution.ObjectiveValue());

    // get start for vehicle 0
    cost_ = 0;
    int64_t index = routing.Start(0);
    std::stringstream route;

    // Extract full route
    while (routing.IsEnd(index) == false) {
        route << manager.IndexToNode(index).value() << " -> ";
        path_nodes_.push_back(index);
        int64_t previous_index = index;
        index = solution.Value(routing.NextVar(index));

        cost_ += routing.GetArcCostForVehicle(previous_index, index, int64_t{0});
    }
    ROS_INFO_STREAM(route.str() << manager.IndexToNode(index).value());
    ROS_INFO_STREAM("Distance of the route: " << cost_ << "m");
    ROS_INFO_STREAM("Problem solved in " << routing.solver()->wall_time() << "ms");
}

std::vector<int> OrInterface::getTSPSolution() {
    return path_nodes_;
}

std::vector<int> OrInterface::getGTSPSolution() {
    ROS_ASSERT(!path_nodes_.empty());

    int prev_cluster = -1;
    std::vector<int> path_nodes_filtered;

    // only keep first element of each cluster
    for(int i = 0; i < path_nodes_.size(); i++){
        int current_node = path_nodes_[i];
        int current_cluster = -1;
        if(cluster_lookup_.find(current_node) != cluster_lookup_.end()) { // locate element in cluster
          current_cluster = cluster_lookup_[current_node];
        }
        if (current_cluster == -1 ||
              current_cluster != prev_cluster){
            path_nodes_filtered.push_back(current_node);
        }
        prev_cluster = current_cluster;
    }

    //ROS_INFO_STREAM("Corrected distance of the route: " << cost_ - (path_nodes_.size()) * m_ << "m \n");
    ROS_INFO("Result:");
    printVector(path_nodes_filtered);

    return path_nodes_filtered;
}

void printVector(std::vector<int> &data){
    std::stringstream ss;
    std::copy(data.begin(), data.end(), std::ostream_iterator<double>(ss, " "));
    ROS_INFO_STREAM("  " << ss.str());
}
void printVectorVector(std::vector<std::vector<int>> &data){
    for(auto vector : data){
        printVector(vector);
    }
}


}  // namespace or_interface_catkin
