
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

bool OrInterface::loadGTSP(std::vector <std::vector<int>> &adjacency_matrix,
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

    adjacency_matrix_ = transformer_.transformAdjacencyGtspToTsp(adjacency_matrix, clusters);
    return true;
}

bool OrInterface::loadTSP(std::vector <std::vector<int>> &adjacency_matrix) {
    adjacency_matrix_ = adjacency_matrix;
    return true;
}

bool OrInterface::solve() {
    ROS_ASSERT(!adjacency_matrix_.empty());
    ROS_INFO("Solving TSP with %i nodes", (int) adjacency_matrix_.size());

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

    std::vector<int> gtsp_result = transformer_.transformSolutionTsptoGTSP(path_nodes_);
    //ROS_INFO_STREAM("Corrected distance of the route: " << cost_ - (path_nodes_.size()) * m_ << "m \n");
    ROS_INFO("Result:");
    printVector(gtsp_result);

    return gtsp_result;
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
