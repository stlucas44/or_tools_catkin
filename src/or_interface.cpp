

#include <cmath>
#include <cstdint>
#include <vector>

#include <ros/package.h>
#include <ros/ros.h>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

#include "or_tools_catkin/or_interface.h"

namespace or_tools_catkin {
using namespace operations_research;

OrInterface::OrInterface(int num_nodes) :
  num_vehicles_(1),
  manager_(num_nodes, num_vehicles_, RoutingIndexManager::NodeIndex{num_nodes-2}),
  routing_(manager_) {
  num_vehicles_ = 1;

  search_params_= DefaultRoutingSearchParameters();

}

bool OrInterface::loadGTSP(std::vector<std::vector<int>> &adjancy_matrix,
    std::vector<std::vector<int>> &clusters){

  // Noon and bean transform:
  // https://www.researchgate.net/publication/265366022_An_Efficient_Transformation_Of_The_Generalized_Traveling_Salesman_Problem

  int m = INT_MAX / 4; // / Precision;
  // TODO(stlucas): Improve choosing m


  int size = adjancy_matrix.size();
  // TODO(stlucas): manipulate adjancy_matrix to create TSP
  for (int i = 0; i < size; i++){
    for(int j = 0; j < size; j++){
      if(i == j){
        continue;
      }
      if(false) {// check if element is in cluster
        adjancy_matrix[i][j] += m;
      }

    }
  }
  // Loading underlying TSP
  return loadTSP(adjancy_matrix);
}

bool OrInterface::loadTSP(std::vector<std::vector<int>> &adjancy_matrix){
  adjancy_matrix_ = adjancy_matrix;

  return setup();
}

bool OrInterface::setup(){
    ROS_ASSERT(!adjancy_matrix_.empty());

    //TODO(stlucas): Fix routing callback -> might be thrown away before solver starts
    const auto local_adjancy_matrix = adjancy_matrix_;

    const int transit_callback_index = routing_.RegisterTransitCallback(
        [this](int64_t from_index,
                                     int64_t to_index) -> int64_t {
          // Convert from routing variable Index to distance matrix NodeIndex.
          auto from_node = manager_.IndexToNode(from_index).value();
          auto to_node = manager_.IndexToNode(to_index).value();
          return adjancy_matrix_[from_node][to_node];
        });


    routing_.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);


    return true;
}

bool OrInterface::solve() {
  const Assignment* solution = routing_.SolveWithParameters(search_params_);
  // TODO(stlucas): check if solver was successful

  extractSolution(*solution);
}

void OrInterface::extractSolution(const Assignment& solution) {
  path_nodes_.clear();

  ROS_INFO_STREAM("Objective: " << solution.ObjectiveValue());

  // get start for vehicle 0
  int64_t distance{0};
  int64_t index = routing_.Start(0);
  std::stringstream route;

  // Extract full route
  while (routing_.IsEnd(index) == false) {
    route << manager_.IndexToNode(index).value() << " -> ";
    path_nodes_.push_back(index);
    int64_t previous_index = index;
    index = solution.Value(routing_.NextVar(index));

    distance += routing_.GetArcCostForVehicle(previous_index, index, int64_t{0});
  }
  ROS_INFO_STREAM(route.str() << manager_.IndexToNode(index).value());
  ROS_INFO_STREAM("Distance of the route: " << distance << "m \n");
  ROS_INFO_STREAM( "Advanced usage: \n" <<
                   "Problem solved in " << routing_.solver()->wall_time() << "ms");

  // TODO(stlucas): filter in-cluster tours for the GTSP?
}

std::vector<int> OrInterface::getSolution(){
  return path_nodes_;
}

}  // namespace or_interface_catkin
