
#include <cstdint>
#include <ros/package.h>
#include <ros/ros.h>

#include "or_tools_catkin/or_interface.h"

namespace or_tools_catkin {
using namespace operations_research;

OrInterface::OrInterface(int num_nodes) :
  num_vehicles_(1),
  depot_{num_nodes-2} // TODO(stlucas): change to num_nodes -2 at some point!
   {
  ROS_ASSERT(num_nodes > 1);

  search_params_= DefaultRoutingSearchParameters();
  search_params_.set_first_solution_strategy(
      FirstSolutionStrategy::PATH_CHEAPEST_ARC); //May adapt this!
}

bool OrInterface::loadGTSP(std::vector<std::vector<int>> &adjancy_matrix,
    std::vector<std::vector<int>> &clusters){

  clusters_ = clusters;
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
  // TODO introcude better test!
  return true;
}

bool OrInterface::solve() {
  ROS_ASSERT(!adjancy_matrix_.empty());

  RoutingIndexManager manager(adjancy_matrix_.size(), 1,
                              depot_);
  RoutingModel routing(manager);

  const int transit_callback_index = routing.RegisterTransitCallback(
      [this, &manager](int64_t from_index,
                                   int64_t to_index) -> int64_t {
        // Convert from routing variable Index to distance matrix NodeIndex.
        auto from_node = manager.IndexToNode(from_index).value();
        auto to_node = manager.IndexToNode(to_index).value();
        return adjancy_matrix_[from_node][to_node];
      });
  // NOTE: manager has to be passed by reference strictly!

  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  searchParameters.set_first_solution_strategy(
      FirstSolutionStrategy::PATH_CHEAPEST_ARC);

  const Assignment* solution = routing.SolveWithParameters(search_params_);

  extractSolution(*solution, manager, routing);
  return true;
}

void OrInterface::extractSolution(const Assignment& solution,
      RoutingIndexManager manager,
      RoutingModel& routing) {
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
  ROS_INFO_STREAM( "Advanced usage: \n" <<
                   "Problem solved in " << routing.solver()->wall_time() << "ms");
}

std::vector<int> OrInterface::getTSPSolution(){
  return path_nodes_;
}

std::vector<int> OrInterface::getGTSPsolution(){
  // TODO(stlucas): filter in-cluster tours for the GTSP?
  return path_nodes_;
}

}  // namespace or_interface_catkin
