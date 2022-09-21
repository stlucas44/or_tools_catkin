#ifndef OR_TOOLS_CATKIN_OR_INTERFACE_H
#define OR_TOOLS_CATKIN_OR_INTERFACE_H

namespace or_tools_catkin {
using namespace operations_research;

class OrInterface {
  OrInterface(int num_nodes);
  bool loadGTSP(std::vector<std::vector<int>> &adjancy_matrix, std::vector<std::vector<int>> &clusters);
  bool loadTSP(std::vector<std::vector<int>> &adjancy_matrix);
  bool solve();
  std::vector<int> getSolution();
  //void getStats();

private:
  bool setup();
  void extractSolution(const Assignment& solution);

  RoutingIndexManager manager_;
  RoutingModel routing_;

  RoutingSearchParameters search_params_;
  int num_vehicles_;

  std::vector<std::vector<int>> adjancy_matrix_;

  //const int transit_callback_index_; // const might get issues


  std::vector<int> path_nodes_;


};
} // namespace or_interface_catkin

#endif // OR_TOOLS_CATKIN_OR_INTERFACE_H
