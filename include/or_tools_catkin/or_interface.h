#ifndef OR_TOOLS_CATKIN_OR_INTERFACE_H
#define OR_TOOLS_CATKIN_OR_INTERFACE_H

#include <vector>
#include <map>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace or_tools_catkin {
    using namespace operations_research;

    class OrInterface {
    public:
        OrInterface(int num_nodes);

        bool loadGTSP(std::vector<std::vector<int>> &adjacency_matrix,
                      std::vector<std::vector<int>> &clusters);

        bool loadTSP(std::vector<std::vector<int>> &adjacency_matrix);

        bool solve();

        std::vector<int> getTSPSolution();

        std::vector<int> getGTSPSolution();

    private:
        void extractSolution(const Assignment &solution,
                             RoutingIndexManager manager,
                             RoutingModel &routing);

        RoutingSearchParameters search_params_;
        const int num_vehicles_;
        const RoutingIndexManager::NodeIndex depot_;

        std::vector<std::vector<int>> adjacency_matrix_;
        std::vector<std::vector<int>> clusters_;
        std::map<int, int> cluster_lookup_; // key: node_index, value: cluster

        std::vector<int> path_nodes_;
    };
} // namespace or_interface_catkin

#endif // OR_TOOLS_CATKIN_OR_INTERFACE_H
