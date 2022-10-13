#ifndef OR_TOOLS_CATKIN_OR_INTERFACE_H
#define OR_TOOLS_CATKIN_OR_INTERFACE_H

#include <vector>
#include <map>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

#include "or_tools_catkin/tsp_transform.h"

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

        std::vector<std::vector<int>> adjacency_matrix_;

    private:
        void extractSolution(const Assignment &solution,
                             const RoutingIndexManager &manager,
                             const RoutingModel &routing);

        TSPTransform transformer_;

        RoutingSearchParameters search_params_;
        const int num_nodes_;
        const int num_vehicles_;
        const RoutingIndexManager::NodeIndex depot_;
        int cost_;

        std::vector<std::vector<int>> clusters_;

        std::vector<int> path_nodes_;
    };

    void printVector(std::vector<int> &data);
    void printVectorVector(std::vector<std::vector<int>> &data);
} // namespace or_interface_catkin

#endif // OR_TOOLS_CATKIN_OR_INTERFACE_H
