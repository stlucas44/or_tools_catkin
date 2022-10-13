#include <vector>

namespace or_tools_catkin {

    class TSPTransform {
    public:
      TSPTransform(){}

      std::vector<std::vector<int>> transformAdjacencyGtspToTsp(std::vector<std::vector<int>> adjacency_matrix,
                                      std::vector<std::vector<int>>& clusters) {
        // Noon and bean transform: (currently implemented)
        // https://www.researchgate.net/publication/265366022_An_Efficient_Transformation_Of_The_Generalized_Traveling_Salesman_Problem

        // Improved (maybe simpler): Generalized network design problems
        // https://www.degruyter.com/document/doi/10.1515/9783110267686.60/html?lang=de

        int total_sum = 0;
        for(std::vector<int>& row : adjacency_matrix) {
            for(int& element: row){
                total_sum += element;
            }
        }
        m_ = total_sum;
        num_nodes_ = adjacency_matrix.size();

        // cluster membership lookup
        std::vector<int> row(num_nodes_, -1);
        cluster_matrix_ = std::vector<std::vector<int>>(num_nodes_, row);

        // fill lookup
        for (int cluster_index = 0; cluster_index < clusters_.size(); cluster_index++) { // per cluster
            std::vector<int>& current_cluster =clusters_[cluster_index];
            for(int j = 0; j < current_cluster.size(); j++) { // iterate over each node element
                int node_index = current_cluster[j];
                cluster_lookup_[node_index] = cluster_index; //for node index, assign cluster index

                // mark intracluster edges
                for(int k = j+1; k < current_cluster.size(); k++){
                  cluster_matrix_[node_index][current_cluster[k]] = cluster_index;
                  cluster_matrix_[current_cluster[k]][node_index] = cluster_index;
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
                    if(cluster_matrix_[current_cluster[i]][k] == -1){ // if its an external edge, put it one back
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
                if(cluster_matrix_[current_cluster.front()][k] == -1){
                    adjacency_matrix[current_cluster.back()][k] = tmp[k];
                }
            }
        }
        // step 2
        //assigning M to inter-cluster connections (creating P'')
        for (int i = 0; i < num_nodes_; i++) {
            for (int j = 0; j < num_nodes_; j++) {
                if (cluster_matrix_[i][j] == -1 &&
                    adjacency_matrix[i][j] < INT_MAX) {
                  adjacency_matrix[i][j] += m_;
                }
            }
        }
        return adjacency_matrix;
      }

      std::vector<int> transformSolutionTsptoGTSP(std::vector<int> tsp_result) {
        int prev_cluster = -1;
        gtsp_result_ = std::vector<int>();

        // only keep first element of each cluster
        for(int i = 0; i < tsp_result.size(); i++){
            int current_node = tsp_result[i];
            int current_cluster = -1;
            if(cluster_lookup_.find(current_node) != cluster_lookup_.end()) { // locate element in cluster
              current_cluster = cluster_lookup_[current_node];
            }
            if (current_cluster == -1 || // push element if its new
                  current_cluster != prev_cluster){
                gtsp_result_.push_back(current_node);
            }
            prev_cluster = current_cluster;
        }
        return gtsp_result_;
      }



    private:
        std::vector<std::vector<int>> adjacency_matrix_;
        std::vector<std::vector<int>> clusters_;


        std::map<int, int> cluster_lookup_; // key: node_index, value: cluster
        std::vector<std::vector<int>> cluster_matrix_;
        int m_;
        int num_nodes_;

        std::vector<int> tsp_result_;
        std::vector<int> gtsp_result_;
    };
} // namespace or_tools_catkin
