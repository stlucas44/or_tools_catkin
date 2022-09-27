#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "or_tools_catkin/or_interface.h"

namespace or_tools_catkin {


std::vector<std::vector<int>> simpleTSP(int size = 10){
  //create adjacency matrix
  std::vector<std::vector<int>> adjacency(size);
  for(int i = 0; i < size; i++){
    std::vector<int> row(size, 0);
    for(int j = 0; j < size; j++){
      if(i != j){
        row[j] = rand() % 100 + 1;
      }
    }
    adjacency[i] = row;
  }

  return adjacency;
}

std::vector<std::vector<int>> simpleCluster(std::vector<std::vector<int>> adjacency){
    int num_nodes = adjacency.size();
    int num_clusters = (int)(0.25 * num_nodes) + 1; // max n * 0.25 clusters
    int nodes_per_cluster = num_nodes / num_clusters;
    ROS_INFO("Creating cluster: %i nodes, %i clusters,  %i nodes per cluster",
             num_nodes, num_clusters, nodes_per_cluster);

    std::vector<std::vector<int>>  cluster_set(num_clusters);
    for(int i = 0; i < num_nodes; i++){
        cluster_set[rand() % num_clusters].push_back(i);
    }
    return cluster_set;
}


void printVector(std::vector<int> &data){
    std::stringstream ss;
    std::copy(data.begin(), data.end(), std::ostream_iterator<double>(ss, " "));
    ss << std::endl;
    ROS_INFO_STREAM(ss.str());
}
void printVectorVector(std::vector<std::vector<int>> &data){
    for(auto vector : data){
        printVector(vector);
    }
}


TEST(OrInterfaceTest, manyGTSP){
    int num_runs = 1000;
    for(int i = 0; i < num_runs; i++){
      auto adjacency = simpleTSP();

      auto clusters = simpleCluster(adjacency);
      ROS_INFO("Cluster:");
      printVectorVector(clusters);

      OrInterface solver(adjacency.size());
      EXPECT_TRUE(solver.loadGTSP(adjacency, clusters));
      //ROS_INFO("Manipulated adjacency");
      //printVectorVector(solver.adjacency_matrix_);

      EXPECT_TRUE(solver.solve());
      //auto result = solver.getTSPSolution();
      //printVector(result);
      auto result_pruned = solver.getGTSPSolution();
      ROS_INFO("Solution # %i:", i);
      printVector(result_pruned);

    }

}

TEST(OrInterfaceTest, simpleGTSP){
    auto adjacency = simpleTSP();
    //ROS_INFO("adjacency: ");
    //printVectorVector(adjacency);

    auto clusters = simpleCluster(adjacency);
    ROS_INFO("Cluster:");
    printVectorVector(clusters);

    OrInterface solver(adjacency.size());
    EXPECT_TRUE(solver.loadGTSP(adjacency, clusters));
    //ROS_INFO("Manipulated adjacency");
    //printVectorVector(solver.adjacency_matrix_);

    EXPECT_TRUE(solver.solve());
    auto result = solver.getTSPSolution();
    printVector(result);
    auto result_pruned = solver.getGTSPSolution();
    printVector(result_pruned);

}
/*
TEST(OrInterfaceTest, TSP){
  auto adjacency = simpleTSP();

  OrInterface solver(adjacency.size());
  EXPECT_TRUE(solver.loadTSP(adjacency));
  EXPECT_TRUE(solver.solve());
  EXPECT_NO_THROW(solver.getTSPSolution());

}
 */
} // namespace or_interface_catkin


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
