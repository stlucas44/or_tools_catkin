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

std::vector<std::vector<int>> simpleCluster(std::vector<std::vector<int>> adjacency, int num_clusters = -1){
    int num_nodes = adjacency.size();
    if (num_clusters == -1){
        num_clusters = (int)(0.25 * num_nodes) + 1; // max n * 0.25 clusters
    }

    //ROS_INFO("Creating cluster: %i nodes, %i clusters", num_nodes, num_clusters);

    std::vector<std::vector<int>>  cluster_set(num_clusters);
    for(int i = 0; i < num_nodes; i++){
        cluster_set[rand() % num_clusters].push_back(i);
    }
    for(int i = 0; i < cluster_set.size(); i++){
        if (cluster_set[i].size() == 0){
            cluster_set.erase(cluster_set.begin() + i);
        }
    }

    return cluster_set;
}


void printVector(std::vector<int> &data){
    std::stringstream ss;
    std::copy(data.begin(), data.end(), std::ostream_iterator<double>(ss, " "));
    ROS_INFO_STREAM(ss.str());
}
void printVectorVector(std::vector<std::vector<int>> &data){
    for(auto vector : data){
        printVector(vector);
    }
}

TEST(OrInterfaceTest, largeGTSP){
    auto adjacency = simpleTSP(400);
    auto clusters = simpleCluster(adjacency, 50);

    OrInterface solver(adjacency.size());
    EXPECT_TRUE(solver.loadGTSP(adjacency, clusters));
    EXPECT_TRUE(solver.solve());

    auto result_pruned = solver.getGTSPSolution();
    //printVector(result_pruned);
}

TEST(OrInterfaceTest, polygonSample){
      // prepare adjacency
      std::vector<std::vector<int>> adjacency(10,std::vector<int>(10,INT_MAX));
      std::vector<int> column9 = {94,97,64,86,77,96,108,106};
      std::vector<int> row8 = {28,25,28,6,25,6,22,25, INT_MAX, INT_MAX};

      for(int i = 0; i < 8; i++){
        adjacency[i][9] = column9[i];
      }
      adjacency[8] = row8;
      // prepare cluster
      std::vector<std::vector<int>> clusters = {{0,1,2,3,4,5,6,7}, {8}, {9}};

      OrInterface solver(adjacency.size());
      EXPECT_TRUE(solver.loadGTSP(adjacency, clusters));
      ROS_INFO("Manipulated adjacency");
      printVectorVector(solver.adjacency_matrix_);

      EXPECT_TRUE(solver.solve());
      auto result_pruned = solver.getGTSPSolution();
      printVector(result_pruned);
  }

TEST(OrInterfaceTest, manyGTSP){
    int num_runs = 1000;
    for(int i = 0; i < num_runs; i++){
      auto adjacency = simpleTSP();

      auto clusters = simpleCluster(adjacency);
      ROS_INFO("Cluster:");

      OrInterface solver(adjacency.size());
      EXPECT_TRUE(solver.loadGTSP(adjacency, clusters));
      //ROS_INFO("Manipulated adjacency");
      //printVectorVector(solver.adjacency_matrix_);

      EXPECT_TRUE(solver.solve());
      auto result_pruned = solver.getGTSPSolution();
      ROS_INFO("Solution # %i:", i);
      printVector(result_pruned);
    }

}

TEST(OrInterfaceTest, simpleGTSP){
    auto adjacency = simpleTSP();
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

TEST(OrInterfaceTest, TSP){
  auto adjacency = simpleTSP();

  OrInterface solver(adjacency.size());
  EXPECT_TRUE(solver.loadTSP(adjacency));
  EXPECT_TRUE(solver.solve());
  EXPECT_NO_THROW(solver.getTSPSolution());

}

} // namespace or_interface_catkin


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
