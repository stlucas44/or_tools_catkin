#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "or_tools_catkin/or_interface.h"

namespace or_tools_catkin {


std::vector<std::vector<int>> simpleTSP(){
  //create adjancy matrix
  int size = 10;
  std::vector<std::vector<int>> adjancy(size);
  for(int i = 0; i < size; i++){
    std::vector<int> row(size, 0);
    for(int j = 0; j < size; j++){
      if(i != j){
        row[j] = rand() % 100 + 1;
      }
    }
    adjancy[i] = row;
  }

  return adjancy;
}

std::vector<std::vector<int>> simpleCluster(std::vector<std::vector<int>> adjancy){
    int num_nodes = adjancy.size();
    int num_clusters = rand() % (int)(0.25 * num_nodes) + 1;
    int nodes_per_cluster = num_nodes % num_clusters;

    std::vector<std::vector<int>>  cluster_set;
    for(int i = 0; i < num_clusters; i++){
        std::vector<std::vector<int>> cluster;
        for(int j = 0; j<nodes_per_cluster; j++){
            // TODO(stlucas): Create random cluster!
        }
    }
}


TEST(OrInterfaceTest, GTSP){
    EXPECT_TRUE(true);
}

TEST(OrInterfaceTest, TSP){
  auto adjancy = simpleTSP();

  OrInterface solver(adjancy.size());
  EXPECT_TRUE(solver.loadTSP(adjancy));
  EXPECT_TRUE(solver.solve());
  EXPECT_NO_THROW(solver.getTSPSolution());

}
} // namespace or_interface_catkin


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
