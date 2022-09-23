#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "or_tools_catkin/or_interface.h"

namespace or_tools_catkin {


struct DataModel {
  const std::vector<std::vector<int>> locations{
      {4, 4}, {2, 0}, {8, 0}, {0, 1}, {1, 1}, {5, 2}, {7, 2}, {3, 3}, {6, 3},
      {5, 5}, {8, 5}, {1, 6}, {2, 6}, {3, 7}, {6, 7}, {0, 8}, {7, 8},
  };
  const int num_vehicles = 1;
  const RoutingIndexManager::NodeIndex depot{0};
  DataModel() {
    // Convert locations in meters using a city block dimension of 114m x 80m.
    for (auto& it : const_cast<std::vector<std::vector<int>>&>(locations)) {
      it[0] *= 114;
      it[1] *= 80;
    }
  }
};
// [END data_model]

// [START manhattan_distance_matrix]
/*! @brief Generate Manhattan distance matrix.
 * @details It uses the data.locations to computes the Manhattan distance
 * between the two positions of two different indices.*/
std::vector<std::vector<int>> GenerateManhattanDistanceMatrix(
    const std::vector<std::vector<int>>& locations) {
  std::vector<std::vector<int>> distances =
      std::vector<std::vector<int>>(
          locations.size(), std::vector<int>(locations.size(), int{0}));
  for (int fromNode = 0; fromNode < locations.size(); fromNode++) {
    for (int toNode = 0; toNode < locations.size(); toNode++) {
      if (fromNode != toNode)
        distances[fromNode][toNode] =
            int{std::abs(locations[toNode][0] - locations[fromNode][0]) +
                    std::abs(locations[toNode][1] - locations[fromNode][1])};
    }
  }
  return distances;

}



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


TEST(OrInterfaceTest, Access){
  auto adjancy = simpleTSP();

  //DataModel data;
  //auto adjancy = GenerateManhattanDistanceMatrix(data.locations);

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
