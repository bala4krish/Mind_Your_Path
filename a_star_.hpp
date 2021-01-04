//
//  a_star_.hpp
//  A_Star_
//
//  Created by Balaji Krishnamurthy on 1/3/21.
//  Copyright © 2021 Balaji Krishnamurthy. All rights reserved.
//

#ifndef a_star__hpp
#define a_star__hpp

#include <iostream>
#include <sstream>
#include <cmath>
#include <vector>
#include <queue>
#include <unordered_set>

// A class for each node with LinkedList like structure having
// node position and pointer to the parent node
// Example - {2,2} visited from {1,1} will have
//           current_    = {2,2}
//           prev_       = {1,1}
//           total_cost_ = to_go_cost_ + heuristic_cost_
struct Node
{
      const std::pair<int, int> current_;
      std::shared_ptr<Node> prev_;
      double total_cost_;
      double to_go_cost_;
      double heuristic_cost_;
      Node(std::pair<int, int> point, std::shared_ptr<Node> def = nullptr)
            : current_(point), prev_(def), total_cost_(0.0), to_go_cost_(0.0), heuristic_cost_(0.0) {}
      friend bool operator == (const std::shared_ptr<Node>& N1, const std::shared_ptr<Node>& N2);
      ~Node() {}
};

// type def to use for the priority queue for having sorted storage
typedef std::tuple<double, std::shared_ptr<Node>> MyTup;

// Main class taking variables required for the grid construction,
// Starting and target node of the path to be found
// ComputePath() performs the major part of the alogirthm
// BackTrack() is called find the optimal path from the goal to the start when the goal node is traversed.
// feasible_points_ store the path found.
// Other members and symbols are self-explanatory
class Finder
{
public:
      Finder(std::pair<int, int> start_point, std::pair<int, int> goal_point, int r, int c, int obs)
            : start_node_(start_point), goal_node_(goal_point), row_(r), col_(c), obs_(obs) {}
      auto get_row_col() const;
      auto get_grid() const;
      void GenerateGrid();
      void GenerateObstacles();
      void BackTrack(std::shared_ptr<Node>& );
      void ComputePath();
      void PrintResult();
      ~Finder() {}
      
private:
      const int row_;
      const int col_;
      const int obs_;
      const std::pair<int, int> start_node_;
      const std::pair<int, int> goal_node_;
      std::vector<std::vector<int>> grid_;
      std::priority_queue<MyTup, std::vector<MyTup>, std::greater<MyTup>> open_list_;
      std::unordered_set<std::shared_ptr<Node>> scanned_nodes_;
      std::unordered_set<std::shared_ptr<Node>> closed_list_;
      std::vector<std::pair<int, int>> feasible_points_;
};

#endif /* a_star__hpp */
