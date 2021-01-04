//
//  a_star_.cpp
//  A_Star_
//
//  Created by Balaji Krishnamurthy on 1/3/21.
//  Copyright © 2021 Balaji Krishnamurthy. All rights reserved.
//

//Initialize both open and closed list
//    let the openList equal empty list of nodes
//    let the closedList equal empty list of nodes

//Add the start node
//    put the startNode on the openList (leave it's f at zero)

//Loop until you find the end
//while the openList is not empty
//    // Get the current node
//          let the currentNode equal the node with the least f value
//          remove the currentNode from the openList
//          add the currentNode to the closedList
//    // Found the goal
//          if currentNode is the goal
//                Congratz! You've found the end! Backtrack to get path
//    // Generate children
//          let the children of the currentNode equal the adjacent nodes
//
//    for each child in the children
//        // Child is on the closedList
//          if child is in the closedList
//                continue to beginning of for loop
//        // Create the f, g, and h values
//          child.g = currentNode.g + distance between child and current
//          child.h = distance from child to end
//          child.f = child.g + child.h
//        // Child is already in openList
//          if child.position is in the openList's nodes positions
//                if the child.g is higher than the openList node's g
//                      continue to beginning of for loop
//        // Add the child to the openList
//          add the child to the openList

#include "a_star_.hpp"

void Finder::GenerateGrid()
{
      this->grid_.resize(this->row_, std::vector<int> (this->col_, 0));
}

void Finder::GenerateObstacles()
{
      int n_obs = this->obs_;
      std::pair<int, int> rand_point = {0,0};
      std::srand((u_int)std::time(NULL));
      while(n_obs)
      {
            rand_point.first = std::rand() % this->row_;
            rand_point.second = std::rand() % this->col_;
            if(rand_point == this->start_node_ || rand_point == this->goal_node_ || this->grid_[rand_point.first][rand_point.second] == 1)
            {
                  continue;
            }
            this->grid_[rand_point.first][rand_point.second] = 1;
            n_obs--;
      }
}

bool operator == (const std::shared_ptr<Node>& N1, const std::shared_ptr<Node>& N2)
{
      return N1->current_ == N2->current_;
}

auto Finder::get_row_col() const
{
      return std::tuple<const int, const int> (row_,col_);
}

auto Finder::get_grid() const
{
      return grid_;
}

void Finder::BackTrack(std::shared_ptr<Node>& node)
{
      while(node != nullptr)
      {
            feasible_points_.emplace_back(node->current_);
            node = node->prev_;
      }
}

void Finder::PrintResult() {
      std::cout << "*********************************************************** Path Planning Output ***********************************************************" << std::endl;
      std::cout << "\n\n";
      std::cout << "Starting node : (" << this->start_node_.first << "," << this->start_node_.second << ")\n\n";
      std::cout << "Goal node : (" << this->goal_node_.first << "," << this->goal_node_.second << ")\n\n";
      std::cout << "Number of random obstacles : " << this->obs_ << "\n\n";
      std::cout << "The grid with obstacles is : \n\n";
      std::cout << "\t\t\t\t {";
      auto H = this->grid_.size();
      for(int i = 0; i < H; i++)
      {
            auto W = this->grid_[i].size();
            std::cout << "{";
            for(int j = 0; j < W; j++)
            {
                  std::cout << this->grid_[i][j];
                  if(j == W - 1)
                  {
                        continue;
                  }
                  std::cout << ",";
            }
            std::cout << "}";
            if(i == H - 1)
            {
                  continue;
            }
            std::cout << ",";
            std::cout << "\n\t\t\t\t  ";
      }
      std::cout << "}\n\n\n\n";
      
      if(this->feasible_points_.empty())
      {
            std::cout << "No path found \n";
            return;
      }
      std::reverse(this->feasible_points_.begin(), this->feasible_points_.end());
      std::cout << "The path found is : \n\t\t\t";
      auto sz = this->feasible_points_.size();
      for(int i = 0; i < sz; i++)
      {
            std::cout << "(" << this->feasible_points_[i].first << "," << this->feasible_points_[i].second << ")";
            if (i == sz - 1)
            {
                  continue;
            }
            std::cout << " -> ";
      }
      std::cout << std::endl;
      return;
}

void Finder::ComputePath()
{
      this->GenerateGrid();
      this->GenerateObstacles();
      auto st_node = std::make_shared<Node> (start_node_);
      auto g_node = std::make_shared<Node> (goal_node_);

      open_list_.emplace(st_node->total_cost_, st_node);
      scanned_nodes_.emplace(st_node);
      auto grid = get_grid();
      int height = 0;
      int width = 0;
      std::tie(height,width) = get_row_col();
      std::vector<std::shared_ptr<Node>> children;
      std::vector<std::pair<int, int>> neighbors = {{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};
      auto g_node_pos = g_node->current_;
      
      const auto valid_child = [&height, &width, &grid] (const auto& child_pos) -> bool
      {
            bool cond1 = child_pos.first >= 0 && child_pos.first < height;
            bool cond2 = child_pos.second >= 0 && child_pos.second < width;
            return (cond1 && cond2 && grid[child_pos.first][child_pos.second] != 1);
      };
      
      const auto find_child = [&] (const auto& node) -> void
      {
            auto node_pos = node->current_;
            for(const auto& n : neighbors)
            {
                  std::pair<int, int> child_pos = {node_pos.first + n.first, node_pos.second + n.second};
                  if(!valid_child(child_pos))
                        continue;
                  auto child = std::make_shared<Node> (child_pos, node);
                  children.emplace_back(child);
            }
      };
      
      const auto is_closed = [&] (auto& node) -> bool
      {
            for(const auto& n : closed_list_)
            {
                  if(n == node)
                  {
                        return true;
                  }
            }
            return false;
      };
      
      const auto is_scanned = [&] (auto& node) -> auto
      {
            for(const auto& n : scanned_nodes_)
            {
                  if(n == node)
                  {
                        return std::make_tuple(true, n->to_go_cost_);
                  }
            }
            return std::make_tuple(false, double(row_*col_));
      };
      
      while (!open_list_.empty())
      {
            auto current = open_list_.top();
            auto current_node = std::get<1> (current);
            
            open_list_.pop();
            scanned_nodes_.erase(current_node);
            closed_list_.emplace(current_node);
            
            if(current_node == g_node)
            {
                  BackTrack(current_node);
                  return;
            }
            
            find_child(current_node);
            
            auto current_node_pos = current_node->current_;
            
            for(const auto& child : children)
            {
                  if(is_closed(child))
                  {
                        continue;
                  }
                  
                  auto child_pos = child->current_;
                  child->to_go_cost_ = current_node->to_go_cost_ + std::sqrt(std::pow(current_node_pos.first - child_pos.first, 2) + std::pow(current_node_pos.second - child_pos.second, 2));
                  child->heuristic_cost_ = std::sqrt(std::pow(g_node_pos.first - current_node_pos.first, 2) + std::pow(g_node_pos.second - current_node_pos.second, 2));
                  child->total_cost_ = child->to_go_cost_ + child->heuristic_cost_;
                  
                  auto child_scan = is_scanned(child);
                  if (std::get<0>(child_scan) && child->to_go_cost_ > std::get<1>(child_scan))
                  {
                        continue;
                  }
                  open_list_.emplace(child->total_cost_,child);
                  scanned_nodes_.emplace(child);
            }
            children.clear();
      }
}



