
#ifndef PYASTAR
#define PYASTAR

#include <numeric> // Standard library import for std::accumulate
#include <queue>
#include <limits>
#include <cmath>
#include <cstddef>
#include <vector>
#include <map>
#include <algorithm>
#include <iostream>
#include <iterator>

#include "pybind11/pybind11.h" // Pybind11 import to define Python bindings
#include "pybind11/stl.h"      // Pybind11 import for STL containers
#include "pybind11/numpy.h"

#define DEBUG 0            // 1 if you want debug printouts
#define INFO 0             // 1 if you want less verbose information
#define NUM_NEIGHBORS 26   // How many neighbors a 3X3X3 cube has
#define LARGE_NUMBER 1.0   // If this number is found in the map, it will be considered as an obstacle (infinite cost)
#define NORM_PATH_COST 1.0 // Default normalizing path cost
#define GOAL_WEIGHT 0.5    // Default Weighting constant to goal value
#define PATH_WEIGHT 0.5    // Default Weighting constant to path cost
#define KEEP_NODES false   // Default to reuse nodes in search (only used in multigoal search)
#define RES 2.0f           // Size (meters) of of each grid cell member in 3X3X3 cube.
#define W0 1.0f            // Weighting between risk on grid and the traversal cost
// Neighbor cube distances on a unit cube
#define ST 1.0f        // left, right, forward, backward, up, down
#define DG1 1.4142135f // root 2, diagonal
#define DG2 1.7320508f // root 3, diagonal and up

template <class T, std::size_t N>
std::ostream &operator<<(std::ostream &o, const std::array<T, N> &arr)
{
  copy(arr.cbegin(), arr.cend(), std::ostream_iterator<T>(o, " "));
  return o;
}

// Node Class, for priority Queue.
// Only has an index (representing postion in 3D grid) and the priority (cost)
class Node
{
public:
  int idx;    // index in the flattened grid
  float cost; // priority
  Node(int i, float c) : idx(i), cost(c) {}
  bool operator<(const Node &n2) const
  {
    // the top of the priority queue is the greatest element by default,
    // but we want the smallest, so flip the sign
    return cost > n2.cost;
  }
  bool operator==(const Node &n2) const
  {
    return idx == n2.idx;
  }
};

// Neighbor Struct
// Simple structure to hold data about the neighbors in a cube, their indexes in the world map and their transition costs
struct NB
{
  int idx; // index of neighbor in world map
  float m; // transition cost
  NB(int a, float b) : idx(a), m(b) {}
  NB() : idx(-1), m(0.0) {}
};

struct NodeData
{
  float path_cost;
  int parent_idx;
  bool closed;
  NodeData(float path_cost_, int parent_idx_) : path_cost(path_cost_), parent_idx(parent_idx_), closed(false) {}
  NodeData() : path_cost(-1), parent_idx(-1), closed(false) {}
};

struct GoalData
{
  int idx;
  float goal_value;
  float path_cost;
  float total_cost;
  bool found;
  GoalData(int idx_, float goal_value_, float path_cost_, float total_cost_) : idx(idx_), goal_value(goal_value_), path_cost(path_cost_), total_cost(total_cost_), found(false) {}
  GoalData() : idx(-1), goal_value(-1), path_cost(-1), total_cost(std::numeric_limits<float>::infinity()), found(false) {}
};

// Must make a custom priority queue that exposes the underlying PROTECTED container
// Allows the ability to resort the priority queue efficiently.
template <
    class T,
    class Container = std::vector<T>,
    class Compare = std::less<typename Container::value_type>>
class PriorityQueueContainer : public std::priority_queue<T, Container, Compare>
{
public:
  Container &getContainer() { return this->c; }
};

class PyMultiAStar
{
public:
  PyMultiAStar(pybind11::array_t<float> map, bool allow_diag = true, float map_res = RES, float obstacle_value = LARGE_NUMBER, float normalizing_path_cost = 1.0, float goal_weight = 0.5, float path_weight = 0.5, bool keep_nodes = false, float path_w0 = W0); // Constructor

  // Search for multiple goals that have a value associated with them
  std::tuple<pybind11::array_t<int, 2>, pybind11::dict> search_multiple(std::array<int, 3> start_cell, std::vector<std::tuple<std::array<int, 3>, float>> goal_cells);
  // Private member function
  std::tuple<pybind11::array_t<int, 2>, float, int> search_single(std::array<int, 3> &start_cell, const GoalData &primary_goal, std::vector<GoalData> &goals);
  // Classic A*, 1 goal and no goal value
  std::tuple<pybind11::array_t<int, 2>, float> search_single_public(std::array<int, 3> start_cell, std::array<int, 3> goal_cell);
  inline void getNeighbors(struct NB nbrs[], std::array<int, 3> &cell);

  void resort_queue(std::array<int, 3> &new_goal)
  {
    std::vector<Node> &cont = frontier.getContainer();
    for (auto &&node : cont)
    {
      std::array<int, 3> node_cell = index_to_cell(node.idx);
      node.cost = nodes_hash[node.idx].path_cost + octile(node_cell, new_goal);
    }
    std::make_heap(cont.begin(), cont.end());
  }

  void reset_nodes()
  {
    nodes_hash.clear();
    frontier = PriorityQueueContainer<Node>();
  }

  int get_index(std::array<int, 3> &cell)
  {
    return cell[2] + map_depth * (cell[1] + map_width * cell[0]);
  }

  int get_index(int i, int j, int k, int w, int d)
  {
    return k + d * (j + w * i);
  }

  std::array<int, 3> index_to_cell(int index)
  {
    std::array<int, 3> cell;
    cell[0] = index / (map_width * map_depth);
    cell[1] = (index - (map_width * map_depth * cell[0])) / map_depth;
    cell[2] = (index - (map_width * map_depth * cell[0])) % map_depth;
    return cell;
  }

  // L_1 norm (manhattan distance)
  float l1_norm(std::array<int, 3> &cell1, std::array<int, 3> &cell2)
  {
    return (float)std::abs(cell1[0] - cell2[0]) + std::abs(cell1[1] - cell2[1]) + std::abs(cell1[2] - cell2[2]);
  }

  float l2_norm(std::array<int, 3> &cell1, std::array<int, 3> &cell2)
  {
    return map_res * (float)std::sqrt(std::pow(std::abs(cell1[0] - cell2[0]), 2) + std::pow(std::abs(cell1[1] - cell2[1]), 2) + std::pow(std::abs(cell1[2] - cell2[2]), 2));
  }

  float octile(std::array<int, 3> &cell1, std::array<int, 3> &cell2)
  {
    int i0 = cell1[0], j0 = cell1[1], k0 = cell1[2];
    int i1 = cell2[0], j1 = cell2[1], k1 = cell2[2];
    int a = std::abs(i0 - i1), b = std::abs(j0 - j1), c = std::abs(k0 - k1);
    if (a >= b)
    {
      if (b >= c)
      {
        return map_res * (a * ST + (DG1 - ST) * b + (DG2 - DG1) * c);
      }
      else
      {
        if (a >= c)
        {
          return map_res * (a * ST + (DG1 - ST) * c + (DG2 - DG1) * b);
        }
        else
        {
          return map_res * (c * ST + (DG1 - ST) * a + (DG2 - DG1) * b);
        }
      }
    }

    // b is greater than a
    if (a >= c)
    {
      return map_res * (b * ST + (DG1 - ST) * a + (DG2 - DG1) * c);
    }
    else
    {
      if (b >= c)
      {
        // b is greater than a, c is greater than a, and b is greater than c
        return map_res * (b * ST + (DG1 - ST) * c + (DG2 - DG1) * a);
      }
      else
      {
        return map_res * (c * ST + (DG1 - ST) * b + (DG2 - DG1) * a);
      }
    }

    return map_res * (c * ST + (DG1 - ST) * a + (DG2 - DG1) * b);
  }

  pybind11::array_t<int, 2> convert_path(std::vector<int> const &path1D)
  {
    int n = static_cast<int>(path1D.size());
    auto path = pybind11::array_t<int, 2>({n, 3});
    auto path_ = path.mutable_unchecked<2>();
    std::array<int, 3> cell;

    // Iterate through the path backwards
    int path_idx = 0;
    for (std::vector<int>::size_type i = n - 1; i != (std::vector<int>::size_type) - 1; i--, path_idx++)
    {
      cell = index_to_cell(path1D[i]);
      path_(path_idx, 0) = cell[0];
      path_(path_idx, 1) = cell[1];
      path_(path_idx, 2) = cell[2];
    }
    // I believe -03 optimization will make this a move, and not a copy.
    return path;
  }

  std::vector<int> get_path(int start_idx, int goal_idx)
  {
    std::vector<int> path1D;
    int path_idx = goal_idx;
    path1D.push_back(path_idx);
    while (path_idx != start_idx)
    {
      NodeData &nd = nodes_hash[path_idx];
      path_idx = nd.parent_idx;
      path1D.push_back(path_idx);
    }
    return path1D;
  }

  void set_normalizing_path_cost(float cost)
  {
    normalizing_path_cost = cost;
  }

  float get_normalizing_path_cost()
  {
    return normalizing_path_cost;
  }

private:
  pybind11::array_t<float> map;
  // std::array<int, 3> start_cell;
  // std::vector<std::tuple<std::array<int, 3>, float>> goal_cells;
  bool allow_diag;
  float map_res;
  float obstacle_value;
  float path_w0;
  float normalizing_path_cost;
  float goal_weight;
  float path_weight;
  bool keep_nodes;
  int map_width;
  int map_height;
  int map_depth;
  // std::unordered_map<int, int> paths_hash;
  // std::unordered_map<int, float> costs_hash;
  std::unordered_map<int, NodeData> nodes_hash;
  PriorityQueueContainer<Node> frontier;
};

#endif
