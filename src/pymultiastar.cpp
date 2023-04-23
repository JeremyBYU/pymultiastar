#include "include/pymultiastar.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

void printArray(struct NB data[], int length)
{
  for (int i(0); i < length; ++i)
  {
    std::cout << data[i].idx << ' ';
  }
  std::cout << std::endl;
}

inline void PyMultiAStar::getNeighbors(struct NB nbrs[], std::array<int, 3> &cell)
{
  int row = cell[0], col = cell[1], depth = cell[2];
  int &h = map_height, &w = map_width, &d = map_depth;
  bool &diag_ok = allow_diag;
  // Middle of Cube
  nbrs[0] = (diag_ok && row > 0 && col > 0) ? NB(get_index(row - 1, col - 1, depth, w, d), DG1) : NB(-1, 0.0);         // -Y -X
  nbrs[1] = (row > 0) ? NB(get_index(row - 1, col, depth, w, d), ST) : NB(-1, 0.0);                                    // -Y
  nbrs[2] = (diag_ok && row > 0 && col + 1 < w) ? NB(get_index(row - 1, col + 1, depth, w, d), DG1) : NB(-1, 0.0);     // -Y +X
  nbrs[3] = (col > 0) ? NB(get_index(row, col - 1, depth, w, d), ST) : NB(-1, 0.0);                                    // -X
  nbrs[4] = (col + 1 < w) ? NB(get_index(row, col + 1, depth, w, d), ST) : NB(-1, 0.0);                                // +X
  nbrs[5] = (diag_ok && row + 1 < h && col > 0) ? NB(get_index(row + 1, col - 1, depth, w, d), DG1) : NB(-1, 0.0);     // +Y -X
  nbrs[6] = (row + 1 < h) ? NB(get_index(row + 1, col, depth, w, d), ST) : NB(-1, 0.0);                                // +Y
  nbrs[7] = (diag_ok && row + 1 < h && col + 1 < w) ? NB(get_index(row + 1, col + 1, depth, w, d), DG1) : NB(-1, 0.0); // +Y +X
  // Top of Cube
  nbrs[8] = (diag_ok && row > 0 && col > 0 && depth + 1 < d) ? NB(get_index(row - 1, col - 1, depth + 1, w, d), DG2) : NB(-1, 0.0);          // -Y -X +Z
  nbrs[9] = (row > 0 && depth + 1 < d) ? NB(get_index(row - 1, col, depth + 1, w, d), DG1) : NB(-1, 0.0);                                    // -Y +Z
  nbrs[10] = (diag_ok && row > 0 && col + 1 < w && depth + 1 < d) ? NB(get_index(row - 1, col + 1, depth + 1, w, d), DG2) : NB(-1, 0.0);     // -Y +X+Z
  nbrs[11] = (col > 0 && depth + 1 < d) ? NB(get_index(row, col - 1, depth + 1, w, d), DG1) : NB(-1, 0.0);                                   // -X +Z
  nbrs[12] = (depth + 1 < d) ? NB(get_index(row, col, depth + 1, w, d), ST) : NB(-1, 0.0);                                                   // +Z
  nbrs[13] = (col + 1 < w && depth + 1 < d) ? NB(get_index(row, col + 1, depth + 1, w, d), DG1) : NB(-1, 0.0);                               // +X +Z
  nbrs[14] = (diag_ok && row + 1 < h && col > 0 && depth + 1 < d) ? NB(get_index(row + 1, col - 1, depth + 1, w, d), DG2) : NB(-1, 0.0);     // +Y -X +Z
  nbrs[15] = (row + 1 < h && depth + 1 < d) ? NB(get_index(row + 1, col, depth + 1, w, d), DG1) : NB(-1, 0.0);                               // +Y +Z
  nbrs[16] = (diag_ok && row + 1 < h && col + 1 < w && depth + 1 < d) ? NB(get_index(row + 1, col + 1, depth + 1, w, d), DG2) : NB(-1, 0.0); // +Y +X +Z

  // Bottom of Cube
  nbrs[17] = (diag_ok && row > 0 && col > 0 && depth > 0) ? NB(get_index(row - 1, col - 1, depth - 1, w, d), DG2) : NB(-1, 0.0);         // -Y -X -Z
  nbrs[18] = (row > 0 && depth > 0) ? NB(get_index(row - 1, col, depth - 1, w, d), DG1) : NB(-1, 0.0);                                   // -Y -Z
  nbrs[19] = (diag_ok && row > 0 && col + 1 < w && depth > 0) ? NB(get_index(row - 1, col + 1, depth - 1, w, d), DG2) : NB(-1, 0.0);     // -Y +X-Z
  nbrs[20] = (col > 0 && depth > 0) ? NB(get_index(row, col - 1, depth - 1, w, d), DG1) : NB(-1, 0.0);                                   // -X -Z
  nbrs[21] = (depth > 0) ? NB(get_index(row, col, depth - 1, w, d), ST) : NB(-1, 0.0);                                                   // -Z
  nbrs[22] = (col + 1 < w && depth > 0) ? NB(get_index(row, col + 1, depth - 1, w, d), DG1) : NB(-1, 0.0);                               // +X -Z
  nbrs[23] = (diag_ok && row + 1 < h && col > 0 && depth > 0) ? NB(get_index(row + 1, col - 1, depth - 1, w, d), DG2) : NB(-1, 0.0);     // +Y -X -Z
  nbrs[24] = (row + 1 < h && depth > 0) ? NB(get_index(row + 1, col, depth - 1, w, d), DG1) : NB(-1, 0.0);                               // +Y -Z
  nbrs[25] = (diag_ok && row + 1 < h && col + 1 < w && depth > 0) ? NB(get_index(row + 1, col + 1, depth - 1, w, d), DG2) : NB(-1, 0.0); // +Y +X -Z
}

// Constructor for PyMultiastar
PyMultiAStar::PyMultiAStar(py::array_t<float> map, bool allow_diag, float map_res, float obstacle_value,  float normalizing_path_cost, float goal_weight, float path_weight,
                           bool keep_nodes, float path_w0) : map(map), allow_diag(allow_diag), map_res(map_res), obstacle_value(obstacle_value), normalizing_path_cost(normalizing_path_cost),
                                              goal_weight(goal_weight), path_weight(path_weight), keep_nodes(keep_nodes), path_w0(path_w0)
{
  auto shape = map.shape();
  map_height = static_cast<int>(shape[0]), map_width = static_cast<int>(shape[1]), map_depth = static_cast<int>(shape[2]);
}

//Singe Search Public API
std::tuple<pybind11::array_t<int, 2>, float> PyMultiAStar::search_single_public(std::array<int, 3> start_cell, std::array<int, 3> goal_cell)
{
  GoalData primary_goal = GoalData(get_index(goal_cell), 1, 1, 1); // make goal data structure, goal values and costs dont matter so set them all to 1
  std::vector<GoalData> goals = {primary_goal};

  py::array_t<int, 2> path;
  float path_cost;
  int goal_index;
  std::tie(path, path_cost, goal_index) = search_single(start_cell, primary_goal, goals); // search and destructure the results
  return std::make_tuple(path, path_cost);
}

// Single Search torwards one primary goal. Goal checking looks at all the goals
std::tuple<py::array_t<int, 2>, float, int> PyMultiAStar::search_single(std::array<int, 3> &start_cell, const GoalData &primary_goal, std::vector<GoalData> &goals)
{
  // auto shape = map.shape();
  auto map_ = map.unchecked<3>();
  // std::cout << shape[0] << " " << shape[1] << " "  << shape[2] << "; " << map_(0, 0, 0)  <<std::endl;
  std::array<int, 3> goal_cell = index_to_cell(primary_goal.idx);
  int start_idx = get_index(start_cell);

  // Create a condensed, sorted, integer array for fast goal checking (we may have multiple goals)
  std::vector<int> goals_condensed;
  for (auto const &elem : goals)
  {
    if (!elem.found)
      goals_condensed.push_back(elem.idx);
    
  }
  std::sort(goals_condensed.begin(), goals_condensed.end());
  // Resort the priority queue if we are keeping the nodes, else just empty them
  if (keep_nodes)
  {
    resort_queue(goal_cell);
  }
  else
  {
    reset_nodes();
    // std::printf("Resetting Nodes - Hash size: %d, \n", nodes_hash.size());
  }
  // create the start and goal node, initialize the costs
  Node start_node(start_idx, 0.);
  Node goal_node(primary_goal.idx, 0.);

  nodes_hash[start_idx] = NodeData{0, -1};

  // // Note: To speed up the algorithm we MAY have duplicate nodes in the frontier
  // // See: https://www.redblobgames.com/pathfinding/a-star/implementation.html#algorithm
  frontier.push(start_node);

  // An array to hold neighbors of the currently expanded node
  NB nbrs[NUM_NEIGHBORS];

  // counters for # of nodes expanded
  int expansions = 0;
  int duplicates = 0;

  bool solution_found = false;
  while (!frontier.empty())
  {
    //   // .top() doesn't actually remove the node
    Node cur = frontier.top();

    // row,col,depth indexes of current node
    std::array<int, 3> cur_cell = index_to_cell(cur.idx);
    int row = cur_cell[0];
    int col = cur_cell[1];
    int depth = cur_cell[2];

    if (DEBUG)
    {
      std::printf("\nExpanding Node: idx = %d; (%d, %d, %d)\n", cur.idx, row, col, depth);
    }

    // Check if the current node is the goal node.
    if (std::binary_search(goals_condensed.begin(), goals_condensed.end(), cur.idx))
    {
      if (DEBUG)
      {
        std::printf("Found a goal: idx = %d; (%d, %d, %d)\n", cur.idx, row, col, depth);
      }
      goal_node.idx = cur.idx; // NOTE: Could be different than the primary goal which drives the heuristic
      goal_node.cost = nodes_hash[cur.idx].path_cost;
      solution_found = true;
      break;
    }

    frontier.pop();
    expansions++;

    // Our implementation allows the possibility to have previously expanded nodes on the frontier (this allows a speed up, see comment on priority queue declaration)
    // This if statement checks this situation and skips this node if it has already been expanded
    // If this is not done, nothing bad will occur, just extra CPU cycles wasted generating neighbors that have already
    // been generated and whose costs will be the **same** as before, which prevents them from being added to the frontier.
    NodeData &cur_node_data = nodes_hash[cur.idx];
    if (cur_node_data.closed)
    {
      duplicates++;
      continue;
    }
    else
    {
      cur_node_data.closed = true;
    }
    // Fill nbrs data structure with nearby neighbors that are valid
    getNeighbors(nbrs, cur_cell);

    if (DEBUG)
    {
      printArray(nbrs, NUM_NEIGHBORS);
    }

    float heuristic_cost;
    for (int i = 0; i < NUM_NEIGHBORS; ++i)
    {
      // Check if a valid neighbor (outside map, etc.)
      if (nbrs[i].idx >= 0)
      {
        int nbr_idx = nbrs[i].idx;
        std::array<int, 3> nbr_cell = index_to_cell(nbr_idx);
        int row_n = nbr_cell[0];
        int col_n = nbr_cell[1];
        int depth_n = nbr_cell[2];
        if (DEBUG)
        {
          std::printf("Inspecting neighbor idx: %d; (%d, %d, %d)\t", nbr_idx, row_n, col_n, depth_n);
          // std::printf("Made it to here \n");
          // std::printf("Map shape %d \n", map.shape()[0]);
        }
        // Dont expand neighbors that are obstacles
        // std::printf("Yo\n");
        if (map_(row_n, col_n, depth_n) >= obstacle_value)
        {
          if (DEBUG)
          {
            std::printf("Skipping Obstacle Node: %.3f\n", map_(row_n, col_n, depth_n));
          }
          continue;
        }
        // std::printf("No Obstacle\n");
        // the sum of the cost so far and the cost of this move
        // new cost =    Parent Cost             + Transition Cost     + Weighted Risk Cost
        float new_cost = cur_node_data.path_cost + map_res * nbrs[i].m + W0 * map_res * map_(row_n, col_n, depth_n);
        auto nbr_node_data = nodes_hash.find(nbr_idx);
        // std::printf("Got Node Data\n");
        if (nbr_node_data == nodes_hash.end() || new_cost < nbr_node_data->second.path_cost)
        {
          // std::printf("Before Debug Current Cost\n");
          if (DEBUG)
          {
            std::printf("Path Cost %.3f \n", new_cost);
          }
          // estimate the cost to the goal based on legal moves
          if (allow_diag)
          {
            heuristic_cost = octile(goal_cell, nbr_cell);
          }
          else
          {
            heuristic_cost = l1_norm(goal_cell, nbr_cell);
          }

          // paths with lower expected cost are explored first
          float priority = new_cost + heuristic_cost;
          if (DEBUG)
          {
            std::printf("Creating a new node for idx: %d; (%d, %d, %d). Priority = %.3f \n", nbr_idx, row_n, col_n, depth_n, priority);
          }

          // Create new node for priority queue
          frontier.push(Node(nbr_idx, priority));
          // check if this is a brand new node
          if (nbr_node_data == nodes_hash.end())
          {
            // new node!
            nodes_hash[nbr_idx] = NodeData{new_cost, cur.idx};
          }
          else
          {
            nbr_node_data->second.path_cost = new_cost;
            nbr_node_data->second.parent_idx = cur.idx;
          }
        }
        else
        {
          if (DEBUG)
          {
            std::printf("\n");
          }
        }
        // std::printf("End of Neighbor loop\n");
      }
    }
  }
  if (DEBUG)
  {
    std::printf("A-Star C++ Planner Finished\n");
    std::printf("Expansions: %d, Duplicates: %d\n", expansions, duplicates);
  }

  std::vector<int> path1D;    // holds the path where each element is the index in the 1D contigious array
  py::array_t<int, 2> path3D; // holds a multidimensional array (numpy) where each element is an array of 3 elemnts (row, col, depth) of the path
  if (solution_found)
  {
    auto it = std::find_if(goals.begin(), goals.end(), [goal_node](const GoalData &goal_) { return goal_node.idx == goal_.idx; });
    int goal_index = static_cast<int>(std::distance(goals.begin(), it));
    path1D = get_path(start_idx, goal_node.idx);
    path3D = convert_path(path1D);
    if (DEBUG)
    {
      std::printf("Goal Node Path Cost: %.3f\n", goal_node.cost);
    }
    return std::make_tuple(path3D, goal_node.cost, goal_index);
  }
  else
  {
    return std::make_tuple(path3D, std::numeric_limits<float>::infinity(), -1);
  }

  // xt::pytensor<int, 2> path({1, 3});
  // return std::make_tuple(path, 5.0, 1);
}

std::tuple<py::array_t<int, 2>, py::dict> PyMultiAStar::search_multiple(std::array<int, 3> start_cell, std::vector<std::tuple<std::array<int, 3>, float>> goal_cells)
{
  // Ensure that
  reset_nodes();

  std::vector<GoalData> goals; // new data structure that has additional fields we need
  std::transform(goal_cells.begin(), goal_cells.end(), std::back_inserter(goals), [&](const std::tuple<std::array<int, 3>, float> &goal) {
    std::array<int, 3> temp_goal = std::get<0>(goal);
    float goal_value = std::get<1>(goal);
    float estimated_path_cost = allow_diag ? octile(start_cell, temp_goal) : l1_norm(start_cell, temp_goal);
    float estimated_total_cost = goal_value * goal_weight + estimated_path_cost / normalizing_path_cost * path_weight;
    // std::cout << estimated_total_cost <<std::endl;
    return GoalData(get_index(temp_goal), goal_value, estimated_path_cost, estimated_total_cost);
  });
  // Sort the goal list
  std::sort(goals.begin(), goals.end(), [](const GoalData &goal_left, const GoalData &goal_right) { return goal_left.total_cost < goal_right.total_cost; });

  std::vector<int> found_goals;          // list of goals that were found, may be useful later
  GoalData best_planned_goal;            // will hold the best goal that has been found
  py::array_t<int, 2> best_planned_path; // will hold path to the best goal

  int total_goal_searches = 0;
  for (auto it = goals.begin(); it != goals.end(); ++it)
  {
    // if this goal has already been found then skip it
    // this can happen when searching for one goal, we find a different goal that is later in the list
    if (it->found)
      continue;
    total_goal_searches++;
    if (DEBUG || INFO)
    {
      std::printf("\n\nBegginning Search of goal idx: %d \n", it->idx);
    }

    py::array_t<int, 2> path;
    float path_cost;
    int goal_index;
    std::tie(path, path_cost, goal_index) = search_single(start_cell, *it, goals); // destructuring
    GoalData &found_goal = goal_index >= 0 ? goals[goal_index] : *it; // goal_index = -1 if no goal is found, therefore use *it to point to the fact that we could not find this goal!
    if (DEBUG || INFO)
    {
      std::printf("Finished Search -  idx: %d, Path Cost: %.3f, goal_index: %d,\n", found_goal.idx, path_cost, goal_index);
    }
    found_goal.path_cost = path_cost;
    found_goal.total_cost = (found_goal.goal_value * goal_weight) + ((path_cost / normalizing_path_cost) * path_weight);
    found_goal.found = true;

    if (DEBUG || INFO)
    {
      std::printf("Best Planned Goal - idx: %d, total cost: %.3f \n", best_planned_goal.idx, best_planned_goal.total_cost);
      std::printf("Found Goal - idx: %d, total cost: %.3f \n", found_goal.idx, found_goal.total_cost);
    }

    if (found_goal.total_cost < best_planned_goal.total_cost)
    {
      best_planned_goal = found_goal;
      best_planned_path = path; // TODO is this a move or a copy?
      if (DEBUG || INFO)
      {
        std::printf("Marking the found goal as Best Planned Goal- idx: %d \n", best_planned_goal.idx);
      }
    }
    // Get the next goal that has NOT been found with the lowest ESTIMATED total cost
    auto it_next = std::find_if(it, goals.end(), [](GoalData &goal) { return !goal.found; });
    float best_estimated_goal_cost = it_next != goals.end() ? it_next->total_cost : std::numeric_limits<float>::infinity(); // if this is the end it will default to infinite cost
    if (best_planned_goal.total_cost <= best_estimated_goal_cost)
    {
      // We are done! We have found a goal that is less than the or equal to the very best goals found and the mininum cost for remaining unplanned goals
      if (DEBUG || INFO)
      {
        printf("Best Planned Goal is now conclusively the best, no need to keep searching other goals! \n");
      }
      break;
    }
    // if found goal does not equal the primary goal (*it) we were searching for, then decrement the iterator so that that
    // we keep searching for the same primary goal in the next loop
    if (it->idx != found_goal.idx)
      --it;
  }

  // Provides some additional meta data that may be useful for analysis
  // goal_index is the index in the argument vector 'goal_cells' for which this function determined was the very best goal/path combination
  // This way the caller knows precisely which is being planned to (they passed the argument)
  int goal_index = -1;
  if (best_planned_goal.idx != -1) {
    goal_index = std::distance(goal_cells.begin(), std::find_if(goal_cells.begin(), goal_cells.end(), [this, &best_planned_goal] (const std::tuple<std::array<int, 3>, float>& goal_cell) { 
      std::array<int, 3> cell = std::get<0>(goal_cell);
      int idx = get_index(cell);
      return idx == best_planned_goal.idx;
      } ));
  }
  py::dict additional_data = py::dict("goal_index"_a=goal_index, "total_goal_searches"_a=total_goal_searches, 
                                      "goal_total_cost"_a=best_planned_goal.total_cost, "goal_path_cost"_a=best_planned_goal.path_cost,
                                      "goal_value"_a=best_planned_goal.goal_value, "num_expansions"_a=0);

  if (DEBUG || INFO)
  {
    std::printf("Finished Multi-goal search \n");
  }
  return std::make_tuple(best_planned_path, additional_data);
}
