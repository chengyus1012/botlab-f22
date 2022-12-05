#include <planning/frontiers.hpp>
#include <planning/motion_planner.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/timestamp.h>
#include <common_utils/geometric/angle_functions.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/robot_path_t.hpp>
#include <queue>
#include <set>
#include <cassert>


bool is_frontier_cell(int x, int y, const OccupancyGrid& map);
frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers);
mbot_lcm_msgs::robot_path_t path_to_frontier(const frontier_t& frontier,
                                              const mbot_lcm_msgs::pose_xyt_t& pose,
                                              const OccupancyGrid& map,
                                              const MotionPlanner& planner);
mbot_lcm_msgs::pose_xyt_t nearest_navigable_cell(mbot_lcm_msgs::pose_xyt_t pose,
                                                  Point<float> desiredPosition,
                                                  const OccupancyGrid& map,
                                                  const MotionPlanner& planner);
mbot_lcm_msgs::pose_xyt_t search_to_nearest_free_space(Point<float> position,
                                                        const OccupancyGrid& map,
                                                        const MotionPlanner& planner);
double path_length(const mbot_lcm_msgs::robot_path_t& path);


std::vector<frontier_t> find_map_frontiers(const OccupancyGrid& map, 
                                           const mbot_lcm_msgs::pose_xyt_t& robotPose,
                                           double minFrontierLength)
{
    /*
    * To find frontiers, we use a connected components search in the occupancy grid. Each connected components consists
    * only of cells where is_frontier_cell returns true. We scan the grid until an unvisited frontier cell is
    * encountered, then we grow that frontier until all connected cells are found. We then continue scanning through the
    * grid. This algorithm can also perform very fast blob detection if you change is_frontier_cell to some other check
    * based on pixel color or another condition amongst pixels.
    */
    std::vector<frontier_t> frontiers;
    std::set<Point<int>> visitedCells;
    
    Point<int> robotCell = global_position_to_grid_cell(Point<float>(robotPose.x, robotPose.y), map);
    std::queue<Point<int>> cellQueue;
    cellQueue.push(robotCell);
    visitedCells.insert(robotCell);
  
    // Use a 4-way connected check for expanding through free space.
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    // Do a simple BFS to find all connected free space cells and thus avoid unreachable frontiers
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            
            // If the cell has been visited or isn't in the map, then skip it
            if(visitedCells.find(neighbor) != visitedCells.end() || !map.isCellInGrid(neighbor.x, neighbor.y))
            {
                continue;
            }
            // If it is a frontier cell, then grow that frontier
            else if(is_frontier_cell(neighbor.x, neighbor.y, map))
            {
                frontier_t f = grow_frontier(neighbor, map, visitedCells);
                
                // If the frontier is large enough, then add it to the collection of map frontiers
                if(f.cells.size() * map.metersPerCell() >= minFrontierLength)
                {
                    frontiers.push_back(f);
                }
            }
            // If it is a free space cell, then keep growing the frontiers
            else if(map(neighbor.x, neighbor.y) < 0)
            {
                visitedCells.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }    
    return frontiers;
}

struct CompareCentroids
{
    CompareCentroids(mbot_lcm_msgs::pose_xyt_t robotPose) { this->robotPose = robotPose;}
    inline bool operator() (const Point<double>& centr_1, const Point<double>& centr_2)
    {
        // Diff 1
        float diff_1_x = robotPose.x - centr_1.x;
        float diff_1_y = robotPose.y - centr_1.y;
        float diff_1 = diff_1_x * diff_1_x + diff_1_y * diff_1_y;
        // Diff 2
        float diff_2_x = robotPose.x - centr_2.x;
        float diff_2_y = robotPose.y - centr_2.y;
        float diff_2 = diff_2_x * diff_2_x + diff_2_y * diff_2_y;

        return (diff_1 < diff_2);
    }
    mbot_lcm_msgs::pose_xyt_t robotPose;
};

frontier_processing_t plan_path_to_frontier(const std::vector<frontier_t>& frontiers, 
                                            const mbot_lcm_msgs::pose_xyt_t& robotPose,
                                            const OccupancyGrid& map,
                                            const MotionPlanner& planner)
{
    ///////////// TODO: Implement your strategy to select the next frontier to explore here //////////////////
    /*
    * NOTES:
    *   - If there's multiple frontiers, you'll need to decide which to drive to.
    *   - A frontier is a collection of cells, you'll need to decide which one to attempt to drive to.
    *   - The cells along the frontier might not be in the configuration space of the robot, so you won't necessarily
    *       be able to drive straight to a frontier cell, but will need to drive somewhere close.
    */

    // First, choose the frontier to go to
    // Initial alg: find the nearest one
    std::vector<Point<double>> goal_list;
    for(auto frontier : frontiers){
        goal_list.push_back(find_valid_goal_search(frontier, map, robotPose, planner));
    } // global central position

    CompareCentroids CentrComparator(robotPose);

    std::sort(goal_list.begin(), goal_list.end(), CentrComparator);
    

    bool path_valid = false;
    int i = 0;
    int unreachable_frontiers = 0;
    mbot_lcm_msgs::robot_path_t path;
    while (!path_valid && i<goal_list.size())
    {
        Point<double> closest_point = goal_list[i];
        mbot_lcm_msgs::pose_xyt_t goal;

        goal.x = closest_point.x;
        goal.y = closest_point.y;
        goal.theta = 0;
        path = planner.planPath(robotPose, goal);
        // path.utime = utime_now();
        // path.path.push_back(robotPose);
        if(path.path_length <= 1)
        {
            i++;
            unreachable_frontiers++;
        }
        else
        {
            path_valid = true;
            break;
        }
    }
    
    return frontier_processing_t(path, unreachable_frontiers);
}


bool is_frontier_cell(int x, int y, const OccupancyGrid& map)
{
    // A cell is a frontier if it has log-odds 0 and a neighbor has log-odds < 0
    
    // A cell must be in the grid and must have log-odds 0 to even be considered as a frontier
    if(!map.isCellInGrid(x, y) || (map(x, y) != 0))
    {
        return false;
    }
    
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    for(int n = 0; n < kNumNeighbors; ++n)
    {
        // If any of the neighbors are free, then it's a frontier
        // Note that logOdds returns 0 for out-of-map cells, so no explicit check is needed.
        if(map.logOdds(x + xDeltas[n], y + yDeltas[n]) < 0)
        {
            return true;
        }
    }
    
    return false;
}


frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers)
{
    // Every cell in cellQueue is assumed to be in visitedFrontiers as well
    std::queue<Point<int>> cellQueue;
    cellQueue.push(cell);
    visitedFrontiers.insert(cell);
    
    // Use an 8-way connected search for growing a frontier
    const int kNumNeighbors = 8;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = {  0,  1, -1, 0, 1,-1, 1,-1 };
 
    frontier_t frontier;
    
    // Do a simple BFS to find all connected frontier cells to the starting cell
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // The frontier stores the global coordinate of the cells, so convert it first
        frontier.cells.push_back(grid_position_to_global_position(nextCell, map));
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            if((visitedFrontiers.find(neighbor) == visitedFrontiers.end()) 
                && (is_frontier_cell(neighbor.x, neighbor.y, map)))
            {
                visitedFrontiers.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    
    return frontier;
}

Point<double> find_valid_goal_projection(const frontier_t& frontier, const OccupancyGrid& map, const mbot_lcm_msgs::pose_xyt_t& robotPose)
{
    int n = frontier.cells.size();
    Point<double> start = frontier.cells[0];
    Point<double> end = frontier.cells[n-1];
    Point<double> center = find_frontier_centroid(frontier);
    Point<double> goal;

    double frontier_angle = atan2(end.y - start.y, end.x - start.x);
    double theta = M_PI_2 + frontier_angle;
    
    // frontier_angle = wrap_to_2pi(frontier_angle);
    // double bot_angle = wrap_to_2pi(robotPose.theta);
    double dis = 0.25;
    Point<double> positive_direction(start.x + dis*cos(theta), start.y + dis * sin(theta));
    Point<int> positive_cell = global_position_to_grid_cell(positive_direction, map);
    Point<double> negative_direction(start.x - dis*cos(theta), start.y - dis * sin(theta));
    Point<int> negative_cell = global_position_to_grid_cell(negative_direction, map);

    if(map.isCellInGrid(positive_cell.x, positive_cell.y))
    {
        goal = positive_direction;
    }
    else if (map.isCellInGrid(negative_cell.x, negative_cell.y))
    {
        goal = negative_direction;
    }
    else
    {
        std::cout << " can not find valid goal" << std::endl;
    }

    return goal;

}

Point<double> find_valid_goal_search(const frontier_t& frontier, 
                                    const OccupancyGrid& map, 
                                    const mbot_lcm_msgs::pose_xyt_t& robotPose, 
                                    const MotionPlanner& planner)
{
    int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    int dy[8] = {0, 0, 1, -1, 1, 1, -1, -1};

    Point<double> center = find_frontier_centroid(frontier);
    cell_t center_cell = global_position_to_grid_cell(center, map);
    f_Node* center_node = new f_Node(center_cell.x, center_cell.y);
    f_Node* valid_node;

    ObstacleDistanceGrid distances = planner.obstacleDistances();
    f_PriorityQueue openList;
    std::vector<f_Node*> closedList;
    std::vector<f_Node*> searchedList;
    center_node->g_cost = 0;
    openList.push(center_node);

    bool found_cell = false;
    while(!openList.empty() && found_cell == false)
    {
        f_Node* currentNode = openList.pop();
        int x = currentNode->cell.x;
        int y = currentNode->cell.y;

        if(distances.isCellInGrid(x,y) && distances(x,y)>planner.searchparams().minDistanceToObstacle)
        {
            found_cell == true;
            valid_node = currentNode;
        }
        else
        {
            closedList.push_back(currentNode);
            for(int i=0; i<8; i++)
            {
                int x = currentNode->cell.x + dx[i];
                int y = currentNode->cell.y + dy[i];
                f_Node* neighbor = new f_Node(x,y);
                if(f_is_in_list(neighbor, searchedList))
                    neighbor = f_get_from_list(neighbor, searchedList);
                
                if(!f_is_in_list(neighbor, closedList) && distances.isCellInGrid(x, y) && distances(x, y) > planner.searchparams().minDistanceToObstacle)
                {
                    if(!f_is_in_list(neighbor, searchedList))
                    {
                        neighbor->g_cost = g_cost_frontier(currentNode, neighbor, distances, planner.searchparams());
                        openList.push(neighbor);
                        searchedList.push_back(neighbor);
                    }
                    else if(neighbor->g_cost > g_cost_frontier(currentNode, neighbor, distances, planner.searchparams()))
                    {
                        neighbor->g_cost = g_cost_frontier(currentNode, neighbor, distances, planner.searchparams());

                        openList.push(neighbor);
                    }
                }
            }

        }
        
    }
    cell_t valid_cell = valid_node->cell;
    Point<double> valid_position = grid_position_to_global_position(valid_cell, map);
    return valid_position;
}

double g_cost_frontier(f_Node* from, f_Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return calculated g cost
    double g_cost = from->g_cost;

    int dx = abs(goal->cell.x - from->cell.x);
    int dy = abs(goal->cell.y - from->cell.y);

    if(dx == 1 && dy == 1){
        g_cost += 1.41;
    }else{
        g_cost += 1.0;
    }

    // Penalize if close to obstacle
    double penalization = 0.0;
    if(distances(goal->cell.x, goal->cell.y) <= params.maxDistanceWithCost)
    {
        penalization = (params.maxDistanceWithCost - distances(goal->cell.x, goal->cell.y)) * params.distanceCostExponent;
    }

    g_cost = g_cost*distances.metersPerCell() + penalization;
    return g_cost;
}

Point<double> find_frontier_centroid(const frontier_t& frontier)
{
    // Using the mid point of the frontier
    Point<double> mid_point;
    int index = (int)(frontier.cells.size() / 2.0);
    // printf("index: %d, size: %d\n", index, frontier.cells.size());
    mid_point = frontier.cells[index];
    printf("Mid point of frontier: (%f,%f)\n", mid_point.x, mid_point.y);

    return mid_point;
}

bool f_is_in_list(f_Node* node, std::vector<f_Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

f_Node* f_get_from_list(f_Node* node, std::vector<f_Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;
    
}
