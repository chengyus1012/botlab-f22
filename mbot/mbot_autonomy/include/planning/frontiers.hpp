#ifndef PLANNING_FRONTIERS_HPP
#define PLANNING_FRONTIERS_HPP

#include <common_utils/geometric/point.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/robot_path_t.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <planning/astar.hpp>
#include <vector>
#include <queue>

class MotionPlanner;
class OccupancyGrid;
// typedef Point<int> cell_t;

/**
* frontier_t represents a frontier in the map. A frontier is a collection of contiguous cells in the occupancy grid that
* sits on the border of known free space and unknown space.
*/
struct frontier_t
{
    std::vector<Point<float>> cells;  // The global coordinate of cells that make up the frontier
};

struct frontier_processing_t
{
    frontier_processing_t(mbot_lcm_msgs::robot_path_t path, int unreachable_frontiers)
    {
        this->path_selected = path;
        this->num_unreachable_frontiers = unreachable_frontiers;
    }
    mbot_lcm_msgs::robot_path_t path_selected;
    int num_unreachable_frontiers;
};

struct f_Node 
{
    double g_cost;
    cell_t cell;
    f_Node(int a, int b) : g_cost(1.0E16), cell(a,b) {}

    // double f_cost(void) const { return g_cost + h_cost; }
    bool operator==(const f_Node& rhs) const
    {
        // std::cout << "comparing " << (cell == rhs.cell) << std::endl;
        return (cell == rhs.cell);
    }
};

struct f_Compare_Node
{
    bool operator() (Node* n1, Node* n2)
    {
        return (n1->g_cost >= n2->g_cost);
    }
};

struct f_PriorityQueue
{
    std::queue<Node*> Q;
    std::vector<Node*> elements;

    bool empty()
    {
        return Q.empty();
    }

    bool is_member(Node* n)
    {
        for (auto &&node : elements)
        {
            if (*n == *node) return true;
        }
        return false;
    }

    Node* get_member(Node* n)
    {
        for (auto &&node : elements)
        {
            if (*n == *node) return node;
        }
        return NULL;
    }

    Node* pop()
    {
        Node* n = Q.front();
        Q.pop();
        int idx = -1;
        // Remove the node from the elements vector
        // for (unsigned i = 0; i < elements.size(); i++)
        // {
        //     if (*elements[i] == *n)
        //     {
        //         idx = i;
        //         break;
        //     }
        // }
        // elements.erase(elements.begin() + idx);
        return n;
        
    }

    void push(Node* n)
    {
        Q.push(n);
        // elements.push_back(n);
    }
};


/**
* find_map_frontiers locates all frontiers in the provided map. A frontier cell is an unknown cell (log-odds == 0) that
* borders a free space cell (log-odds < 0). A frontier is a contiguous region of frontier cells.
* 
* The frontiers found by this function are all frontiers that are reachable through free space from the current robot
* pose. By defining frontiers in this way, you don't have to worry about a small mapping error creating an unreachable
* frontier that's on the opposite side of a wall. All frontiers returned by this function are connected by free space
* and therefore reachable -- potentially, as the exact reachability depends on the configuration space of the robot.
* 
* \param    map                     Map in which to find the frontiers
* \param    robotPose               Pose of the robot at the start
* \param    minFrontierLength       Minimum length of a valid frontier (meters) (optional, default = 0.1m)
* \return   All frontiers found in the map. A fully-explored map will have no frontiers, so the returned vector will be
*   empty in that case.
*/
std::vector<frontier_t> find_map_frontiers(const OccupancyGrid& map, 
                                           const mbot_lcm_msgs::pose_xyt_t& robotPose,
                                           double minFrontierLength = 0.35);


/**
* plan_path_to_frontier selects amongst the available frontiers and plans a path to one of them. The path to the
* frontier is returned. If no frontiers exist or there are no valid paths to any of the frontiers, then a path of length
* 1, with the only pose being the robot pose should be returned indicating an error.
* 
* \param    frontiers           Frontiers in the environment
* \param    robotPose           Pose of the robot from which to plan
* \param    map                 Map being explored
* \param    planner             Planner to use for finding the next frontier
* \return   Path to the selected frontier or a path indicating failure, as described above.
*/
frontier_processing_t plan_path_to_frontier(const std::vector<frontier_t>& frontiers, 
                                   const mbot_lcm_msgs::pose_xyt_t& robotPose,
                                   const OccupancyGrid& map,
                                   const MotionPlanner& planner);


Point<double> find_frontier_centroid(const frontier_t& frontier);

bool is_frontier_cell(int x, int y, const OccupancyGrid& map);
/**
* find_valid_goal projection method to find a valid goal (not work well)
*/
Point<double> find_valid_goal_projection(const frontier_t& frontier, const OccupancyGrid& map, const mbot_lcm_msgs::pose_xyt_t& robotPose);
Point<double> find_valid_goal_search(const frontier_t& frontier, 
                                    const OccupancyGrid& map, 
                                    const mbot_lcm_msgs::pose_xyt_t& robotPose, 
                                    const MotionPlanner& planner,
                                    bool& flag);
double g_cost_frontier(f_Node* from, f_Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params);
bool f_is_in_list(f_Node* node, std::vector<f_Node*> list);
f_Node* f_get_from_list(f_Node* node, std::vector<f_Node*> list);
bool is_centroid_reachable(const Point<double>& centroid, 
                            const mbot_lcm_msgs::pose_xyt_t& robotPose,
                            const OccupancyGrid& map,
                            const MotionPlanner& planner);

#endif // PLANNING_FRONTIERS_HPP
