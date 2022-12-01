#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;
// https://www.geeksforgeeks.org/a-search-algorithm/
mbot_lcm_msgs::robot_path_t search_for_path(mbot_lcm_msgs::pose_xyt_t start,
                                             mbot_lcm_msgs::pose_xyt_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
     ////////////////// TODO: Implement your A* search here //////////////////////////

    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    Node* goalNode = new Node(goalCell.x, goalCell.y);
    Node* startNode = new Node(startCell.x, startCell.y);

    PriorityQueue openList;
    std::vector<Node*> closedList;

    startNode->g_cost = 0;
    startNode->h_cost = h_cost(startNode, goalNode, distances);

    openList.push(startNode);

    bool found_path = false;
    while(!openList.empty() && found_path == false)
    {
        Node* nextNode = openList.pop();

        std::vector<Node*> children = expand_node(nextNode, distances, params);

        for(auto &&childNode : children){
            childNode->parent = nextNode;

            if(*childNode == *goalNode){
                openList.push(childNode)
                goalNode = childNode;
                found_path = true;
                break
            }

            childNode->g_cost = g_cost(nextNode, childNode, distances, params);
            childNode->h_cost = h_cost(childNode, goalNode, distances);

            if(openList.is_member(childNode)){
                Node* existingNode = openList.get_member(childNode);
                if(childNode->f_cost() > existingNode->f_cost()){
                    continue;
                }
            }
            if(is_in_list(childNode, closedList)){
                existingNode = get_from_list(childNode, closedList);
                if(childNode->f_cost() > existingNode->f_cost()){
                    continue;
                }
            }
            openList.push(childNode)
        }
    }

    mbot_lcm_msgs::robot_path_t path;
    path.utime = start.utime;

    if(found_path){
        std::vector<Node*> nodePath = extract_node_path(goalNode, startNode);
        std::vector<Node*> prunedNodePath = prune_node_path(nodePath);
        path.path = extract_pose_path(prunedNodePath, distances);
    }else{
        std::cout << "did not find path :< \n"<< std::endl;
    } 
    path.path_length = path.path.size();
    return path;
}

double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    // TODO: Return calculated h cost
    // diagonal distance
    int dx = abs(goal->cell.x - from->cell.x);
    int dy = abs(goal->cell.y - from->cell.y);
    double straight_distance = 1.0;
    double diag_distance = 1.41;

    double h_cost = straight_distance*(dx + dy) + (diag_distance - 2*straight_distance) * std::min(dx, dy);
    return h_cost;
}
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
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
        penalization = pow(params.maxDistanceWithCost - distances(goal->cell.x, goal->cell.y), params.distanceCostExponent);
    }

    g_cost += penalization;
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return children of a given node that are not obstacles
    int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    int dy[8] = {0, 0, 1, -1, 1, 1, -1, -1};

    std::vector<Node*> children;
    for(int i=0; i<8; i++){
        int x = node->cell.x + dx[i];
        int y = node->cell.y + dy[i];

        if(distances.isCellInGrid(x, y) && distances(x, y) > params.minDistanceToObstacle){
            Node* childNode = new Node(x, y);
            children.push_back(childNode);
        }
    }
    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    // TODO: Generate path by following parent nodes
    std::vector<Node*> path;
    Node* curr_node = goal_node;

    while (!(*curr_node == *start_node))
    {
        path.push_back(curr_node);
        curr_node = curr_node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath){
    if(nodePath.size() < 3) return nodePath;

    std::vector<Node*> newPath;
    newPath.push_back(nodePath[0]);

    Node* prevNode, currNode, nextNode;
    int prev_dx, prev_dy, next_dx, next_dy;
    for(int i=1; i<nodePath.size()-1; i++){
        //dont add node if direction doesnt change
        prevNode = nodePath[i-1];
        currNode = nodePath[i];
        nextNode = nodePath[i+1];

        prev_dx = currNode->cell.x - prevNode->cell.x;
        prev_dy = currNode->cell.y - prevNode->cell.y;
        next_dx = nextNode->cell.x - currNode->cell.x;
        next_dy = nextNode->cell.y - currNode->cell.y;

        if(prev_dx != next_dx || prev_dy != next_dy){
            newPath.push_back(currNode);
        }
    }
    return newPath
}

// get poses from path
std::vector<mbot_lcm_msgs::pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    std::vector<mbot_lcm_msgs::pose_xyt_t> posePath;
    for(auto &&node : nodes){
        Point<double> global_pose = grid_position_to_global_position(node->cell, distances);
        mbot_lcm_msgs::pose_xyt_t currPose;
        currPose.x = global_pose.x;
        currPose.y = global_pose.y;

        if(posePath.size() == 0){
            currPose.theta = 0;
        }else{
            mbot_lcm_msgs::pose_xyt_t prevPose = posePath.back();
            currPose.theta = atan2(currPose.y -  prevPose.y, currPose.x - prevPose.x);
        }
    
        currPose.utime = 0;
        posePath.push_back(currPose);
    }
    return posePath;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;
    
}
