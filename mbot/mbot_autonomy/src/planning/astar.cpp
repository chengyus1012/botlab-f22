#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;
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
    std::vector<Node*> searchedList;

    startNode->g_cost = 0;
    startNode->h_cost = h_cost(startNode, goalNode, distances);

    openList.push(startNode);

    bool found_path = false;
    while(!openList.empty() && found_path == false)
    {

        Node* currentNode = openList.pop();
        if(!(*currentNode == *goalNode)){
            closedList.push_back(currentNode);
            expand_node(currentNode, goalNode, distances, params, openList, closedList, searchedList);
        }else{
            goalNode = currentNode;
            found_path = true;
        }
    }

    mbot_lcm_msgs::robot_path_t path;
    path.utime = start.utime;

    if(found_path){
        std::cout<<"openList len = "<<openList.elements.size()<<std::endl;
        std::vector<Node*> nodePath = extract_node_path(goalNode, startNode);
        std::cout<<"nodePath len = "<<nodePath.size()<<std::endl;
        std::vector<Node*> prunedNodePath = prune_node_path(nodePath);
        std::cout<<"pruned len = "<<prunedNodePath.size()<<std::endl;
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
    double diag_distance = 1.414;

    double h_cost = straight_distance*(dx + dy) + (diag_distance - 2*straight_distance) * std::min(dx, dy);
    h_cost *= distances.metersPerCell();
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

    g_cost = g_cost*distances.metersPerCell() + penalization;
    return g_cost;
}

void expand_node(Node* node, Node* goalNode, const ObstacleDistanceGrid& distances, const SearchParams& params, PriorityQueue& openList, std::vector<Node*>& closedList, std::vector<Node*>& searchedList)
{
    // TODO: Return children of a given node that are not obstacles
    int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    int dy[8] = {0, 0, 1, -1, 1, 1, -1, -1};

    for(int i=0; i<8; i++)
    {
        int x = node->cell.x + dx[i];
        int y = node->cell.y + dy[i];
        Node* neighbor = new Node(x,y);
        if(is_in_list(neighbor, searchedList))
            neighbor = get_from_list(neighbor, searchedList);
        
        if(!is_in_list(neighbor, closedList) && distances.isCellInGrid(x, y) && distances(x, y) > params.minDistanceToObstacle)
        {
            if(!is_in_list(neighbor, searchedList))
            {
                neighbor->g_cost = g_cost(node, neighbor, distances, params);
                neighbor->h_cost = h_cost(neighbor, goalNode, distances);
                neighbor->parent = node;
                openList.push(neighbor);
                searchedList.push_back(neighbor);
            }
            else if(neighbor->g_cost > g_cost(node, neighbor, distances, params))
            {
                neighbor->g_cost = g_cost(node, neighbor, distances, params);
                neighbor->parent = node;
                openList.push(neighbor);
            }
        }
    }
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    // TODO: Generate path by following parent nodes
    std::vector<Node*> path;
    Node* curr_node = goal_node;

    while (!(*curr_node == *start_node)){
        
        path.push_back(curr_node);
        curr_node = curr_node->parent;
        // std::cout << " found parent"<<std::endl;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath){
    if(nodePath.size() < 3) return nodePath;

    std::vector<Node*> newPath;
    newPath.push_back(nodePath[0]);

    Node* prevNode = NULL;
    Node* currNode = NULL;
    Node* nextNode = NULL;
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

    newPath.push_back(nodePath.back());
    return newPath;
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
