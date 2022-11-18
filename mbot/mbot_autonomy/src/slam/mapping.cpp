#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono> 
using namespace std::chrono; 

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose_xyt_t& pose,
                        OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if(!initialized_){
        previousPose_ = pose;
    }

    MovingLaserScan movingscan(scan, previousPose_, pose);

    for(auto& ray : movingscan){
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }

    initialized_ = true;
    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your endpoint score ///////////////////////
    if(ray.range <= kMaxLaserDistance_){
        Point<float> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayCell;
        rayCell.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayCell.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y); 
        if(map.isCellInGrid(rayCell.x, rayCell.y)){
            increaseCellOdds(rayCell.x, rayCell.y);
        }
    }
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your ray score ///////////////////////
    if(ray.range <= kMaxLaserDistance_)
        {
            std::vector<Point<int>> points = bresenham(ray,map);
        }
    }

    for(auto& point : points){
        if(map.isCellInGrid(point.x, point.y)){
            decreaseCellOdds(point.x, point.y);
        }
    }
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    // Get global positions 
    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );

    // Cells
    Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
    Point<int> end_cell;
    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);
    std::vector<Point<int>> cells_touched;
    //////////////// TODO: Implement Bresenham's Algorithm ////////////////
    int dx, dy, sx, sy, err, x, y;
    float e2;    

    dx = std::abs(end_cell.x - start_cell.x);
    dy = std::abs(end_cell.y - start_cell.y);

    sx = start_cell.x < end_cell.x ? 1 : -1;
    sy = start_cell.y < end_cell.y ? 1 : -1;
    err = dx - dy;
    x = start_cell.x;
    y = start_cell.y;

    while(x!=end_cell.x || y!=end_cell.y)
    {
        e2 = 2 * err;
        if(e2 >= -dy)
        {
            err -= dy;
            x += sx;
        }
        if(e2 <= dx)
        {
            err += dy;
            y += sy;
        }
        cells_touched.push_back({x,y});
    }

    return cells_touched;
}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map){
    if(std::numeric_limits::<CellOdds>::max() - kHitOdds_ > map(x,y)){
        map(x,y) += kHitOdds_;
    }
    else{
        map(x,y) = std::numeric_limits::<CellOdds>::max();
    }
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map){
    if(std::numeric_limits::<CellOdds>::min() + kMissOdds_ < map(x,y)){
        map(x,y) -= kMissOdds_;
    }
    else{
        map(x,y) = std::numeric_limits::<CellOdds>::min();
    }
}