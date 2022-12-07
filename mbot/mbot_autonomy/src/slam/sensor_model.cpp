#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   ray_stride_(1)
{
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    double likelihood = 0.0;
    double fraction_m = 0.35;
    double fraction_bf = 0.325;
    int count = 0;

    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);
    
    for(auto &ray : movingScan){
        Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta), ray.origin.y + ray.range * std::sin(ray.theta));

        Point<int> rayEnd = global_position_to_grid_cell(endpoint, map);
        Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);

        int dx = abs(rayEnd.x - rayStart.x);
        int dy = abs(rayEnd.y - rayStart.y);

        int sx = (rayStart.x < rayEnd.x) ? 1 : -1;
        int sy = (rayStart.y < rayEnd.y) ? 1 : -1;

        int err = dx - dy;
        int x = rayStart.x;
        int y = rayStart.y;

        bool flag = true;

        while ((x != rayEnd.x || y != rayEnd.y) && map.isCellInGrid(x, y)) {
            if(map.logOdds(x,y)>0){
                flag = false;
                break;
            }
            int e2 = 2 * err;
            if (e2 >= -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y += sy;
            }
        }

        if(flag==true){
            CellOdds rayCost_m = map.logOdds(rayEnd.x, rayEnd.y);
            CellOdds rayCost_b = map.logOdds(rayEnd.x-sx, rayEnd.y-sy);
            CellOdds rayCost_f = map.logOdds(rayEnd.x+sx, rayEnd.y+sy);
            if(rayCost_m>0)
            {
                likelihood += fraction_m*rayCost_m;
                count++;
            }else
            {
                if(rayCost_b > 0){
                    likelihood += fraction_bf * rayCost_b;
                    count++;}
                else if(rayCost_f > 0){
                    likelihood += fraction_bf * rayCost_f;
                    count++;}
            }
        }

        
        
    }
    if(count >= movingScan.size()*0.98){
        likelihood *= 15.0;
    }
    
    return likelihood;
}