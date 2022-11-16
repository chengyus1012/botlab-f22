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
    double fraction_m = 0.8;
    double fraction_bf = 0.1;

    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);
    
    for(auto &ray : movingScan){
        Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta), ray.origin.y + ray.range * std::sin(ray.theta));

        auto rayEnd = global_position_to_grid_cell(endpoint, map);
        auto rayCost_m = map.logOdds(rayEnd.x, rayEnd.y);
        auto rayCost_b = map.logOdds(rayEnd.x-1, rayEnd.y-1);
        auto rayCost_f = map.logOdds(rayEnd.x+1, rayEnd.y+1);


        if(rayCost_m>0){
            likelihood += fraction_m * rayCost_m;
        }
        if(rayCost_b > 0)
            likelihood += fraction_bf * rayCost_b;
        if(rayCost_f > 0)
            likelihood += fraction_bf * rayCost_f;
    }
    return likelihood;
}