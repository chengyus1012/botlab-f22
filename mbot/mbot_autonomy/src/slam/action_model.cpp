#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.01f)
, k2_(0.01f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if (!initialized_)
    {
        previousPose_ = odometry;
        initialized_ = true;
    }

    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    dtheta_ = angle_diff(odometry.theta, previousPose_.theta);

    bool moved = (dx_!=0 || dy_!=0 || dtheta_!=0);

    xStd_ = sqrt(k1_ * fabs(max(dx_, min_dist_)));
    yStd_ = sqrt(k1 * fabs(max(dy_, min_dist_)));
    thetaStd_ = sqrt(k2 * fabs(max(dtheta_, min_theta_)));

    utime_ = odometry.utime;
    previousPose_ = odometry;

    return moved;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;

    std::normal_distribution<float> x(dx_, xStd_);
    std::normal_distribution<float> y(dy_, yStd_);
    std::normal_distribution<float> theta(dtheta_, thetaStd_);

    float sampleX = x(numberGenerator_);
    float sampleY = y(numberGenerator_);
    float sampleTheta = theta(numberGenerator_);

    newSample.pose.x += sampleX;
    newSample.pose.y += sampleY;
    newSample.pose.theta = wrap_to_2pi(sample.pose.theta + sampleTheta);
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;
    return newSample;
}
