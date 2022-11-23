#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.0005f) //rotation
, k2_(0.0005f) //translation
, min_dist_(0.0) // 0.0025
, min_theta_(0.0) //0.02
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
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
    // use odometry motion model in probabilistic robotics 5.4
    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    dtheta_ = angle_diff(odometry.theta, previousPose_.theta);
    float direction = 1.0;

    trans_ = std::sqrt(dx_*dx_ + dy_*dy_);
    rot1_ = angle_diff(std::atan2(dy_, dx_), previousPose_.theta);

    if(std::abs(rot1_)>M_PI_2){
        rot1_ = angle_diff(M_PI, rot1_); // rot1 > 90 means it is moving at opposite direction
        direction = -1.0;
    }

    rot2_ = angle_diff(dtheta_, rot1_);

    bool moved = (dx_!=0 || dy_!=0 || dtheta_!=0);
    if(moved){
        rot1Std_ = std::sqrt(k1_ * std::max(std::abs(rot1_), min_theta_));
        tranStd_ = std::sqrt(k2_ * std::max(std::abs(trans_), min_dist_));
        rot2Std_ = std::sqrt(k1_ * std::max(std::abs(rot2_), min_theta_));
    }

    // xStd_ = sqrt(k1_ * fabs(std::max(dx_, min_dist_)));
    // yStd_ = sqrt(k1_ * fabs(std::max(dy_, min_dist_)));
    // thetaStd_ = sqrt(k2_ * fabs(std::max(dtheta_, min_theta_)));

    utime_ = odometry.utime;
    previousPose_ = odometry;
    trans_ *= direction;

    return moved;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;

    // std::normal_distribution<float> x(dx_, xStd_);
    // std::normal_distribution<float> y(dy_, yStd_);
    // std::normal_distribution<float> theta(dtheta_, thetaStd_);

    // float sampleX = x(numberGenerator_);
    // float sampleY = y(numberGenerator_);
    // float sampleTheta = theta(numberGenerator_);

    float sampleRot1 = std::normal_distribution<>(rot1_, rot1Std_)(numberGenerator_);
    float sampleRot2 = std::normal_distribution<>(rot2_, rot2Std_)(numberGenerator_);
    float sampleTrans = std::normal_distribution<>(trans_, tranStd_)(numberGenerator_);


    newSample.pose.x += sampleTrans * std::cos(sample.pose.theta + sampleRot1);
    newSample.pose.y += sampleTrans * std::sin(sample.pose.theta + sampleRot1);;
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampleRot1 + sampleRot2);
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;
    // std::cout<<"newSample pose"<<" "<<newSample.pose.x<<" " << newSample.pose.y<<" " << newSample.pose.theta << std::endl;

    return newSample;
}
