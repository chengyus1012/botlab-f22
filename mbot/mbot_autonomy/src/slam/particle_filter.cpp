#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0/kNumParticles_;

    posteriorPose_ = pose;
    std::cout<<" initial pose.\n"<<" "<<pose.x << pose.y << pose.theta << std::endl;
    for(auto &&p : posterior_){
        p.pose.x = pose.x;
        p.pose.y = pose.y;
        p.pose.theta = pose.theta;
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0/kNumParticles_;

    // posteriorPose_ = pose;

    for(auto &&p : posterior_){
        randomPoseGen.update_map(&map);
        p = randomPoseGen.get_particle();
        p.weight = sampleWeight;
    }

}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose_xyt_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    std::cout<<"has moved: "<<hasRobotMoved << std::endl;
    auto prior = resamplePosteriorDistribution(&map);
    auto proposal = computeProposalDistribution(prior);
    posterior_ = computeNormalizedPosterior(proposal, laser, map);
    // reinvigoration step
    samplingAugmentation.insert_average_weight(cur_avg_weight);
    posteriorPose_ = estimatePosteriorPose(posterior_);
    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}



mbot_lcm_msgs::pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid* map)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    double Neff;
    double temp = 0;
    double sampleWeight = 1.0/kNumParticles_;
    int Nrein = (int)kNumParticles_*quality_reinvigoration_percentage;
    ParticleList prior = posterior_;

    for(int i=0; i<kNumParticles_;i++){
        temp += posterior_[i].weight * posterior_[i].weight;
    }
    Neff = 1.0/temp;
    if(Neff > kNumParticles_/2)
        std::cout << "Enough effective particles.\n";
    else{
        
        double r = (double) rand()/RAND_MAX/kNumParticles_;
        int count = 0;
        std::random_device rd;
        std::mt19937 generator(rd());
        std::normal_distribution<> dist(0.0, 0.04);
        std::vector<double> cumWeight;
        cumWeight[0] = prior[0].weight;
        
        for(int i=1; i<kNumParticles_; i++){
            cumWeight[i] = cumWeight[i-1] + prior[i].weight;
        }
        // do low variance resample
        for(int i=0; i<kNumParticles_;i++){
            double u = r + i/kNumParticles_;
            while (u>cumWeight[count])
                count++;
            prior[i] = posterior_[count];
            prior[i].weight = sampleWeight;
            // p.pose.x = posteriorPose_.x + dist(generator);
            // p.pose.y = posteriorPose_.y + dist(generator);
            // p.pose.theta = posteriorPose_.theta + dist(generator);
            // p.pose.utime = posteriorPose_.utime;
            // p.parent_pose = posteriorPose_;
        }
        std::cout << "Low variance resampling.\n";
    }
    
    if(avg_w_initialized){
        if(samplingAugmentation.sample_randomly()){
            randomPoseGen.update_map(map);
            std::random_shuffle(prior.begin(), prior.end());
            for(int i=0; i<Nrein; i++){
                prior[i] = randomPoseGen.get_particle();
                prior[i].weight = sampleWeight;
            }
            std::cout << "reinvigorate.\n";
        }
    }
    std::cout << "Particles resampled\n";    


    return prior;
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;

    for(auto& p : prior){
        proposal.push_back(actionModel_.applyAction(p));
    }
    std::cout << "Apply action.\n";
    return proposal;
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    ParticleList posterior;
    double sumWeights = 0.0;
    prev_avg_weight = cur_avg_weight;
    cur_avg_weight = 0.0;
    for(auto &&p : proposal){
        mbot_lcm_msgs::particle_t weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        // std::cout<<"weight: "<<weighted.weight << std::endl;
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }

    for(auto &&p : posterior){
        p.weight /= sumWeights;
        cur_avg_weight += p.weight/kNumParticles_;
        if(!avg_w_initialized)
            avg_w_initialized = true;
        else
            cur_avg_weight = 0.1*prev_avg_weight + 0.9*cur_avg_weight;
    }
    std::cout << "Posterior normalized.\n";
    return posterior;
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    mbot_lcm_msgs::pose_xyt_t pose;
    double xMean = 0.0;
    double yMean = 0.0;
    double cosThetaMean = 0.0;
    double sinThetaMean = 0.0;

    for(auto& p : posterior){
        xMean +=p.weight * p.pose.x;
        yMean +=p.weight * p.pose.y;
        cosThetaMean += p.weight*std::cos(p.pose.theta);
        sinThetaMean += p.weight*std::sin(p.pose.theta);
    }

    pose.x = xMean;
    pose.y = yMean;
    pose.theta = std::atan2(sinThetaMean, cosThetaMean);
    
    std::cout << "Mean pose calculated.\n"<<" "<<pose.x << pose.y << pose.theta << std::endl;
    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    //////// TODO: Implement your method for computing the average of a pose distribution
    mbot_lcm_msgs::pose_xyt_t avg_pose;
    return avg_pose;
}
