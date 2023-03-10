#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation(0.5, 0.9, numParticles), // 0.5, 0.9
  distribution_quality(1),
  quality_reinvigoration_percentage(0.25),
  random_initialized(false)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0/kNumParticles_;

    posteriorPose_ = pose;
    std::cout<<" initial pose.\n"<<" "<<pose.x << " "<< pose.y << " "<< pose.theta << std::endl;
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
    random_initialized = true;

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
    // std::cout<<"has moved: "<<hasRobotMoved << std::endl;
    if(hasRobotMoved){
        auto start = std::chrono::high_resolution_clock::now();
        auto prior = resamplePosteriorDistribution(&map);
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        // reinvigoration step
        samplingAugmentation.insert_average_weight(cur_avg_weight);
        posteriorPose_ = estimatePosteriorPose(posterior_);
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        printf("%lld\n", microseconds);
    }
    
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
    int Nrein;
    if(random_initialized)
        Nrein = (int)kNumParticles_*quality_reinvigoration_percentage;
    else
        Nrein = 0;
    ParticleList prior = posterior_;

    // for(int i=0; i<kNumParticles_;i++)
    // {
    //     temp += posterior_[i].weight * posterior_[i].weight;
    // }
    // Neff = 1.0/temp;
    // if(Neff > (int)kNumParticles_)
    // {
    //     std::cout << " Effective particles: "<< Neff<< " thre: "<< (int)kNumParticles_*0.95<<std::endl;
    // }
    // else
    // {
    std::cout << "Low variance resampling.\n";
    // double r = (double) rand()/RAND_MAX/kNumParticles_;
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0 / kNumParticles_);
    double r = dist(generator);
    int count = 0;
    double c = posterior_[0].weight;
    
    // do low variance resample
    for(int i=0; i<kNumParticles_;i++)
    {
        double u = r + (double)i*1.0/kNumParticles_;
        while (u>c)
        {
            count++;
            c += posterior_[count].weight;
        }
        prior[i] = posterior_[count];
        prior[i].weight = sampleWeight;
    }
        // randomPoseGen.update_map(map);
        // std::random_shuffle(prior.begin(), prior.end());
        // for(int i=0; i<Nrein; i++)
        // {
        //     prior[i] = randomPoseGen.get_particle();
        //     prior[i].weight = sampleWeight;
        // }
        
    // }
    
    if(samplingAugmentation.sample_randomly()){
        randomPoseGen.update_map(map);
        std::random_shuffle(prior.begin(), prior.end());
        for(int i=0; i<Nrein; i++){
            prior[i] = randomPoseGen.get_particle();
            prior[i].weight = sampleWeight;
        }
        std::cout << "reinvigorate.\n";
    }
    // std::cout << "Particles resampled\n";    


    return prior;
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;

    for(auto& p : prior){
        proposal.push_back(actionModel_.applyAction(p));
    }
    // std::cout << "Apply action.\n";
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
    for(auto &p : proposal){
        mbot_lcm_msgs::particle_t weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        // std::cout<<"weight: "<<weighted.weight << std::endl;
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }

    for(auto &p : posterior){
        cur_avg_weight += p.weight/kNumParticles_;
        p.weight /= sumWeights;
        
        // if(!avg_w_initialized)
        //     avg_w_initialized = true;
        // else
        //     cur_avg_weight = 0.1*prev_avg_weight + 0.9*cur_avg_weight;
    }
    // std::cout << "Posterior normalized.\n";
    return posterior;
}

struct particle_t_comparator {
  bool operator() (mbot_lcm_msgs::particle_t i,mbot_lcm_msgs::particle_t j) { return (i.weight > j.weight);}
};

mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    mbot_lcm_msgs::pose_xyt_t pose;

    double percentage = 0.05; // 0.2
    particle_t_comparator comparator;
    ParticleList posterior_sorted = posterior;
    std::sort(posterior_sorted.begin(), posterior_sorted.end(), comparator);

    // std::cout<<" some simple test" << (posterior_sorted[0].weight >= posterior_sorted[1].weight) << std::endl;

    double xMean = 0.0;
    double yMean = 0.0;
    double cosThetaMean = 0.0;
    double sinThetaMean = 0.0;
    double totalWeight = 0.0;
    int idx = 0;
    // using best percentage particles
    for(auto& p : posterior_sorted){
        if (idx > static_cast<int>(posterior_sorted.size() * percentage))
            break;
        xMean +=p.weight * p.pose.x;
        yMean +=p.weight * p.pose.y;
        cosThetaMean += p.weight*std::cos(p.pose.theta);
        sinThetaMean += p.weight*std::sin(p.pose.theta);
        totalWeight += p.weight;
        idx++;
    }

    pose.x = xMean / totalWeight;
    pose.y = yMean / totalWeight;
    pose.theta = std::atan2(sinThetaMean, cosThetaMean);
    
    std::cout << "Mean pose calculated.\n"<<" "<<pose.x<<" " << pose.y <<" "<< pose.theta << std::endl;
    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    //////// TODO: Implement your method for computing the average of a pose distribution
    mbot_lcm_msgs::pose_xyt_t avg_pose;
    return avg_pose;
}
