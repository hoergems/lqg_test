/** @file DubinOptions.hpp
 *
 * Defines the DubinOptions class, which specifies the configuration settings available for the
 * Dubin problem.
 */
#ifndef DUBIN_OPTIONS_HPP_
#define DUBIN_OPTIONS_HPP_

#include <string>                       // for string

#include <iostream>

#include "SharedOptions.hpp"

using std::cout;
using std::endl;

namespace dubin {
/** A class defining the configuration settings for the Dubin problem. */
struct DubinOptions : public shared::SharedOptions {
    DubinOptions() = default;
    virtual ~DubinOptions() = default;
    
    double exitReward = 1000.0;
    
    double illegalMovePenalty = 500.0;

    double illegalActionPenalty = 500.0;    

    double stepPenalty = 1.0;

    double process_covariance = 0.0;

    double observation_covariance = 0.01;
    
    int num_input_steps = 3;
    
    double control_rate = 0.0;
    
    size_t fixedActionResolution = 0;

    bool verbose_rrt = false;
    
    double stretching_factor = 1.0;
    
    std::string planner = "RRTConnect";
    
    std::string dynamic_planner = "RRT";
    
    bool continuous_collision_check = false;
    
    bool check_linear_path = false;
    
    bool enforce_constraints = false;
    
    std::vector<double> init_state = std::vector<double>({0.0, 0.0, 0.0, 0.0});
    
    bool dynamic_problem = false;
    
    std::string dynamicModel = "lagrange";
    
    bool show_viewer = false;
    
    double simulation_step_size = 0.001;
    
    double control_duration = 0.333;
    
    double planning_velocity = 2.0;
    
    double max_observation_distance = 3.14;
    
    double particle_replenish_timeout = 10.0;

    std::string inc_covariance = "process";
    
    std::string stateSamplingStrategy = "default";
    
    unsigned int numSamplesWeightedLazy = 10;
    
    double gravityConstant = 9.81;
    
    unsigned int particle_plot_limit = 50;
    
    unsigned int num_effective_particles = 90;
    
    std::string particleFilter = "default";
    
    std::string terrains_path = EXPAND_AND_QUOTE(ROOT_PATH) "/problems/Dubin_discrete/terrains";

    std::string env_path = "/home/marcus/PhD/scripts/abt/problems/dubins_car/environment/env_dubin.xml";

    std::string robot_path = "/home/marcus/PhD/scripts/abt/problems/dubins_car/model/test_4dof.urdf";
    
    std::string logPath = "/home/marcus/PhD/scripts/lqg_test/build/log.log";
    
    std::string policyPath = "";
    
    std::string goal_states_path = "/home/marcus/PhD/scripts/lqg_test/goalstates_dubin.txt";
    
    //for HFR
    
    double path_deviation_cost = 500.0;
    
    double control_deviation_cost = 1.0;
    
    unsigned int numEvaluationSamples = 200;
    
    //RRT options
    unsigned int numControlSamples = 1;
    
    double rrtGoalBias = 0.05;
    
    std::vector<int> min_max_control_duration = std::vector<int>({1, 4});
    
    std::string control_sampler = "continuous";
    
    bool add_intermediate_states = true;
    
    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {        
        std::unique_ptr<options::OptionParser> parser = SharedOptions::makeParser(simulating,
                EXPAND_AND_QUOTE(ROOT_PATH) "/problems/Dubin_discrete");
        addDubinOptions(parser.get());
        //addHeuristicOptions(parser.get());
        return std::move(parser);
    }

    /** Adds the core configuration settings for the RpckSample problem to the given parser. */
    static void addDubinOptions(options::OptionParser *parser) {
    	parser->addOption<bool>("problem", "dynamicProblem", &DubinOptions::dynamic_problem);
        parser->addOption<double>("problem", "exitReward", &DubinOptions::exitReward);
        parser->addOption<double>("problem", "illegalMovePenalty", &DubinOptions::illegalMovePenalty);
        parser->addOption<double>("problem", "illegalActionPenalty", &DubinOptions::illegalActionPenalty);
        parser->addOption<double>("problem", "stepPenalty", &DubinOptions::stepPenalty);
        parser->addOption<double>("problem", "process_covariance", &DubinOptions::process_covariance);
        parser->addOption<double>("problem", "observation_covariance", &DubinOptions::observation_covariance);        
        parser->addOption<int>("problem", "num_input_steps", &DubinOptions::num_input_steps);
        parser->addOption<double>("problem", "control_rate", &DubinOptions::control_rate);        
        parser->addOption<std::vector<double>>("problem", "init_state", &DubinOptions::init_state);
        parser->addOption<bool>("problem", "enforce_constraints", &DubinOptions::enforce_constraints);               
        parser->addOption<bool>("RRT", "rrt_verb", &DubinOptions::verbose_rrt);
        parser->addOption<double>("RRT", "stretching_factor", &DubinOptions::stretching_factor);        
        parser->addOption<std::string>("RRT", "planner", &DubinOptions::planner);
        parser->addOption<std::string>("RRT", "dynamicPlanner", &DubinOptions::dynamic_planner);
        parser->addOption<std::string>("RRT", "controlSampler", &DubinOptions::control_sampler);
        parser->addOption<bool>("RRT", "continuous_collision_check", &DubinOptions::continuous_collision_check);
        parser->addOption<bool>("RRT", "check_linear_path", &DubinOptions::check_linear_path);
        parser->addOption<double>("RRT", "planning_velocity", &DubinOptions::planning_velocity);            
        parser->addOption<double>("DYNAMICS", "simulation_step_size", &DubinOptions::simulation_step_size);
        parser->addOption<double>("DYNAMICS", "gravity", &DubinOptions::gravityConstant);
        parser->addOption<bool>("simulation", "showViewer", &DubinOptions::show_viewer);
        parser->addOption<double>("ABT", "maxObservationDistance", &DubinOptions::max_observation_distance);
        parser->addOption<double>("ABT", "particleReplenishTimeout", &DubinOptions::particle_replenish_timeout);
        parser->addOption<std::string>("ABT", "stateSamplingStrategy", &DubinOptions::stateSamplingStrategy);
        parser->addOption<std::string>("ABT", "particleFilter", &DubinOptions::particleFilter);
        parser->addOption<unsigned int>("ABT", "numSamplesWeightedLazy", &DubinOptions::numSamplesWeightedLazy);
        parser->addOption<std::string>("problem", "inc_covariance", &DubinOptions::inc_covariance);
        parser->addOption<unsigned int>("ABT", "particlePlotLimit", &DubinOptions::particle_plot_limit);
        parser->addOption<unsigned int>("ABT", "numEffectiveParticles", &DubinOptions::num_effective_particles);
        parser->addOption<std::string>("problem", "environment_path", &DubinOptions::env_path);
        parser->addOption<std::string>("problem", "robot_path", &DubinOptions::robot_path);
        parser->addOption<std::string>("problem", "goal_states_path", &DubinOptions::goal_states_path);
        parser->addOption<std::string>("problem", "logPath", &DubinOptions::logPath);
        parser->addOption<std::string>("problem", "policyPath", &DubinOptions::policyPath);
        parser->addOption<std::string>("problem", "dynamicModel", &DubinOptions::dynamicModel);
        parser->addOption<double>("problem", "pathDeviationCost", &DubinOptions::path_deviation_cost);
        parser->addOption<double>("problem", "controlDeviationCost", &DubinOptions::path_deviation_cost);  
        parser->addOption<unsigned int>("problem", "numEvaluationSamples", &DubinOptions::numEvaluationSamples);
        parser->addOption<double>("problem", "controlDuration", &DubinOptions::control_duration); 
        
        //env_path = EXPAND_AND_QUOTE(ROOT_PATH) "/problems/Dubin_discrete/environment/" + DubinOptions::environment_file;
        //robot_path = EXPAND_AND_QUOTE(ROOT_PATH) "/problems/Dubin_discrete/model/" + DubinOptions::robot_file;
    }
    
};
} /* namespace Dubin */

#endif /* Dubin_OPTIONS_HPP_ */
