/** @file ManipulatorOptions.hpp
 *
 * Defines the ManipulatorOptions class, which specifies the configuration settings available for the
 * Manipulator problem.
 */
#ifndef MANIPULATOR_OPTIONS_HPP_
#define MANIPULATOR_OPTIONS_HPP_

#include <string>                       // for string

#include <iostream>

#include "SharedOptions.hpp"

using std::cout;
using std::endl;

namespace manipulator {
/** A class defining the configuration settings for the Manipulator problem. */
struct ManipulatorOptions : public shared::SharedOptions {
    ManipulatorOptions() = default;
    virtual ~ManipulatorOptions() = default;
    
    double exitReward = 1000.0;
    
    double illegalMovePenalty = 500.0;

    double illegalActionPenalty = 500.0;    

    double stepPenalty = 1.0;

    double process_covariance = 0.5;

    double observation_covariance = 0.01;
    
    int num_input_steps = 3;
    
    double control_rate = 0.0;
    
    size_t fixedActionResolution = 0;

    bool verbose_rrt = false;
    
    double stretching_factor = 1.0;
    
    int trajopt_points = 50;
    
    double contact_distance = 0.25;
    
    double penalty_distance = 0.0001; 

    bool use_trajopt = false;
    
    std::string planner = "RRTConnect";
    
    std::string dynamic_planner = "RRT";
    
    bool continuous_collision_check = false;
    
    bool check_linear_path = false;
    
    bool enforce_constraints = false;
    
    std::vector<double> init_state = std::vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    
    bool dynamic_problem = false;
    
    std::string dynamicModel = "lagrange";
    
    bool show_viewer = false;
    
    double simulation_step_size = 0.001;
    
    double control_duration = 0.0333;
    
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
    
    std::string terrains_path = EXPAND_AND_QUOTE(ROOT_PATH) "/problems/manipulator_discrete/terrains";

    std::string env_path = "/home/marcus/PhD/scripts/abt/problems/manipulator_discrete/environment/env_4dof.xml";

    std::string robot_path = "/home/marcus/PhD/scripts/abt/problems/manipulator_discrete/model/test_4dof.urdf";
    
    std::string logPath = "/home/marcus/PhD/scripts/lqg_test/build/log.log";
    
    std::string policyPath = "";
    
    std::string goal_states_path = "/home/marcus/PhD/scripts/abt/problems/manipulator_discrete/goalstates.txt";
    
    //for HFR
    
    double path_deviation_cost = 500.0;
    
    double control_deviation_cost = 1.0;
    
    unsigned int numEvaluationSamples = 200;
    
    //RRT options
    unsigned int numControlSamples = 1;
    
    double rrtGoalBias = 0.05;
    
    std::vector<int> min_max_control_duration = std::vector<int>({1, 4});
    
    std::string control_sampler = "discrete";
    
    bool add_intermediate_states = true;
    
    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {        
        std::unique_ptr<options::OptionParser> parser = SharedOptions::makeParser(simulating,
                EXPAND_AND_QUOTE(ROOT_PATH) "/problems/manipulator_discrete");
        addManipulatorOptions(parser.get());
        //addHeuristicOptions(parser.get());
        return std::move(parser);
    }

    /** Adds the core configuration settings for the RpckSample problem to the given parser. */
    static void addManipulatorOptions(options::OptionParser *parser) {
    	parser->addOption<bool>("problem", "dynamicProblem", &ManipulatorOptions::dynamic_problem);
        parser->addOption<double>("problem", "exitReward", &ManipulatorOptions::exitReward);
        parser->addOption<double>("problem", "illegalMovePenalty", &ManipulatorOptions::illegalMovePenalty);
        parser->addOption<double>("problem", "illegalActionPenalty", &ManipulatorOptions::illegalActionPenalty);
        parser->addOption<double>("problem", "stepPenalty", &ManipulatorOptions::stepPenalty);
        parser->addOption<double>("problem", "process_covariance", &ManipulatorOptions::process_covariance);
        parser->addOption<double>("problem", "observation_covariance", &ManipulatorOptions::observation_covariance);        
        parser->addOption<int>("problem", "num_input_steps", &ManipulatorOptions::num_input_steps);
        parser->addOption<double>("problem", "control_rate", &ManipulatorOptions::control_rate);        
        parser->addOption<std::vector<double>>("problem", "init_state", &ManipulatorOptions::init_state);
        parser->addOption<bool>("problem", "enforce_constraints", &ManipulatorOptions::enforce_constraints);               
        parser->addOption<bool>("RRT", "rrt_verb", &ManipulatorOptions::verbose_rrt);
        parser->addOption<double>("RRT", "stretching_factor", &ManipulatorOptions::stretching_factor);        
        parser->addOption<std::string>("RRT", "planner", &ManipulatorOptions::planner);
        parser->addOption<std::string>("RRT", "dynamicPlanner", &ManipulatorOptions::dynamic_planner);
        parser->addOption<std::string>("RRT", "controlSampler", &ManipulatorOptions::control_sampler);
        parser->addOption<bool>("RRT", "continuous_collision_check", &ManipulatorOptions::continuous_collision_check);
        parser->addOption<bool>("RRT", "check_linear_path", &ManipulatorOptions::check_linear_path);
        parser->addOption<double>("RRT", "planning_velocity", &ManipulatorOptions::planning_velocity);
        parser->addOption<int>("TRAJOPT", "trajopt_points", &ManipulatorOptions::trajopt_points);
        parser->addOption<bool>("TRAJOPT", "use_trajopt", &ManipulatorOptions::use_trajopt);
        parser->addOption<double>("TRAJOPT", "contact_distance", &ManipulatorOptions::contact_distance);
        parser->addOption<double>("TRAJOPT", "penalty_distance", &ManipulatorOptions::penalty_distance);        
        parser->addOption<double>("DYNAMICS", "simulation_step_size", &ManipulatorOptions::simulation_step_size);
        parser->addOption<double>("DYNAMICS", "gravity", &ManipulatorOptions::gravityConstant);
        parser->addOption<bool>("simulation", "showViewer", &ManipulatorOptions::show_viewer);
        parser->addOption<double>("ABT", "maxObservationDistance", &ManipulatorOptions::max_observation_distance);
        parser->addOption<double>("ABT", "particleReplenishTimeout", &ManipulatorOptions::particle_replenish_timeout);
        parser->addOption<std::string>("ABT", "stateSamplingStrategy", &ManipulatorOptions::stateSamplingStrategy);
        parser->addOption<std::string>("ABT", "particleFilter", &ManipulatorOptions::particleFilter);
        parser->addOption<unsigned int>("ABT", "numSamplesWeightedLazy", &ManipulatorOptions::numSamplesWeightedLazy);
        parser->addOption<std::string>("problem", "inc_covariance", &ManipulatorOptions::inc_covariance);
        parser->addOption<unsigned int>("ABT", "particlePlotLimit", &ManipulatorOptions::particle_plot_limit);
        parser->addOption<unsigned int>("ABT", "numEffectiveParticles", &ManipulatorOptions::num_effective_particles);
        parser->addOption<std::string>("problem", "environment_path", &ManipulatorOptions::env_path);
        parser->addOption<std::string>("problem", "robot_path", &ManipulatorOptions::robot_path);
        parser->addOption<std::string>("problem", "goal_states_path", &ManipulatorOptions::goal_states_path);
        parser->addOption<std::string>("problem", "logPath", &ManipulatorOptions::logPath);
        parser->addOption<std::string>("problem", "policyPath", &ManipulatorOptions::policyPath);
        parser->addOption<std::string>("problem", "dynamicModel", &ManipulatorOptions::dynamicModel);
        parser->addOption<double>("problem", "pathDeviationCost", &ManipulatorOptions::path_deviation_cost);
        parser->addOption<double>("problem", "controlDeviationCost", &ManipulatorOptions::path_deviation_cost);  
        parser->addOption<unsigned int>("problem", "numEvaluationSamples", &ManipulatorOptions::numEvaluationSamples);
        parser->addOption<double>("problem", "controlDuration", &ManipulatorOptions::control_duration); 
        
        //env_path = EXPAND_AND_QUOTE(ROOT_PATH) "/problems/manipulator_discrete/environment/" + ManipulatorOptions::environment_file;
        //robot_path = EXPAND_AND_QUOTE(ROOT_PATH) "/problems/manipulator_discrete/model/" + ManipulatorOptions::robot_file;
    }
    
};
} /* namespace Manipulator */

#endif /* Manipulator_OPTIONS_HPP_ */
