#ifndef _HFR_HPP_
#define _HFR_HPP_
#include <robot_environment/robot_environment.hpp>
#include <path_planner/dynamic_path_planner.hpp>
#include <path_planner/Options.hpp>
#include <kalman_filter/path_evaluator.hpp>
#include <Eigen/Dense>

namespace shared {

class HFR{
public:
	HFR();
	
	Eigen::MatrixXd getC();
	
	Eigen::MatrixXd getD();
	
	Eigen::MatrixXd build_covariance_matrix(std::string covariance_type, double &error);
	
private:
	std::shared_ptr<shared::RobotEnvironment> robot_environment_;
	
	std::shared_ptr<shared::PathEvaluator> path_evaluator_;
	
	double path_deviation_cost_;
	
	double control_deviation_cost_;
	
	double step_penalty_;
	
	double terminal_reward_;
	
	double illegal_move_penalty_;
	
	double discount_factor_;
	
	unsigned int num_evaluation_samples_;
	
	unsigned int num_control_samples_;
	
	double rrt_goal_bias_;
	
	double process_error_;
	
	double observation_error_;
	
	std::vector<int> min_max_control_duration_;
	
	std::shared_ptr<shared::EigenMultivariateNormal<double>> process_distribution_;
	    
	std::shared_ptr<shared::EigenMultivariateNormal<double>> observation_distribution_;
	
};

}

#endif