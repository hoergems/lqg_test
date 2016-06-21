#ifndef _HFR_HPP_
#define _HFR_HPP_
#include <robot_environment/robot_environment.hpp>
#include <path_planner/dynamic_path_planner.hpp>
#include <path_planner/Options.hpp>
#include <kalman_filter/path_evaluator.hpp>
#include <Eigen/Dense>
#include "ManipulatorOptions.hpp"

namespace shared {

class RobotOptions {
public:
	RobotOptions() {};
	
	double process_error;
	
	double path_deviation_cost;
	
	double simulation_step_size;
	
	double control_duration;
	
	std::string robot_path;
	
	std::string environment_path;
	
	
};

class ManipulatorOptions: public RobotOptions {
public:
	ManipulatorOptions():
	RobotOptions(){
		
	}
};

class SimulationStepResult {
public:
	SimulationStepResult() {
		
	}
	
	std::vector<double> resulting_state;
	
	double reward;
	
	bool collided;
};

template<class RobotType, class OptionsType>
class HFR{
public:
	HFR(std::shared_ptr<OptionsType> &robot_options):
		robot_environment_(new shared::RobotEnvironment()),
		path_evaluator_(new shared::PathEvaluator()),
		path_deviation_cost_(robot_options->path_deviation_cost),
		control_deviation_cost_(0.0),
		step_penalty_(0.0),	
		illegal_move_penalty_(0.0),
		terminal_reward_(0.0),
		discount_factor_(0.0),
		num_evaluation_samples_(0),
		num_control_samples_(0),
		rrt_goal_bias_(0.0),
		min_max_control_duration_(),
		process_error_(0.0),
		observation_error_(0.0),
		simulation_step_size_(0.0),
		process_distribution_(nullptr),			
		observation_distribution_(nullptr){
		
	}
	
	Eigen::MatrixXd getC() {
		int state_space_dimension = robot_environment_->getRobot()->getStateSpaceDimension();
		Eigen::MatrixXd C = Eigen::MatrixXd::Identity(state_space_dimension, state_space_dimension);
		C = path_deviation_cost_ * C;
		return C;
	}
	
	Eigen::MatrixXd getD() {
		int control_space_dimension = robot_environment_->getRobot()->getControlSpaceDimension();
		Eigen::MatrixXd D = Eigen::MatrixXd::Identity(control_space_dimension, control_space_dimension);
	    D = control_deviation_cost_ * D;	
		return D;
	}
	
	Eigen::MatrixXd build_covariance_matrix(std::string covariance_type, double &error) {
		if (covariance_type == "process") {
			Eigen::MatrixXd covariance_matrix = Eigen::MatrixXd::Identity(robot_environment_->getRobot()->getControlSpaceDimension(), 
							                                              robot_environment_->getRobot()->getControlSpaceDimension());
						
			std::vector<double> lowerControlLimits;
			std::vector<double> upperControlLimits;
			robot_environment_->getRobot()->getControlLimits(lowerControlLimits, upperControlLimits);
			for (size_t i = 0; i < lowerControlLimits.size(); i++) {
				double control_range = upperControlLimits[i] - lowerControlLimits[i];
				cout << "control range: " << control_range << endl;
				covariance_matrix(i, i) = std::pow((control_range / 100.0) * error, 2);
			}
			return covariance_matrix;
		}		
			
		else if (covariance_type == "observation") {
			Eigen::MatrixXd covariance_matrix = Eigen::MatrixXd::Identity(robot_environment_->getRobot()->getStateSpaceDimension(), 
						                                                  robot_environment_->getRobot()->getStateSpaceDimension());
			std::vector<double> lowerStateLimits;
			std::vector<double> upperStateLimits;
			robot_environment_->getRobot()->getStateLimits(lowerStateLimits, upperStateLimits);		
			for (size_t i = 0; i < lowerStateLimits.size(); i++) {
				double state_range = upperStateLimits[i] - lowerStateLimits[i];			
				covariance_matrix(i, i) = std::pow((state_range / 100.0) * error, 2);			
			}
			return covariance_matrix;
		}
	}
	
	std::shared_ptr<shared::SimulationStepResult> simulateStep(std::vector<double> &current_state,
			                                                   std::vector<double> &action,
			                                                   double &control_duration,
			                                                   std::vector<double> &estimated_state) {
		std::shared_ptr<shared::SimulationStepResult> result(new shared::SimulationStepResult);
			
		//Sample control error
		std::vector<double> control_error;
		Eigen::MatrixXd sample(action.size(), 1);
		robot_environment_->getProcessDistribution()->nextSample(sample);
		for (size_t i = 0; i < action.size(); i++) {
			control_error.push_back(sample(i));
		}
		/**robot_environment_->getRobot()->propagateState(current_state,
					                                       action,
					                                       control_error,
					                                       control_duration,
					                                       )*/
		return result;
	}
	
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
	
	double control_duration_;
	
	double simulation_step_size_;
	
	std::vector<int> min_max_control_duration_;
	
	std::shared_ptr<shared::EigenMultivariateNormal<double>> process_distribution_;
	    
	std::shared_ptr<shared::EigenMultivariateNormal<double>> observation_distribution_;
	
};

}

#endif