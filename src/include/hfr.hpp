#ifndef _HFR_HPP_
#define _HFR_HPP_
#include <robot_environment/robot_environment.hpp>
#include <path_planner/dynamic_path_planner.hpp>
#include <path_planner/Options.hpp>
#include <kalman_filter/path_evaluator.hpp>
#include <Eigen/Dense>
#include "ManipulatorOptions.hpp"

namespace shared {

class SimulationStepResult {
public:
	SimulationStepResult() {
		
	}
	
	std::vector<double> resulting_state;
	
	double reward;
	
	bool collided;
	
	bool terminal;
};

template<class RobotType, class OptionsType>
class HFR{
public:
	HFR(std::shared_ptr<OptionsType> &robot_options,
	    std::shared_ptr<shared::RobotEnvironment> &robot_environment,
	    std::shared_ptr<shared::PathEvaluator<OptionsType>> &path_evaluator,
	    std::shared_ptr<shared::DynamicPathPlanner> &dynamic_path_planner):
		options_(robot_options),
		robot_environment_(robot_environment),
		path_evaluator_(path_evaluator),		
		dynamic_path_planner_(dynamic_path_planner) {
		
	}
	
	void runSimulation(std::ofstream &os) {
		for (size_t i = 0; i < options_->nRuns; i++) {
			simulate(os);
		}
	}
	
	void simulate(std::ofstream &os) {
		bool canDoSimulation = true;
		
		Eigen::MatrixXd P_t = Eigen::MatrixXd::Zero(robot_environment_->getRobot()->getStateSpaceDimension(),
				                                    robot_environment_->getRobot()->getStateSpaceDimension());
		unsigned int current_step = 0;
		unsigned int num_threads = 3;
		std::shared_ptr<shared::PathEvaluationResult> res;
		path_evaluator_->planAndEvaluatePaths(options_->init_state,
				                              P_t,
				                              current_step,
				                              num_threads,
				                              res);
		if (res) {
			cout << res->path_objective << endl;
		}
		
		//while (canDoSimulation) {
		//	cout << "hello" << endl;
		//}
	}
	
	void setDynamicPathPlanner(std::shared_ptr<shared::DynamicPathPlanner> &dynamic_path_planner) {
		dynamic_path_planner_ = dynamic_path_planner;
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
			                                                   std::vector<double> &estimated_state) {
		std::shared_ptr<shared::SimulationStepResult> result(new shared::SimulationStepResult);
			
		//Sample control error
		std::vector<double> control_error;
		Eigen::MatrixXd sample(action.size(), 1);
		robot_environment_->getProcessDistribution()->nextSample(sample);
		for (size_t i = 0; i < action.size(); i++) {
			control_error.push_back(sample(i));
		}
		
		std::vector<double> propagation_result;
		
		robot_environment_->getRobot()->propagateState(current_state,
					                                   action,
					                                   control_error,
					                                   options_->control_duration,					                                   
					                                   options_->simulation_step_size,
					                                   propagation_result);
		//Check for collision
		bool collided = static_cast<shared::MotionValidator *>(dynamic_path_planner_->getMotionValidator().get())->collidesContinuous(current_state, propagation_result);
		result->collided = collided;
		
		//Check for terminal if we're not collided
		bool terminal = false;
		if (collided) {
			//We need to handle collisions here
		}
		else {
			terminal = robot_environment_->getRobot()->isTerminal(propagation_result);
		}
		
		result->resulting_state = propagation_result;
		result->terminal = terminal;
		
		return result;
	}
	
private:
	std::shared_ptr<shared::DynamicPathPlanner> dynamic_path_planner_;
	
	std::shared_ptr<OptionsType> options_;
	
	std::shared_ptr<shared::RobotEnvironment> robot_environment_;
	
	std::shared_ptr<shared::PathEvaluator<OptionsType>> path_evaluator_;
	
};

}

#endif