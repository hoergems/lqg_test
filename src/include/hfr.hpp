#ifndef _HFR_HPP_
#define _HFR_HPP_
#include <robot_environment/robot_environment.hpp>
#include <path_planner/dynamic_path_planner.hpp>
#include <kalman_filter/path_evaluator.hpp>
#include <Eigen/Dense>
#include "ManipulatorOptions.hpp"
#include "DubinOptions.hpp"
#include <boost/timer.hpp>

namespace shared {

class SimulationStepResult {
public:
	SimulationStepResult() {
		
	}
	
	std::vector<double> current_state;
	
	std::vector<double> action;
	
	std::vector<double> resulting_state;
	
	std::vector<double> observation;
	
	double reward;
	
	bool collided;
	
	bool terminal;
};

template<class RobotType, class OptionsType>
class HFR{
public:
	HFR(std::shared_ptr<OptionsType> &robot_options,
	    std::shared_ptr<shared::RobotEnvironment> &robot_environment,
	    std::shared_ptr<shared::PathEvaluator<RobotType, OptionsType>> &path_evaluator,
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
		unsigned int num_threads = 1;
		std::shared_ptr<shared::PathEvaluationResult> eval_res;		
		std::vector<double> current_state = options_->init_state;		
		utils::print_vector(current_state, "current_state");
		path_evaluator_->planAndEvaluatePaths(current_state,
				                              P_t,
				                              current_step,
				                              num_threads,
				                              eval_res);
		if (eval_res) {			
			std::vector<double> x_estimated = options_->init_state;
			//std::vector<double> u = eval_res->trajectory.us[0];
			std::vector<double> x_predicted = current_state;
			std::vector<double> x_predicted_temp;
			Eigen::MatrixXd P_predicted(P_t.rows(), P_t.cols());
			
			utils::print_vector(current_state, "current_state");
		    utils::print_vector(x_predicted, "x_predicted");
		    utils::print_vector(x_estimated, "x_estimate");
					
			while (true) {
				cout << "---------------------------------" << endl;
				//Get linear model matrices
				
				std::vector<Eigen::MatrixXd> A_B_V_H_W_M_N = path_evaluator_->getLinearModelMatricesState(x_estimated, 
						                                                                                  eval_res->trajectory.us[0],
						                                                                                  eval_res->trajectory.control_durations[0]);
				
				
				//Predict the next state
				//utils::print_vector(eval_res->trajectory.control_durations, "control durations");
				path_evaluator_->getKalmanFilter()->ekfPredictState(robot_environment_,
						                                            x_estimated,
						                                            eval_res->trajectory.us[0],
						                                            eval_res->trajectory.control_durations[0],
						                                            options_->simulation_step_size,
						                                            A_B_V_H_W_M_N[0],						                                            
						                                            A_B_V_H_W_M_N[2],
						                                            A_B_V_H_W_M_N[5],
						                                            P_t,
						                                            x_predicted_temp,
						                                            P_predicted);
				
				//check if x_predicted_collides
				bool p_collides = static_cast<shared::MotionValidator *>(dynamic_path_planner_->getMotionValidator().get())->collidesDiscrete(x_predicted_temp);
				if (p_collides) {
					x_predicted = x_estimated;
				}
				else {
					x_predicted = x_predicted_temp;
				}
				
				utils::print_vector(current_state, "current_state");
				utils::print_vector(x_predicted, "x_predicted");				
				utils::print_vector(eval_res->trajectory.xs[1], "xs[i + 1]");
				//Execute path for 1 step
				std::shared_ptr<shared::SimulationStepResult> simulation_step_result = simulateStep(current_state, eval_res->trajectory.us[0]);
				current_state = simulation_step_result->resulting_state;
				current_step += 1;	
				
				utils::print_vector(simulation_step_result->resulting_state, "res state");				
				
				if (simulation_step_result->terminal) {
					cout << "TERMINAL STATE REACHED" << endl;
					break;
				}
				
				//Plan new trajectories from predicted state
				std::shared_ptr<shared::PathEvaluationResult> eval_res_new;
				path_evaluator_->planAndEvaluatePaths(x_predicted,
								                      P_predicted,
								                      current_step,
								                      num_threads,
								                      eval_res_new);
				
				//Filter update
				std::vector<double> x_estimated_temp;
				path_evaluator_->getKalmanFilter()->kalmanUpdate(x_predicted,						     
						     simulation_step_result->observation, 
						     A_B_V_H_W_M_N[3],
						     P_predicted,
						     A_B_V_H_W_M_N[4],
						     A_B_V_H_W_M_N[6],
						     x_estimated_temp,
						     P_t);
				
				//Make sure x_estimated is okay
				robot_environment_->getRobot()->enforceConstraints(x_estimated_temp);
				bool e_collides = static_cast<shared::MotionValidator *>(dynamic_path_planner_->getMotionValidator().get())->collidesDiscrete(x_estimated_temp);
				if (e_collides) {
					std::vector<double> x_estimated_next;
					robot_environment_->getRobot()->makeNextStateAfterCollision(x_estimated, x_estimated_temp, x_estimated_next);
					x_estimated = x_estimated_next;
					utils::print_vector(x_estimated, "x_estimated after collision");
				}				
				else {
					x_estimated = x_estimated_temp;
				}
				utils::print_vector(x_estimated, "x_estimate");
				cout << "distance to goal: " << robot_environment_->getRobot()->distanceGoal(simulation_step_result->resulting_state) << endl;
				//Adjust plan
				std::shared_ptr<shared::PathEvaluationResult> eval_res_adjusted;				
				path_evaluator_->adjustAndEvaluatePath(eval_res->trajectory,
						                               x_estimated,
						                               P_t,
						                               current_step,
						                               eval_res_adjusted);
				// Determine the nominal path for the next step
				if (eval_res_adjusted->path_objective && eval_res_adjusted->path_objective > eval_res->path_objective) {
					eval_res = eval_res_adjusted;
					cout << "SWITCH TO ADJUSTED" << endl;
				}
				else {
					eval_res = eval_res_new;
				}
				
				
				cout << eval_res->path_objective << endl;
				//break;
			}
		}
		cout << "done " << endl; 
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
			                                                   std::vector<double> &action) {
		std::shared_ptr<shared::SimulationStepResult> result(new shared::SimulationStepResult);
		result->current_state = current_state;
		result->action = action;
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
		std::vector<double> next_state;
		
		//Check for collision
		bool collided = static_cast<shared::MotionValidator *>(dynamic_path_planner_->getMotionValidator().get())->collidesContinuous(current_state, propagation_result);
		result->collided = collided;
		
		//Check for terminal if we're not collided
		bool terminal = false;
		if (collided) {
			//We need to handle collisions here
			cout << "COLLIDED!" << endl;
			robot_environment_->getRobot()->makeNextStateAfterCollision(current_state, propagation_result, next_state);
		}
		else {
			next_state = propagation_result;
			terminal = robot_environment_->getRobot()->isTerminal(propagation_result);
		}
		
		//Generate an observation
		std::vector<double> observation;		
		robot_environment_->makeObservation(next_state, observation);
		result->resulting_state = next_state;
		result->observation = observation;
		result->terminal = terminal;
		
		return result;
	}
	
private:
	std::shared_ptr<shared::DynamicPathPlanner> dynamic_path_planner_;
	
	std::shared_ptr<OptionsType> options_;
	
	std::shared_ptr<shared::RobotEnvironment> robot_environment_;
	
	std::shared_ptr<shared::PathEvaluator<RobotType, OptionsType>> path_evaluator_;
	
};

}

#endif