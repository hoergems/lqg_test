#ifndef __UTILS__HPP_
#define __UTILS__HPP_
#include <path_planner/dynamic_path_planner.hpp>
#include <kalman_filter/path_evaluator.hpp>
#include <robot_environment/robot_environment.hpp>

namespace utils {

template<class OptionsType>
Eigen::MatrixXd getC(std::shared_ptr<shared::RobotEnvironment> &robot_environment,
		             std::shared_ptr<OptionsType> &options) {
	int state_space_dimension = robot_environment->getRobot()->getStateSpaceDimension();
	Eigen::MatrixXd C = Eigen::MatrixXd::Identity(state_space_dimension, state_space_dimension);
	C = options->path_deviation_cost * C;
	return C;
}

template<class OptionsType>
Eigen::MatrixXd getD(std::shared_ptr<shared::RobotEnvironment> &robot_environment,
                     std::shared_ptr<OptionsType> &options) {
	int control_space_dimension = robot_environment->getRobot()->getControlSpaceDimension();
	Eigen::MatrixXd D = Eigen::MatrixXd::Identity(control_space_dimension, control_space_dimension);
	D = options->control_deviation_cost * D;	
	return D;
}

template<class OptionsType>
std::shared_ptr<shared::DynamicPathPlanner> makeDynamicPathPlanner(std::shared_ptr<shared::RobotEnvironment> &robot_environment,
		                                                           std::shared_ptr<OptionsType> &options) {
	std::shared_ptr<shared::DynamicPathPlanner> dynamic_path_planner = std::make_shared<shared::DynamicPathPlanner>(false);
	dynamic_path_planner->setup(robot_environment, options->dynamic_planner);
	
	std::vector<double> goal_area;
	robot_environment->getGoalArea(goal_area);
	std::vector<double> goal_position({goal_area[0], goal_area[1], goal_area[2]});
	double goal_radius = goal_area[3];
	std::vector<std::vector<double>> goal_states = robot_environment->loadGoalStatesFromFile(options->goal_states_path);
	ompl::base::GoalPtr goal_region = 
				shared::makeManipulatorGoalRegion(dynamic_path_planner->getSpaceInformation(),
				                                  robot_environment,
				                                  goal_states,
				                                  goal_position,
				                                  goal_radius);
	dynamic_path_planner->setGoal(goal_region);	
	dynamic_path_planner->setControlSampler(options->control_sampler);
	dynamic_path_planner->addIntermediateStates(true);
	dynamic_path_planner->setNumControlSamples(options->numControlSamples);
	dynamic_path_planner->setRRTGoalBias(options->rrtGoalBias);
	dynamic_path_planner->setMinMaxControlDuration(options->min_max_control_duration);
	return dynamic_path_planner;
}

template<class OptionsType>
std::shared_ptr<shared::PathEvaluator> makePathEvaluator(std::shared_ptr<shared::RobotEnvironment> &robot_environment,
		                                                 std::shared_ptr<OptionsType> &options) {
	std::shared_ptr<shared::PathEvaluator> path_evaluator = std::make_shared<shared::PathEvaluator>();
	path_evaluator->setRobotEnvironment(robot_environment);
	path_evaluator->setRewardModel(options->stepPenalty, 
			                       options->illegalMovePenalty, 
			                       options->exitReward, 
			                       options->discountFactor);
	path_evaluator->setNumSamples(options->numEvaluationSamples);
	Eigen::MatrixXd C = getC<OptionsType>(robot_environment, options);
	Eigen::MatrixXd D = getD<OptionsType>(robot_environment, options);
	path_evaluator->setC(C);
	path_evaluator->setD(D);
	return path_evaluator;
}

template<class OptionsType>
std::shared_ptr<shared::RobotEnvironment> makeRobotEnvironment(std::shared_ptr<OptionsType> &options) {
	std::shared_ptr<shared::RobotEnvironment> robot_environment = std::make_shared<shared::RobotEnvironment>();
	bool created = robot_environment->createManipulatorRobot(options->robot_path);
	bool loaded_environment = robot_environment->loadEnvironment(options->env_path);
	assert(created == true && "Robot couldn't be loaded");
	assert(loaded_environment == true && "Environment couldn't be loaded");
	std::vector<double> goal_area;
	robot_environment->getGoalArea(goal_area);
	std::vector<double> goal_position({goal_area[0], goal_area[1], goal_area[2]});
	double goal_radius = goal_area[3];	
	
	robot_environment->getRobot()->setGoalArea(goal_position, goal_radius);
	robot_environment->setControlDuration(options->control_duration);
	robot_environment->setSimulationStepSize(options->simulation_step_size);
	return robot_environment;
}

}

#endif