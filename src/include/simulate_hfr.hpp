#ifndef __SIMULATE_HFR_HPP_
#define __SIMULATE_HFR_HPP_
#include <fstream>                      // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, ofstream, endl, ostream, ifstream
#include <iostream>

#include "utils.hpp"
#include "ManipulatorOptions.hpp"
#include "hfr.hpp"

template<class RobotType, class OptionsType>
int simulate() {
	std::unique_ptr<options::OptionParser> parser = OptionsType::makeParser(true);
	OptionsType options;
	parser->setOptions(&options);
	
	std::shared_ptr<OptionsType> robot_options = std::make_shared<OptionsType>(options);	
	std::shared_ptr<shared::RobotEnvironment> robot_environment = utils::makeRobotEnvironment<OptionsType>(robot_options);	
	std::shared_ptr<shared::DynamicPathPlanner> dynamic_path_planner = utils::makeDynamicPathPlanner<OptionsType>(robot_environment, robot_options);
	std::shared_ptr<shared::PathEvaluator<OptionsType>> path_evaluator = utils::makePathEvaluator<OptionsType>(robot_environment, robot_options);
	
	//std::shared_ptr<shared::HFR<RobotType, RobotType>> hfr = std::make_shared<shared::HFR<RobotType, RobotType>>(robot_options);
	shared::HFR<RobotType, OptionsType> hfr(robot_options, robot_environment, path_evaluator, dynamic_path_planner);
	
	remove(options.logPath.c_str());	
	std::ofstream os(options.logPath, std::ios_base::app | std::ios_base::out);
	
	Eigen::MatrixXd process_covariance_matrix = hfr.build_covariance_matrix("process", robot_options->process_covariance);
	Eigen::MatrixXd observation_covariance_matrix = hfr.build_covariance_matrix("observation", robot_options->observation_covariance);
	Eigen::MatrixXd mean_process = Eigen::MatrixXd::Zero(robot_environment->getRobot()->getControlSpaceDimension(), 1);
	Eigen::MatrixXd mean_observation = Eigen::MatrixXd::Zero(robot_environment->getRobot()->getControlSpaceDimension(), 1);
	
	std::shared_ptr<shared::EigenMultivariateNormal<double>> process_distribution = 
			robot_environment->createDistribution(mean_process, process_covariance_matrix);
	std::shared_ptr<shared::EigenMultivariateNormal<double>> observation_distribution = 
			robot_environment->createDistribution(mean_observation, observation_covariance_matrix);
		
	robot_environment->setProcessDistribution(process_distribution);
	robot_environment->setObservationDistribution(observation_distribution);		
	robot_environment->getRobot()->setStateCovarianceMatrix(process_covariance_matrix);
	robot_environment->getRobot()->setObservationCovarianceMatrix(observation_covariance_matrix);
	
	hfr.runSimulation(os);
	
	os.close();
	return 0;
}

#endif