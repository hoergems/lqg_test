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
	std::shared_ptr<shared::PathEvaluator> path_evaluator = utils::makePathEvaluator<OptionsType>(robot_environment, robot_options);
	
	//std::shared_ptr<shared::HFR<RobotType, RobotType>> hfr = std::make_shared<shared::HFR<RobotType, RobotType>>(robot_options);
	shared::HFR<RobotType, OptionsType> hfr(robot_options, robot_environment, path_evaluator, dynamic_path_planner);
	
	bool can_do_simulation = true;
	while (can_do_simulation) {
		
	}
	return 0;
}