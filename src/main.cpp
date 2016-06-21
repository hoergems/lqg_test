#include "include/hfr.hpp"
#include "ManipulatorOptions.hpp"
#include <robots/ManipulatorRobot.hpp>

int main( int argc, const char* argv[] )
{	
	/**std::shared_ptr<shared::RobotOptions> robot_options = std::make_shared<shared::RobotOptions>();
	
	robot_options->process_error = 2.5;
	
	robot_options->path_deviation_cost = 500.0;
	robot_options->simulation_step_size = 0.001;
	robot_options->control_duration = 1.0 / 30.0;
	robot_options->robot_path = "/home/marcus/PhD/scripts/abt/problems/manipulator_discrete/model/test_4dof.urdf";
	robot_options->environment_path = "/home/marcus/PhD/scripts/abt/problems/manipulator_discrete/environment/env_4dof.xml";
	
	shared::HFR hfr(robot_options);*/
	//std::unique_ptr<options::OptionParser> parser = manipulator::ManipulatorOptions::makeParser(true);
	//parser->setOptions(&options);
	
	std::unique_ptr<options::OptionParser> parser = manipulator::ManipulatorOptions::makeParser(true);
	manipulator::ManipulatorOptions options;
	parser->setOptions(&options);
	
	std::shared_ptr<manipulator::ManipulatorOptions> options_manipulator = std::make_shared<manipulator::ManipulatorOptions>(options);
	shared::HFR<shared::ManipulatorRobot, manipulator::ManipulatorOptions> hfr(options_manipulator);
	
}