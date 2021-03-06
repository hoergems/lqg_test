#include "include/hfr.hpp"

using std::cout;
using std::endl;

namespace shared {

template<class RobotType, class OptionsType>
HFR<RobotType, OptionsType>::HFR(std::shared_ptr<shared::RobotOptions> &robot_options):
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
    observation_distribution_(nullptr)
{	
	
	double process_error_ = 2.5;
	double observation_error_ = 2.5;
	
	simulation_step_size_ = 0.001;
	
	path_deviation_cost_ = 200.0;
	control_deviation_cost_ = 1.0;
	
	step_penalty_ = 1.0;
	terminal_reward_ = 1000.0;
	illegal_move_penalty_ = 50;
	discount_factor_ = 0.998;
	
	num_evaluation_samples_ = 500;
	num_control_samples_ = 1;
	rrt_goal_bias_ = 0.05;
	min_max_control_duration_ = std::vector<int>({1, 4});	
	
	
	//create the robot and environment here
	bool created = robot_environment_->createManipulatorRobot(robot_options->robot_path);
	bool loaded_environment = robot_environment_->loadEnvironment(robot_options->environment_path);
	
	std::vector<double> goal_area;
	robot_environment_->getGoalArea(goal_area);
	std::vector<double> goal_position({goal_area[0], goal_area[1], goal_area[2]});
	double goal_radius = goal_area[3];
	
	robot_environment_->getRobot()->setGoalArea(goal_position, goal_radius);
	
	std::shared_ptr<shared::DynamicPathPlanner> dynamic_path_planner = std::make_shared<shared::DynamicPathPlanner>(false);
	path_evaluator_->setRobotEnvironment(robot_environment_);	
	path_evaluator_->setRewardModel(step_penalty_, illegal_move_penalty_, terminal_reward_, discount_factor_);
	path_evaluator_->setNumSamples(num_evaluation_samples_);
	
	Eigen::MatrixXd C = getC();
	Eigen::MatrixXd D = getD();
	path_evaluator_->setC(C);
	path_evaluator_->setD(D);	
	
	robot_environment_->setControlDuration(0.0333);
	robot_environment_->setSimulationStepSize(simulation_step_size_);
	
	std::vector<std::vector<double>> goal_states = robot_environment_->loadGoalStatesFromFile("/home/marcus/PhD/scripts/abt/problems/manipulator_discrete/goalstates.txt");
	
	std::shared_ptr<shared::PathPlannerOptions> options =
			std::make_shared<shared::ManipulatorPathPlannerOptions>();
	
	options->planning_algorithm = "RRT";
	options->goal_states = goal_states;
	static_cast<shared::ManipulatorPathPlannerOptions *>(options.get())->ee_goal_position = goal_position;
	options->goal_radius = goal_area[3];
	options->control_sampler = "discrete";
	options->addIntermediateStates = true;
	options->numControlSamples = num_control_samples_;
	options->RRTGoalBias = 0.05;
	options->min_max_control_durations = min_max_control_duration_;			
	
	dynamic_path_planner->setup(robot_environment_,				                
								"RRT");
	ompl::base::GoalPtr goal_region = 
			shared::makeManipulatorGoalRegion(dynamic_path_planner->getSpaceInformation(),
			                                  robot_environment_,
			                                  goal_states,
			                                  goal_position,
			                                  goal_area[3]);
	if (goal_region) {
		cout << "Goal region ok!" << endl;
	}
	
	
	
	dynamic_path_planner->setGoal(goal_region);	
	dynamic_path_planner->setControlSampler("discrete");
	dynamic_path_planner->addIntermediateStates(true);
	dynamic_path_planner->setNumControlSamples(num_control_samples_);
	dynamic_path_planner->setRRTGoalBias(0.05);
	dynamic_path_planner->setMinMaxControlDuration(min_max_control_duration_);
	
	
	//Build the error covariance matrices	
	Eigen::MatrixXd process_covariance_matrix = build_covariance_matrix("process", process_error_);
	Eigen::MatrixXd observation_covariance_matrix = build_covariance_matrix("observation", observation_error_);
	
	Eigen::MatrixXd mean_process = Eigen::MatrixXd::Zero(robot_environment_->getRobot()->getControlSpaceDimension(), 1);
	Eigen::MatrixXd mean_observation = Eigen::MatrixXd::Zero(robot_environment_->getRobot()->getControlSpaceDimension(), 1);
	process_distribution_ = robot_environment_->createDistribution(mean_process, process_covariance_matrix);
	observation_distribution_ = robot_environment_->createDistribution(mean_observation, observation_covariance_matrix);
	
	robot_environment_->setProcessDistribution(process_distribution_);
	robot_environment_->setObservationDistribution(observation_distribution_);
	
	robot_environment_->getRobot()->setStateCovarianceMatrix(process_covariance_matrix);
	robot_environment_->getRobot()->setObservationCovarianceMatrix(observation_covariance_matrix);
	
	Eigen::MatrixXd P_t = Eigen::MatrixXd::Zero(robot_environment_->getRobot()->getStateSpaceDimension(), 
			                                    robot_environment_->getRobot()->getStateSpaceDimension());
	
	unsigned int current_step = 1;
	const std::vector<double> start_state({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
	double timeout = 4.0;
	unsigned int num_threads = 8;
	std::shared_ptr<shared::PathEvaluationResult> res;
	path_evaluator_->planAndEvaluatePaths(start_state, 
			                              P_t, 
			                              current_step,
			                              timeout,
			                              num_threads,
										  options,
										  res);
	cout << "final result: " << res->path_objective << endl; 
	assert(!robot_environment_->getRobot() == false && "Robot is nullptr!!!");	
	assert(loaded_environment == true && "Couldn't load environment");
	
	
}

template<class RobotType, class OptionsType>
Eigen::MatrixXd HFR<RobotType, OptionsType>::build_covariance_matrix(std::string covariance_type, double &error) {	
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

template<class RobotType, class OptionsType>
std::shared_ptr<shared::SimulationStepResult> HFR<RobotType, OptionsType>::simulateStep(std::vector<double> &current_state,
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

template<class RobotType, class OptionsType>
Eigen::MatrixXd HFR<RobotType, OptionsType>::getC() {
	int state_space_dimension = robot_environment_->getRobot()->getStateSpaceDimension();
	Eigen::MatrixXd C = Eigen::MatrixXd::Identity(state_space_dimension, state_space_dimension);
	C = path_deviation_cost_ * C;
	return C;
}

template<class RobotType, class OptionsType>
Eigen::MatrixXd HFR<RobotType, OptionsType>::getD() {
	int control_space_dimension = robot_environment_->getRobot()->getControlSpaceDimension();
    Eigen::MatrixXd D = Eigen::MatrixXd::Identity(control_space_dimension, control_space_dimension);
	D = control_deviation_cost_ * D;	
	return D;
}

}