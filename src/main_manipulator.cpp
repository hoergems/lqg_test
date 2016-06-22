#include "include/simulate_hfr.hpp"
#include "ManipulatorOptions.hpp"
#include <robots/ManipulatorRobot.hpp>

int main( int argc, const char* argv[] )
{	
	return simulate<shared::ManipulatorRobot, manipulator::ManipulatorOptions>();
}