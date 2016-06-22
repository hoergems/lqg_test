#include "include/simulate_hfr.hpp"
#include "DubinOptions.hpp"
#include <robots/DubinRobot.hpp>

int main( int argc, const char* argv[] )
{	
	return simulate<shared::DubinRobot, dubin::DubinOptions>();
}