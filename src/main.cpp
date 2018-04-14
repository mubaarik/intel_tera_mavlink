#include "teraranger_one.h"
#include "log.h"

int main (int argc, char *argv[])
{
	teraranger::TerarangerOne _teraOne;
	int ret = _teraOne.init();
	DEBUG("MAVLINK INITIALIZATION RESULT! %u", ret);
	_teraOne.run();
	_teraOne.shutdown();


return 0;
}
