#include "teraranger_one.h"
#include "log.h"

int main ()
{
	teraranger::TerarangerOne _teraOne;
	int ret = _teraOne.init();
	DEBUG("MAVLINK INITIALIZATION RESULT! %u", ret);
	_teraOne.run();
	_teraOne.shutdown();


return 0;
}
