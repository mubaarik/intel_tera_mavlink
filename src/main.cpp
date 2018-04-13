#include "teraranger_one.h"
#include "log.h"

int main (int argc, char *argv[])
{
	teraranger::TerarangerOne _teraOne;
	_teraOne.init();
	_teraOne.run();
	_teraOne.shutdown();


return 0;
}
