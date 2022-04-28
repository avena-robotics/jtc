#include "Control.h"
extern sControl* pC;
int main(void)
{
 	Control_SystemConf();
	while(1)
	{
		TG_TrajGen();
	}
}
