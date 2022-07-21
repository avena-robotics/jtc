#include "Control.h"
extern sControl* pC;
int main(void)
{
 	Control_SystemConf();
	while(1)
	{
		ControlJtcJogKinCalc();
		TG_TrajGen();
	}
}
