#include "Control.h"
extern sControl* pC;
int main(void)
{
 	Control_SystemConf();
	while(1)
	{
		ControlJtcJogKinCalc();
		TG_TrajGen();
//		for(int i=0;i<6;i++)
//			pC->Joints[i].reqIgnore = true;
//		
//		pC->Gripper.reqIgnore = true;
	}
}
