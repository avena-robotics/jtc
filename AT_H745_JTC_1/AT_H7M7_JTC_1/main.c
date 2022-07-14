#include "Control.h"
extern sControl* pC;
int main(void)
{
 	Control_SystemConf();
	while(1)
	{
		ControlJtcJogKinCalc();
		TG_TrajGen();
		
		
//		pC->Joints[0].reqIgnore = false;
//		pC->Joints[1].reqIgnore = true;
//		pC->Joints[2].reqIgnore = true;
//		pC->Joints[3].reqIgnore = true;
//		pC->Joints[4].reqIgnore = true;
//		pC->Joints[5].reqIgnore = true;
//		pC->Gripper.reqIgnore = true;
	}
}
