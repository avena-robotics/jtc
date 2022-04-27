#include "Control.h"
extern sControl* pC;
int main(void)
{
 	Control_SystemConf();
	double p[8][8]={{0,0,0,0,0,0,0,0},{1,3,2,1,4,3,3,1},{4,2,3,6,5,2,5,1},{6,5,4,2,3,5,2,1},{9,0,1,3,5,4,8,1},{1,5,2,4,7,6,3,1},{16,12,5,2,8,1,3,1},{12,1,8,3,9,2,4,0}};
	pC->Tgen.stepTime = 0.01;
	for(int i=0;i<8;i++)
	{
		for(int j=0;j<6;j++)
			pC->Tgen.waypoints[i].pos[j] = p[i][j];
		pC->Tgen.waypoints[i].vel = p[i][6];
		pC->Tgen.waypoints[i].type = (eSeqPointType)p[i][7];
	}
	pC->Tgen.maxwaypoints = 7;
	Control_Delay(2000);
	while(1)
	{
		TG_TrajGen();
		Control_Delay(500);
	}
}
