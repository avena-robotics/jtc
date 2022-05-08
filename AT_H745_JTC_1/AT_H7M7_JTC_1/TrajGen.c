#include "TrajGen.h"
extern sControl* pC;
extern sTrajectory Traj;
extern sMB_RTUSlave	Mbs;
uint32_t time[5]={0,0,0,0,0};
union conv32
{
    uint32_t u32; // here_write_bits
    float    f32; // here_read_float
};
union conv64
{
    uint64_t u64; // here_write_bits
    double   d64; // here_read_double
};
void TG_SetDefaultVariables(void)
{
	Traj.Tgen.stepTime = 0.01;
	Traj.Tgen.seqNum = 0;
	Traj.Tgen.maxwaypoints = 0;
	Traj.Tgen.maxpoints = 0;
	Traj.Tgen.reqTrajPrepare = false;
	
	
	for(int num=0;num<JOINTS_MAX;num++)
		for(int i=0;i<TG_SEQWAYPOINTSSMAX;i++)
			for(int j=0;j<5;j++)
				Traj.Tgen.path[num][i][j] = 0.0;
	
	for(int i=0;i<TG_SEQWAYPOINTSSMAX;i++)
	{
		Traj.Tgen.waypoints[i].active = false;
		Traj.Tgen.waypoints[i].tend = 0.0;
		Traj.Tgen.waypoints[i].type = SPT_Finish;
		Traj.Tgen.waypoints[i].moveType = SPMT_Null;
		Traj.Tgen.waypoints[i].vel = 0.0;
		for(int num=0;num<JOINTS_MAX;num++)
			Traj.Tgen.waypoints[i].pos[num] = 0.0;
	}
	Traj.Tgen.status = TGS_Idle;
}
void TG_Conf(void)
{
	TG_SetDefaultVariables();
}
bool TG_GetSeqFromMbs(void)
{
	uint32_t idx = MRN_SeqStart;
	Traj.Tgen.seqNum = Mbs.hregs[idx++];
	Traj.Tgen.maxwaypoints = Mbs.hregs[idx++];
	
	if(Traj.Tgen.maxwaypoints == 0)
		return false;
	
	for(uint32_t j=0;j<JOINTS_MAX;j++)
		Traj.Tgen.waypoints[0].pos[j] = pC->Joints[j].currentPos;
	for(uint32_t j=0;j<JOINTS_MAX;j++)
		Traj.Tgen.waypoints[0].pos[j] = 0;
	Traj.Tgen.waypoints[0].vel = 0.0;
	Traj.Tgen.waypoints[0].type = SPT_Start;
	Traj.Tgen.waypoints[0].moveType = SPMT_Ptp;
	
	union conv32 x;
	for(uint32_t i=0;i<Traj.Tgen.maxwaypoints;i++)
	{
		Traj.Tgen.waypoints[i+1].moveType = (eSeqPointMoveType)Mbs.hregs[idx++];
		for(uint32_t j=0;j<JOINTS_MAX;j++)
		{
			x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
			x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
			Traj.Tgen.waypoints[i+1].pos[j] = x.f32;
		}
		x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
		x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
		Traj.Tgen.waypoints[i+1].vel = x.f32;
		Traj.Tgen.waypoints[i+1].type = SPT_Way;
	}
	
	Traj.Tgen.waypoints[Traj.Tgen.maxwaypoints].type = SPT_Finish;
	
	for(uint32_t i=1;i<Traj.Tgen.maxwaypoints;i++)
		if(fabs(Traj.Tgen.waypoints[i].vel) < 0.001)
			return false;
	
	return true;
}
// ******************************** Wielomian 5 stopnia ****************************************
//double a0(double qstart, double qend, double vstart, double vend, double tend)
//{
//	return qstart;
//}
//double a1(double qstart, double qend, double vstart, double vend, double tend)
//{
//	return vstart;
//}
//double a2(double qstart, double qend, double vstart, double vend, double tend)
//{
//	return 0;
//}
//double a3(double qstart, double qend, double vstart, double vend, double tend)
//{
//	return (10.0*qend-10.0*qstart+tend*(0.-4.0*vend-6.0*vstart))/pow(tend,3);
//}
//double a4(double qstart, double qend, double vstart, double vend, double tend)
//{
//	return (-15.0*qend+15.0*qstart+tend*(0.+7.0*vend+8.0*vstart))/pow(tend,4);
//}
//double a5(double qstart, double qend, double vstart, double vend, double tend)
//{
//	return (6.0*qend-6.0*qstart+tend*(0.-3.0*vend-3.0*vstart))/pow(tend,5);
//}
//double q(double qstart, double qend, double vstart, double vend, double tend, double t)
//{
//	return qstart+t*vstart+(pow(t,3)*(0.+10.*qend-10.*qstart-4.*tend*vend-6.*tend*vstart))/pow(tend,3)+(pow(t,5)*(0.+6.*qend-6.*qstart-3.*tend*vend-3.*tend*vstart))/pow(tend,5)+(pow(t,4)*(0.-15.*qend+15.*qstart+7.*tend*vend+8.*tend*vstart))/pow(tend,4);
//}
//double dq(double qstart, double qend, double vstart, double vend, double tend, double t)
//{
//	return vstart+(3.*pow(t,2)*(0.+10.*qend-10.*qstart-4*tend*vend-6.*tend*vstart))/pow(tend,3)+(4*pow(t,3)*(0.-15.*qend+15.*qstart+7.*tend*vend+8.*tend*vstart))/pow(tend,4)+(5*pow(t,4)*(6.*qend-6.*qstart+tend*(0.-3.*(vend+vstart))))/pow(tend,5);
//}
//double ddq(double qstart, double qend, double vstart, double vend, double tend, double t)
//{
//	return 2.*((3.*t*(0.+10.*qend-10.*qstart-4.*tend*vend-6.*tend*vstart))/pow(tend,3)+(6.*pow(t,2)*(0.-15.*qend+15.*qstart+7.*tend*vend+8.*tend*vstart))/pow(tend,4)+(10.*pow(t,3)*(6.*qend-6.*qstart+tend*(0.-3.*(vend+vstart))))/pow(tend,5));
//}
//double q_at(double a5, double a4, double a3, double a2, double a1, double a0, double t)
//{
//	return a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5);
//}
//double dq_at(double a5, double a4, double a3, double a2, double a1, double a0, double t)
//{
//	return a1 + 2.0*a2*t + 3.0*a3*pow(t,2) + 4.0*a4*pow(t,3) + 5.0*a5*pow(t,4);
//}
//double ddq_at(double a5, double a4, double a3, double a2, double a1, double a0, double t)
//{
//	return 2.0*a2 + 6.0*a3*t + 12.0*a4*pow(t,2) + 20.0*a5*pow(t,3);
//}
//double trule(double qstart, double qend, double vstart, double vend, double tend)
//{
//	return -(tend*(-5.*qend + 5.*qstart + 2.*tend*vend + 3.*tend*vstart))/(5.*(2.*qend - 2.*qstart - tend*(vend + vstart)));
//}
//static void TG_FindPath(int num)
//{
//	for(uint32_t i=1;i<=Traj.Tgen.maxwaypoints;i++)
//	{
//		Traj.Tgen.path[num][i-1][0] = Traj.Tgen.waypoints[i-1].pos[num];
//		Traj.Tgen.path[num][i-1][1] = Traj.Tgen.waypoints[i].pos[num];
//		Traj.Tgen.path[num][i-1][2] = Traj.Tgen.waypoints[i-1].vel - 0.001;
//		Traj.Tgen.path[num][i-1][3] = Traj.Tgen.waypoints[i].vel - 0.001;
//		Traj.Tgen.path[num][i-1][4] = Traj.Tgen.waypoints[i].vel;
//	}
//	for(uint32_t i=1;i<Traj.Tgen.maxwaypoints;i++)
//	{
//		if(Traj.Tgen.path[num][i][4] < Traj.Tgen.path[num][i-1][4])
//		{
//			Traj.Tgen.path[num][i-1][3] = Traj.Tgen.path[num][i][4] - 0.001;
//			Traj.Tgen.path[num][i][2] = Traj.Tgen.path[num][i][4] - 0.001;
//		}
//		else
//		{
//			Traj.Tgen.path[num][i-1][3] = Traj.Tgen.path[num][i-1][4] - 0.001;
//			Traj.Tgen.path[num][i][2] = Traj.Tgen.path[num][i-1][4] - 0.001;
//		}
//	}
//	for(uint32_t i=1;i<Traj.Tgen.maxwaypoints;i++)
//	{
//		if(Traj.Tgen.path[num][i][1] < Traj.Tgen.path[num][i-1][0])
//		{
//			Traj.Tgen.path[num][i][2] *= -1.0;
//			Traj.Tgen.path[num][i-1][3] *= -1.0;
//		}
//	}
//	for(uint32_t i=1;i<Traj.Tgen.maxwaypoints;i++)
//	{
//		if(Traj.Tgen.path[num][i-1][1] > Traj.Tgen.path[num][i-1][0] && Traj.Tgen.path[num][i][1] < Traj.Tgen.path[num][i][0])
//		{
//			Traj.Tgen.path[num][i-1][3] = 0.0;
//			Traj.Tgen.path[num][i][2] = 0.0;
//		}
//	}
//	for(uint32_t i=1;i<Traj.Tgen.maxwaypoints;i++)
//	{
//		if(Traj.Tgen.path[num][i-1][1] < Traj.Tgen.path[num][i-1][0] && Traj.Tgen.path[num][i][1] > Traj.Tgen.path[num][i][0])
//		{
//			Traj.Tgen.path[num][i-1][3] = 0.0;
//			Traj.Tgen.path[num][i][2] = 0.0;
//		}
//	}
//	Traj.Tgen.path[num][0][2]=0.0;
//	Traj.Tgen.path[num][Traj.Tgen.maxwaypoints-1][3]=0.0;
//}
//static double TG_FindTend(double in[5])
//{
//	double t, tend, vv;
//	double qstart = in[0], qend = in[1], vstart = in[2], vend = in[3], vmax = in[4];
//	if(fabs(qend - qstart)<0.001)
//		qend += 0.001;
//	tend = 2.0 * fabs(qend - qstart) / vmax;
//	
//	t=trule(qstart, qend, vstart, vend, tend);
//	vv = fabs(dq(qstart, qend, vstart, vend, tend,t));
//	
//	while((vmax - vv) < -0.001)
//	{
//		tend += 0.001;
//		t=trule(qstart, qend, vstart, vend, tend);
//		vv = fabs(dq(qstart, qend, vstart, vend, tend,t));
//	}
//	while((vmax - vv)> 0.001)
//	{
//		tend -= 0.001;
//		t=trule(qstart, qend, vstart, vend, tend);
//		vv = fabs(dq(qstart, qend, vstart, vend, tend,t));
//	}
//	return tend;
//}
//static void TG_FindTendAllDrives()
//{
//	double deltaq, deltaqmax;
//	int deltaqmax_num = 0;
//	for(uint32_t i=0;i<Traj.Tgen.maxwaypoints;i++)
//	{
//		deltaqmax_num = 0;
//		deltaqmax = fabs(Traj.Tgen.path[0][i][1] - Traj.Tgen.path[0][i][0]); //dystans do pokonania dla napedu numer 0
//		for(uint32_t num=1;num<JOINTS_MAX;num++)
//		{
//			deltaq = fabs(Traj.Tgen.path[num][i][1] - Traj.Tgen.path[num][i][0]); //dystans do pokonania dla napedu numer num
//			if(deltaq > deltaqmax)
//			{
//				deltaqmax = deltaq;
//				deltaqmax_num = num;
//			}
//		}
//		Traj.Tgen.waypoints[i].tend = TG_FindTend(Traj.Tgen.path[deltaqmax_num][i]);
//	}
//}
//static void TG_Poly5V_1Drive(int num)
//{
//	double qstart, qend, vstart, vend, tend, t;
//	uint32_t idx = 0;
//	for(uint32_t x=0;x<Traj.Tgen.maxwaypoints;x++)
//	{
//		qstart = Traj.Tgen.path[num][x][0];
//		qend = Traj.Tgen.path[num][x][1];
//		vstart = Traj.Tgen.path[num][x][2];
//		vend = Traj.Tgen.path[num][x][3];
//		tend = Traj.Tgen.waypoints[x].tend;
//		
//		double a0v = a0(qstart, qend, vstart, vend, tend);
//		double a1v = a1(qstart, qend, vstart, vend, tend);
//		double a2v = a2(qstart, qend, vstart, vend, tend);
//		double a3v = a3(qstart, qend, vstart, vend, tend);
//		double a4v = a4(qstart, qend, vstart, vend, tend);
//		double a5v = a5(qstart, qend, vstart, vend, tend);
//		
//		for(uint32_t i=Traj.Tgen.stepTime;i<(tend/Traj.Tgen.stepTime)-1;i++)
//		{
//			t = Traj.Tgen.stepTime*(double)i;
//			Traj.points[idx].pos[num] = q_at(a0v, a1v, a2v, a3v, a4v, a5v, t) / pC->Joints[num].limitPosMax * MAXINT16;
//			Traj.points[idx].vel[num] = dq_at(a0v, a1v, a2v, a3v, a4v, a5v, t) / pC->Joints[num].limitVelMax * MAXINT16;
//			Traj.points[idx].acc[num] = ddq_at(a0v, a1v, a2v, a3v, a4v, a5v, t) / pC->Joints[num].limitAccMax * MAXINT16;
//			idx++;
//		}
//	}
//	Traj.Tgen.maxpoints = idx;
//}
//void TG_TrajGen(void)
//{
//	if(pC->Jtc.currentFsm != JTC_FSM_HoldPos && pC->Jtc.currentFsm != JTC_FSM_Operate)
//		return;
//	if(Traj.Tgen.reqTrajPrepare == false)
//		return;
//	
//	TG_SetDefaultVariables();
//	
//	if(TG_GetSeqFromMbs() == false)
//		return;
//	Control_TrajClear();
//	pC->tick= 0;
//	
//	for(uint32_t num=0;num<JOINTS_MAX;num++)
//		TG_FindPath(num);
//	time[0] = pC->tick;
//	
//	TG_FindTendAllDrives();
//	
//	time[1] = pC->tick;
//	
//	for(uint32_t num=0;num<JOINTS_MAX;num++)
//		TG_Poly5V_1Drive(num);
//	
//	time[2] = pC->tick;
//	
//	Traj.Tgen.status = TGS_Ready;
//	Traj.stepTime = 1000.0  * Traj.Tgen.stepTime; //W Tgen stepTime jest w sekundach (def 0.01), a w Traj stepTime jest w ilosci ms na punkt (def 10)
//	Traj.numRecPoints = Traj.Tgen.maxpoints;
//	Traj.comStatus = TCS_WasRead;
//	Traj.targetTES = TES_Stop;
//	Traj.Tgen.reqTrajPrepare = false;
//}

// ******************************** SLP ****************************************
static const double accmax = 12.0;
uint32_t idx = 0;
static double TG_SLP_q1(double qstart, double qend, double vstart, double vend, double acc1, double acc2, double V1, double t)
{
	return qstart+(acc1*pow(t,2))/2.+t*vstart;
}
static double TG_SLP_q2(double qstart, double qend, double vstart, double vend, double acc1, double acc2, double V1, double t)
{
	return qstart+t*V1+vstart*fabs((V1-vstart)/acc1)+(acc1*pow(fabs((V1-vstart)/acc1),2))/2.;
}
static double TG_SLP_q3(double qstart, double qend, double vstart, double vend, double acc1, double acc2, double V1, double t)
{
	return qend+vend*(t-fabs((V1-vend)/acc2))+(acc2*pow(t-fabs((V1-vend)/acc2),2))/2.;
}
static double TG_SLP_dq1(double qstart, double qend, double vstart, double vend, double acc1, double acc2, double V1, double t)
{
	return acc1*t+vstart;
}
static double TG_SLP_dq2(double qstart, double qend, double vstart, double vend, double acc1, double acc2, double V1, double t)
{
	return V1;
}
static double TG_SLP_dq3(double qstart, double qend, double vstart, double vend, double acc1, double acc2, double V1, double t)
{
	return acc2*t+vend-acc2*fabs((V1-vend)/acc2);
}
static double TG_SLP_ddq1(double qstart, double qend, double vstart, double vend, double acc1, double acc2, double V1, double t)
{
	return acc1;
}
static double TG_SLP_ddq2(double qstart, double qend, double vstart, double vend, double acc1, double acc2, double V1, double t)
{
	return 0;
}
static double TG_SLP_ddq3(double qstart, double qend, double vstart, double vend, double acc1, double acc2, double V1, double t)
{
	return acc2;
}
static double TG_SLP_FindVmax(double tab[5])
{
	double V1, acc1, acc2, t1, t3, q01, q12, q23;
	double qstart=tab[0], qend=tab[1], vstart=tab[2], vend=tab[3], vmax=tab[4];
	double q03 = qend - qstart;
	if(q03 >= 0.0) 		V1 = fabs(vmax);
	else							V1 = -fabs(vmax);
	if(V1 > vstart)		acc1 = fabs(accmax);
	else							acc1 = -fabs(accmax);
	if(vend > V1)			acc2 = fabs(accmax);
	else							acc2 = -fabs(accmax);
	if(acc1 == acc2)	acc2 -= 0.001;
	
	t1=fabs((V1-vstart)/acc1);
	t3=fabs((vend-V1)/acc2);
	
	q01 = vstart * t1 + (acc1 * t1 * t1) / 2.0;
	q23 = V1 * t3 + (acc2 * t3 * t3) / 2.0;
	q12 = q03 - q01 - q23;
	
	if((q03 > 0.0 && q12 <= 0.0) || (q03 < 0.0 && q12 >= 0.0))
	{
		V1 = sqrt((acc1 - acc2)*(acc1*(-2*acc2*q03 + pow(vend,2)) - acc2*pow(vstart,2)))/(acc1 - acc2);
	}

	return fabs(V1);
}
static double TG_SLP_FindTime(double tab[5])
{
	double V1, acc1, acc2, t1, t2, t3, q01, q12, q23;
	double qstart=tab[0], qend=tab[1], vstart=tab[2], vend=tab[3], vmax=tab[4];
	double q03 = qend - qstart;
	if(q03 >= 0.0) 		V1 = fabs(vmax);
	else							V1 = -fabs(vmax);
	if(V1 > vstart)		acc1 = fabs(accmax);
	else							acc1 = -fabs(accmax);
	if(vend > V1)			acc2 = fabs(accmax);
	else							acc2 = -fabs(accmax);
	if(acc1 == acc2)	acc2 -= 0.001;
	
	t1=fabs((V1-vstart)/acc1);
	t3=fabs((vend-V1)/acc2);
	
	q01 = vstart * t1 + (acc1 * t1 * t1) / 2.0;
	q23 = V1 * t3 + (acc2 * t3 * t3) / 2.0;
	q12 = q03 - q01 - q23;
	t2 = fabs(q12 / V1);	
	
	return (t1+t2+t3);
}
double times[JOINTS_MAX][TG_SEQWAYPOINTSSMAX];
static void TG_SLP_FindTimeMax(void)
{
	
	for(int num=0;num<JOINTS_MAX;num++)
	{
		for(int i=0;i<Traj.Tgen.maxwaypoints;i++)
		{
			times[num][i] = TG_SLP_FindTime(Traj.Tgen.path[num][i]);
		}
	}
	for(int i=0;i<Traj.Tgen.maxwaypoints;i++)
	{
		double max = times[0][i];
		for(int num=0;num<JOINTS_MAX;num++)
		{
			if(times[num][i] > max)
				max = times[num][i];
		}
		Traj.Tgen.waypoints[i].tend = max;
	}
}
static void TG_SLP_FindPath(int num)
{
	for(int i=1;i<=Traj.Tgen.maxwaypoints;i++)
	{
		if(fabs(Traj.Tgen.waypoints[i-1].pos[num] - Traj.Tgen.waypoints[i].pos[num]) < 0.001)
			Traj.Tgen.waypoints[i-1].pos[num] += 0.002;
	}
	for(int i=1;i<=Traj.Tgen.maxwaypoints;i++)
	{
		Traj.Tgen.path[num][i-1][0] = Traj.Tgen.waypoints[i-1].pos[num];
		Traj.Tgen.path[num][i-1][1] = Traj.Tgen.waypoints[i].pos[num];
		Traj.Tgen.path[num][i-1][2] = 0.0;
		Traj.Tgen.path[num][i-1][3] = 0.0;
		Traj.Tgen.path[num][i-1][4] = Traj.Tgen.waypoints[i].vel;
	}
	for(int i=0;i<Traj.Tgen.maxwaypoints;i++)
	{
		Traj.Tgen.path[num][i][4] = TG_SLP_FindVmax(Traj.Tgen.path[num][i]);
	}
	for(int i=1;i<Traj.Tgen.maxwaypoints;i++)
	{
		if(Traj.Tgen.path[num][i][4] < Traj.Tgen.path[num][i-1][4])
		{
			Traj.Tgen.path[num][i-1][3] = 0.0;
			Traj.Tgen.path[num][i][2] = 0.0;
		}
		else
		{
			Traj.Tgen.path[num][i-1][3] = 0.0;
			Traj.Tgen.path[num][i][2] = 0.0;
		}
	}
	for(int i=1;i<Traj.Tgen.maxwaypoints;i++)
	{
		if(Traj.Tgen.path[num][i][1] < Traj.Tgen.path[num][i-1][0])
		{
			Traj.Tgen.path[num][i][2] *= -1.0;
			Traj.Tgen.path[num][i-1][3] *= -1.0;
		}
	}
	for(int i=1;i<Traj.Tgen.maxwaypoints;i++)
	{
		if(Traj.Tgen.path[num][i-1][1] > Traj.Tgen.path[num][i-1][0] && Traj.Tgen.path[num][i][1] < Traj.Tgen.path[num][i][0])
		{
			Traj.Tgen.path[num][i-1][3] = 0.0;
			Traj.Tgen.path[num][i][2] = 0.0;
		}
	}
	for(int i=1;i<Traj.Tgen.maxwaypoints;i++)
	{
		if(Traj.Tgen.path[num][i-1][1] < Traj.Tgen.path[num][i-1][0] && Traj.Tgen.path[num][i][1] > Traj.Tgen.path[num][i][0])
		{
			Traj.Tgen.path[num][i-1][3] = 0.0;
			Traj.Tgen.path[num][i][2] = 0.0;
		}
	}
	Traj.Tgen.path[num][0][2]=0.0;
	Traj.Tgen.path[num][Traj.Tgen.maxwaypoints-1][3]=0.0;
}
static void TG_SLP_FindTraj1Drive(int num, int x)
{
	double V1, acc1, acc2, t1, t2, t3, q01, q12, q23, V1a, V1b, t03a, t03b;
	double qstart=Traj.Tgen.path[num][x][0], qend=Traj.Tgen.path[num][x][1], vstart=Traj.Tgen.path[num][x][2], vend=Traj.Tgen.path[num][x][3], vmax=Traj.Tgen.path[num][x][4];
	double tend = Traj.Tgen.waypoints[x].tend;
	double q03 = qend - qstart;
	if(q03 >= 0.0) 		V1 = fabs(vmax);
	else				V1 = -fabs(vmax);
	if(V1 > vstart)		acc1 = fabs(accmax);
	else				acc1 = -fabs(accmax);
	if(vend > V1)		acc2 = fabs(accmax);
	else				acc2 = -fabs(accmax);
	if(acc1 == acc2)	acc2 -= 0.001;
	
	//***************************************
	V1a = -((acc1*acc2*tend - acc1*vend + acc2*vstart + sqrt(acc1*acc2*(acc1*(2*q03 + tend*(acc2*tend - 2*vend)) + pow(vend - vstart,2) - 2*acc2*(q03 - tend*vstart))))/(acc1 - acc2));
	vmax = fabs(V1a);
	if(q03 >= 0.0) 		V1 = fabs(vmax);
	else				V1 = -fabs(vmax);
	if(V1 > vstart)		acc1 = fabs(accmax);
	else				acc1 = -fabs(accmax);
	if(vend > V1)		acc2 = fabs(accmax);
	else				acc2 = -fabs(accmax);
	if(acc1 == acc2)	acc2 -= 0.001;
	t1=fabs((V1-vstart)/acc1);
	t3=fabs((vend-V1)/acc2);
	q01 = vstart * t1 + (acc1 * t1 * t1) / 2.0;
	q23 = V1 * t3 + (acc2 * t3 * t3) / 2.0;
	q12 = q03 - q01 - q23;
	t2 = fabs(q12 / V1);	
	t03a =t1+t2+t3;
	//***************************************
	V1b = (-(acc1*acc2*tend) + acc1*vend - acc2*vstart + sqrt(acc1*acc2*(acc1*(2*q03 + tend*(acc2*tend - 2*vend)) + pow(vend - vstart,2) - 2*acc2*(q03 - tend*vstart))))/(acc1 - acc2);
	vmax = fabs(V1b);
	if(q03 >= 0.0) 		V1 = fabs(vmax);
	else				V1 = -fabs(vmax);
	if(V1 > vstart)		acc1 = fabs(accmax);
	else				acc1 = -fabs(accmax);
	if(vend > V1)		acc2 = fabs(accmax);
	else				acc2 = -fabs(accmax);
	if(acc1 == acc2)	acc2 -= 0.001;
	t1=fabs((V1-vstart)/acc1);
	t3=fabs((vend-V1)/acc2);
	q01 = vstart * t1 + (acc1 * t1 * t1) / 2.0;
	q23 = V1 * t3 + (acc2 * t3 * t3) / 2.0;
	q12 = q03 - q01 - q23;
	t2 = fabs(q12 / V1);	
	t03b =t1+t2+t3;
	//***************************************
	if(fabs(tend - t03a) <= (2.0*Traj.Tgen.stepTime))		vmax = fabs(V1a);
	if(fabs(tend - t03b) <= (2.0*Traj.Tgen.stepTime))		vmax = fabs(V1b);
	if(q03 >= 0.0) 		V1 = fabs(vmax);
	else				V1 = -fabs(vmax);
	if(V1 > vstart)		acc1 = fabs(accmax);
	else				acc1 = -fabs(accmax);
	if(vend > V1)		acc2 = fabs(accmax);
	else				acc2 = -fabs(accmax);
	if(acc1 == acc2)	acc2 -= 0.001;
	t1=fabs((V1-vstart)/acc1);
	t3=fabs((vend-V1)/acc2);
	q01 = vstart * t1 + (acc1 * t1 * t1) / 2.0;
	q23 = V1 * t3 + (acc2 * t3 * t3) / 2.0;
	q12 = q03 - q01 - q23;
	t2 = fabs(q12 / V1);
	
	//***************************************
	double t;
	for(int i=0;i<(t1/Traj.Tgen.stepTime);i++)
	{
		t = Traj.Tgen.stepTime*(double)i;
		Traj.points[idx].pos[num] = TG_SLP_q1(qstart, qend, vstart, vend, acc1, acc2, V1, t) / pC->Joints[num].limitPosMax * MAXINT16;
		Traj.points[idx].vel[num] = TG_SLP_dq1(qstart, qend, vstart, vend, acc1, acc2, V1, t) / pC->Joints[num].limitVelMax * MAXINT16;
		Traj.points[idx].acc[num] = TG_SLP_ddq1(qstart, qend, vstart, vend, acc1, acc2, V1, t) / pC->Joints[num].limitAccMax * MAXINT16;
		idx++;
	}
	for(int i=0;i<(t2/Traj.Tgen.stepTime);i++)
	{
		t = Traj.Tgen.stepTime*(double)i;
		Traj.points[idx].pos[num] = TG_SLP_q2(qstart, qend, vstart, vend, acc1, acc2, V1, t) / pC->Joints[num].limitPosMax * MAXINT16;
		Traj.points[idx].vel[num] = TG_SLP_dq2(qstart, qend, vstart, vend, acc1, acc2, V1, t) / pC->Joints[num].limitVelMax * MAXINT16;
		Traj.points[idx].acc[num] = TG_SLP_ddq2(qstart, qend, vstart, vend, acc1, acc2, V1, t) / pC->Joints[num].limitAccMax * MAXINT16;
		idx++;
	}
	for(int i=0;i<(t3/Traj.Tgen.stepTime)-1;i++)
	{
		t = Traj.Tgen.stepTime*(double)i;
		Traj.points[idx].pos[num] = TG_SLP_q3(qstart, qend, vstart, vend, acc1, acc2, V1, t) / pC->Joints[num].limitPosMax * MAXINT16;
		Traj.points[idx].vel[num] = TG_SLP_dq3(qstart, qend, vstart, vend, acc1, acc2, V1, t) / pC->Joints[num].limitVelMax * MAXINT16;
		Traj.points[idx].acc[num] = TG_SLP_ddq3(qstart, qend, vstart, vend, acc1, acc2, V1, t) / pC->Joints[num].limitAccMax * MAXINT16;
		idx++;
	}
}
void TG_TrajGen(void)
{
	if(pC->Jtc.currentFsm != JTC_FSM_HoldPos && pC->Jtc.currentFsm != JTC_FSM_Operate)
		return;
	if(Traj.Tgen.reqTrajPrepare == false)
		return;
	
	TG_SetDefaultVariables();
	
	if(TG_GetSeqFromMbs() == false)
		return;
	Control_TrajClear();
	pC->tick = 0;
	
	for(uint32_t num=0;num<JOINTS_MAX;num++)
		TG_SLP_FindPath(num);
	time[0] = pC->tick;
	
	TG_SLP_FindTimeMax();
	time[1] = pC->tick;
	
	for(int num=0;num<JOINTS_MAX;num++)
	{
		idx = 0;
		for(uint32_t x=0;x<Traj.Tgen.maxwaypoints;x++)
		{
			TG_SLP_FindTraj1Drive(num, x);
		}
	}
	
	Traj.Tgen.maxpoints = idx;
//	for(uint32_t x=0;x<Traj.Tgen.maxwaypoints;x++)
//		Traj.Tgen.maxpoints += 100.0 * Traj.Tgen.waypoints[x].tend;
	
	time[2] = pC->tick;
	
	Traj.Tgen.status = TGS_Ready;
	Traj.stepTime = 1000.0  * Traj.Tgen.stepTime; //W Tgen stepTime jest w sekundach (def 0.01), a w Traj stepTime jest w ilosci ms na punkt (def 10)
	Traj.numRecPoints = Traj.Tgen.maxpoints;
	Traj.comStatus = TCS_WasRead;
	Traj.targetTES = TES_Stop;
	Traj.Tgen.reqTrajPrepare = false;
}
