#include "TrajGen.h"
extern sControl* pC;
extern sTrajectory Traj;
extern sMB_RTUSlave	Mbs;
double accmax = 1.0; //minimum 1.0, maximum 4*pi
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
	Traj.Tgen.stepTime = 0.001;
	Traj.Tgen.seqNum = 0;
	Traj.Tgen.maxwaypoints = 0;
	Traj.Tgen.maxpoints = 0;
	Traj.Tgen.reqTrajPrepare = false;
	Traj.Tgen.minVelocity = 0.1; //Unit: rad/sek
	
	if(accmax < 1.0)
		accmax = 1.0;
	if(accmax > M_4_PI)
		accmax = M_4_PI;
	
	
	//Znajdujemy najmniejszy z górnych limitów predkosci jointów i te wartosc przyjmujemy jako maksymalna dla ruchu w trajektorii Unit: rad/sek
	Traj.Tgen.maxVelocity = fabs(pC->Joints[0].limitVelMax);
	for(int num=0;num<JOINTS_MAX;num++)
		if(fabs(pC->Joints[num].limitVelMax) < Traj.Tgen.maxVelocity)
			Traj.Tgen.maxVelocity = fabs(pC->Joints[num].limitVelMax);
	
	for(int num=0;num<JOINTS_MAX;num++)
		for(int i=0;i<TG_SEQWAYPOINTSSMAX;i++)
			for(int j=0;j<2;j++)
				Traj.Tgen.path[num][i][j] = 0.0;
		
	for(int num=0;num<JOINTS_MAX;num++)
		for(int i=0;i<TG_SEQWAYPOINTSSMAX;i++)
			for(int j=0;j<3;j++)
				Traj.Tgen.trace[num][i][j] = 0.0;
		
	for(int num=0;num<JOINTS_MAX;num++)
		for(int i=0;i<TG_SEQWAYPOINTSSMAX;i++)
			for(int j=0;j<6;j++)
				Traj.Tgen.stamps[num][i][j] = 0.0;
		
	for(int i=0;i<TG_SEQWAYPOINTSSMAX;i++)
	{
		Traj.Tgen.waypoints[i].active = false;
		Traj.Tgen.waypoints[i].type = SPT_Finish;
		Traj.Tgen.waypoints[i].moveType = SPMT_Null;
		Traj.Tgen.waypoints[i].vel = 0.0;
		Traj.Tgen.waypoints[i].zone = 0.0;
		Traj.Tgen.waypoints[i].refSystem = 0;
		
		Traj.Tgen.waypoints[i].pos = Vec6Zeros();
		Traj.Tgen.waypoints[i].qSol = Vec6Zeros();
		Traj.Tgen.waypoints[i].pos = Vec6Zeros();
		Traj.Tgen.waypoints[i].quat = Vec4Zeros();
		Traj.Tgen.waypoints[i].mat = Mat4Ones();
		for(int j=0;j<IK_SOLNUMREAL;j++)
		{
			Traj.Tgen.waypoints[i].sol.v[j] = Vec6Zeros();
		}
	}
	Traj.Tgen.status = TGS_Idle;
}
void TG_Conf(void)
{
	TG_SetDefaultVariables();
}
static sRobPos TG_GetSeqPointConfSpace(uint16_t numidx)
{
	uint16_t idx = numidx;
	sRobPos p;
	union conv32 x;
	p.moveType = (eSeqPointMoveType)Mbs.hregs[idx++];
	for(uint32_t j=0;j<JOINTS_MAX;j++)
	{
		x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
		x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
		p.pos.v[j] = x.f32;
	}
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.vel = x.f32;
	p.type = SPT_Way;
	
	return p;
}
static sRobPos TG_GetSeqPointKartesianSpace(uint16_t numidx)
{
	uint16_t idx = numidx;
	sRobPos p;
	p.mat = Mat4Ones();
	union conv32 x;
	p.moveType = (eSeqPointMoveType)Mbs.hregs[idx++];
	
	//Wektor pozycji XYZ
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.mat.v[0][3] = x.f32;
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.mat.v[1][3] = x.f32;
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.mat.v[2][3] = x.f32;
	
	//Wektor orientacji w kwaternionach
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.quat.v[0] = x.f32;
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.quat.v[1] = x.f32;
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.quat.v[2] = x.f32;
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.quat.v[3] = x.f32;
	
	//Wektor konfiguracji rozwiazania kinematyki
	p.conf.v[0] = Mbs.hregs[idx++];
	p.conf.v[1] = Mbs.hregs[idx++];
	p.conf.v[2] = Mbs.hregs[idx++];
	p.conf.v[3] = Mbs.hregs[idx++];
	
	//Numer ukladu wspólrzednych wzgledem którego jest podawana pozycja i orientacja: 0 - uklad bazowy robota
	p.refSystem = Mbs.hregs[idx++];
	
	//Predkosc maksymalna dla tego punktu
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.vel = x.f32;
	
	//Promien okregu w jakimma zmiescic sie robot przy omijaniu waypointów. Wartosc 0.0 oznacza brak omijania w tym punkcie.
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.zone = x.f32;
	
	p = Kin_IKCalc(p);
	p.pos = p.qSol;
	
	p.type = SPT_Way;
	
	return p;
}
static bool TG_GetSeqFromMbs(void)
{
	uint32_t idx = MRN_SeqStart;
	Traj.Tgen.seqNum = Mbs.hregs[idx++];
	Traj.Tgen.recwaypoints = Mbs.hregs[idx++];
	Traj.Tgen.maxwaypoints = Traj.Tgen.recwaypoints + 3;
	
	if(Traj.Tgen.recwaypoints == 0)
	{
		Traj.Tgen.trajPrepStatus = TPS_NotEnoughWaypoints;
		return false;
	}
	
	if(Traj.Tgen.maxwaypoints > TG_SEQWAYPOINTSSMAX)
	{
		Traj.Tgen.trajPrepStatus = TPS_ToMuchWaypoints;
		return false;
	}
	
	for(uint32_t j=0;j<JOINTS_MAX;j++)
		Traj.Tgen.waypoints[0].pos.v[j] = pC->Joints[j].currentPos;
	
//	for(uint32_t j=0;j<JOINTS_MAX;j++)
//		Traj.Tgen.waypoints[0].pos[j] = 0;
	
	Traj.Tgen.waypoints[0].vel = 0.0;
	Traj.Tgen.waypoints[0].type = SPT_Start;
	Traj.Tgen.waypoints[0].moveType = SPMT_ConfSpacePtp;
	
	//pomocniczy waypoint bliko punktu startowego
	for(uint32_t j=0;j<JOINTS_MAX;j++)
		Traj.Tgen.waypoints[1].pos.v[j] = Traj.Tgen.waypoints[0].pos.v[j];
	Traj.Tgen.waypoints[1].vel = 0.2 * accmax;
	Traj.Tgen.waypoints[1].type = SPT_Way;
	Traj.Tgen.waypoints[1].moveType = Traj.Tgen.waypoints[0].moveType;
	
	for(uint32_t i=2;i<Traj.Tgen.maxwaypoints-1;i++)
	{
		if((eSeqPointMoveType)Mbs.hregs[idx] == SPMT_ConfSpacePtp)
		{
			Traj.Tgen.waypoints[i] = TG_GetSeqPointConfSpace(idx);
			idx += 15;
		}
		else if((eSeqPointMoveType)Mbs.hregs[idx] == SPMT_KartSpacePtp)
		{
			Traj.Tgen.waypoints[i] = TG_GetSeqPointKartesianSpace(idx);
			idx += 24;
		}
	}
	
	//pomocniczy waypoint na koncu
	for(uint32_t j=0;j<JOINTS_MAX;j++)
		Traj.Tgen.waypoints[Traj.Tgen.maxwaypoints-1].pos.v[j] = Traj.Tgen.waypoints[Traj.Tgen.maxwaypoints-2].pos.v[j];
	Traj.Tgen.waypoints[Traj.Tgen.maxwaypoints-1].vel = 0.2 * accmax;
	Traj.Tgen.waypoints[Traj.Tgen.maxwaypoints-1].type = SPT_Finish;
	Traj.Tgen.waypoints[Traj.Tgen.maxwaypoints-1].moveType = SPMT_ConfSpacePtp;
	
	//Sprawdzenie zadanych pozycji i predkosci pod katem limitów
	for(uint32_t i=0;i<Traj.Tgen.maxwaypoints;i++)
	{
		for(uint32_t j=0;j<JOINTS_MAX;j++)
		{
			if(Traj.Tgen.waypoints[i].pos.v[j] < pC->Joints[j].limitPosMin)
			{
				Traj.Tgen.trajPrepStatus = TPS_PosToLow;
				return false;
			}
			if(Traj.Tgen.waypoints[i].pos.v[j] > pC->Joints[j].limitPosMax)
			{
				Traj.Tgen.trajPrepStatus = TPS_PosToHigh;
				return false;
			}
		}
		
		if((i != 0) && (fabs(Traj.Tgen.waypoints[i].vel) < Traj.Tgen.minVelocity)) //nie sprawdzam predkosci dla punktu startowego
		{
			Traj.Tgen.trajPrepStatus = TPS_VelocityToLow;
			return false;
		}
		if(fabs(Traj.Tgen.waypoints[i].vel) > Traj.Tgen.maxVelocity)
		{
			Traj.Tgen.trajPrepStatus = TPS_VelocityToHigh;
			return false;
		}
	}
	Traj.Tgen.waypoints[Traj.Tgen.maxwaypoints].type = SPT_Finish;
	return true;
}
// ******************************** SLP Blending ****************************************
static double TG_SLP_GetPt12(double pt1, double py1, double pt2, double py2, double pt3, double py3, double acc)
{
	return (2.0*acc*(pt1 - pt2)*pt2*(pt2 - pt3) - pt3*py1 - pt1*py2 + pt3*py2 + pt2*(py1 - py3) + pt1*py3)/(2.0*acc*(pt1 - pt2)*(pt2 - pt3));
}
static double TG_SLP_q1(double pt1, double py1, double pt2, double py2, double pt3, double py3, double acc, double t)
{
	return (-(pt2*py1) + pt1*py2 + (py1 - py2)*t)/(pt1 - pt2);
}
static double TG_SLP_q2(double pt1, double py1, double pt2, double py2, double pt3, double py3, double acc, double t)
{
	return (pow(-(pt3*py1)-pt1*py2+pt3*py2+pt2*(py1-py3)+pt1*py3,2)/(acc*pow(pt1-pt2,2)*pow(pt2-pt3,2))+4.0*acc*pow(pt2-t,2)+(4.0*(-2.0*pt1*pt3*py2-pow(pt2,2)*(py1+py3)+(-(pt3*py1)+pt1*py2+pt3*py2-pt1*py3)*t+pt2*(pt3*(py1+py2)+pt1*(py2+py3)+(py1-2.0*py2+py3)*t)))/((pt1-pt2)*(pt2-pt3)))/8.0;
}
static double TG_SLP_q3(double pt1, double py1, double pt2, double py2, double pt3, double py3, double acc, double t)
{
	return (-(pt3*py2) + pt2*py3 + (py2 - py3)*t)/(pt2 - pt3);
}
static double TG_SLP_dq1(double pt1, double py1, double pt2, double py2, double pt3, double py3, double acc, double t)
{
	return (py1 - py2)/(pt1 - pt2);
}
static double TG_SLP_dq2(double pt1, double py1, double pt2, double py2, double pt3, double py3, double acc, double t)
{
	return ((4.0*(-(pt3*py1) + pt1*py2 + pt3*py2 - pt1*py3 + pt2*(py1 - 2.0*py2 + py3)))/((pt1 - pt2)*(pt2 - pt3)) + 8.0*acc*(-pt2 + t))/8.0;
}
static double TG_SLP_dq3(double pt1, double py1, double pt2, double py2, double pt3, double py3, double acc, double t)
{
	return (py2 - py3)/(pt2 - pt3);
}
static double TG_SLP_ddq1(double pt1, double py1, double pt2, double py2, double pt3, double py3, double acc, double t)
{
	return 0;
}
static double TG_SLP_ddq2(double pt1, double py1, double pt2, double py2, double pt3, double py3, double acc, double t)
{
	return acc;
}
static double TG_SLP_ddq3(double pt1, double py1, double pt2, double py2, double pt3, double py3, double acc, double t)
{
	return 0;
}

static void TG_SLP_FindTrace(int num)
{
	for(int i=0;i<Traj.Tgen.maxwaypoints;i++)
	{
		Traj.Tgen.trace[num][i][0] = 0; //time
		Traj.Tgen.trace[num][i][1] = Traj.Tgen.p1[i][num]; //pos
		Traj.Tgen.trace[num][i][2] = Traj.Tgen.p1[i][JOINTS_MAX]; //velocity
	}
	for(int i=1;i<Traj.Tgen.maxwaypoints;i++)
	{
		if((Traj.Tgen.trace[num][i][1] - Traj.Tgen.trace[num][i-1][1]) >= 0)
			Traj.Tgen.trace[num][i][2] = fabs(Traj.Tgen.trace[num][i][2]);
		else
			Traj.Tgen.trace[num][i][2] = -fabs(Traj.Tgen.trace[num][i][2]);
	}
	for(int i=1;i<Traj.Tgen.maxwaypoints;i++)
	{
		if(fabs(Traj.Tgen.trace[num][i][1] - Traj.Tgen.trace[num][i-1][1]) < 0.001)
			Traj.Tgen.trace[num][i][1] += 0.001;
	}
	
	for(int i=1;i<Traj.Tgen.maxwaypoints;i++)
	{
		double s03 = fabs(Traj.Tgen.trace[num][i][1] - Traj.Tgen.trace[num][i-1][1]);
		double tb = fabs(Traj.Tgen.trace[num][i][2]) / fabs(accmax);
		double sb = fabs(accmax)*tb*tb / 2.0;
		double t = 0.0;
		if((2.0*sb) < s03)
		{
			t = (fabs(Traj.Tgen.trace[num][i][1] - Traj.Tgen.trace[num][i-1][1]) - 2.0 * sb) / fabs(Traj.Tgen.trace[num][i][2]);
		}
		
		Traj.Tgen.trace[num][i][0] = Traj.Tgen.trace[num][i-1][0] + 2.0 * tb + t;
	}
}
static void TG_SLP_FindOneTimeStamps(int num)
{
	TG_SLP_FindTrace(num);
	
	for(int i=1;i<Traj.Tgen.maxwaypoints-1;i++)
	{
		double acc;
		double pt1 = Traj.Tgen.trace[num][i-1][0];
		double py1 = Traj.Tgen.trace[num][i-1][1];
		double pt2 = Traj.Tgen.trace[num][i][0];
		double py2 = Traj.Tgen.trace[num][i][1];
		double pt3 = Traj.Tgen.trace[num][i+1][0];
		double py3 = Traj.Tgen.trace[num][i+1][1];
		double pv3 = Traj.Tgen.trace[num][i+1][2];
		double a1 = (py1 - py2) / (pt1 - pt2);
		double a2 = (py2 - py3) / (pt2 - pt3);
		
		if(a2 > a1)
			acc = fabs(accmax);
		else
			acc = -fabs(accmax);
			
		double dvt = (a2 - a1) / acc;
		double pt12 = TG_SLP_GetPt12(pt1, py1, pt2, py2, pt3, py3, acc);
		double pt23 = pt12 + dvt;
		
		Traj.Tgen.stamps[num][i-1][0] = pt1;
		Traj.Tgen.stamps[num][i-1][1] = pt12;
		Traj.Tgen.stamps[num][i-1][2] = pt2;
		Traj.Tgen.stamps[num][i-1][3] = pt23;
		Traj.Tgen.stamps[num][i-1][4] = pt3;
		Traj.Tgen.stamps[num][i-1][5] = pv3;
	}
}
static bool TG_SLP_FindAllTimeStamps(void)
{
	double prec[TG_SEQWAYPOINTSSMAX];
	for(int num=0;num<JOINTS_MAX;num++)
	{
		for(int i=0;i<Traj.Tgen.maxwaypoints+1;i++)
		{
			for(int k=0;k<JOINTS_MAX;k++)
				Traj.Tgen.p1[i][k] = Traj.Tgen.waypoints[i].pos.v[k];
			
			Traj.Tgen.p1[i][JOINTS_MAX] = Traj.Tgen.waypoints[i].vel;
		}
		
		TG_SLP_FindOneTimeStamps(num);
		
		for(int i=0;i<Traj.Tgen.maxwaypoints-2;i++)
			prec[i] = 0.002 * Traj.Tgen.stamps[num][i][5];
		
		//***************************************************************
		for(int i=0;i<Traj.Tgen.maxwaypoints-3;i++)
		{
			for(int j=0;j<1000;j++)
			{
				double pt1 = Traj.Tgen.stamps[num][i][0];
				double pt12 = Traj.Tgen.stamps[num][i][1];
				double pt2 = Traj.Tgen.stamps[num][i][2];
				double pt23 = Traj.Tgen.stamps[num][i][3];
				double pt3 = Traj.Tgen.stamps[num][i][4];
				if(pt1 > pt12 || pt12 > pt2 || pt2 > pt23 || pt23 > pt3)
				{
					if(Traj.Tgen.p1[i][JOINTS_MAX] > 1.1 * prec[i])
					{
						Traj.Tgen.p1[i][JOINTS_MAX] -= prec[i];
					}
					if(Traj.Tgen.p1[i+1][JOINTS_MAX] > 1.1 * prec[i+1])
					{
						Traj.Tgen.p1[i+1][JOINTS_MAX] -= prec[i+1];
					}
					if(Traj.Tgen.p1[i+2][JOINTS_MAX] > 1.1 * prec[i+1])
					{
						Traj.Tgen.p1[i+2][JOINTS_MAX] -= prec[i+1];
					}
					TG_SLP_FindOneTimeStamps(num);

				}
				else
				{
					break;
				}
			}
		}
		//***************************************************************
		int i = Traj.Tgen.maxwaypoints-3;
		for(int j=0;j<1000;j++)
		{
			double pt1 = Traj.Tgen.stamps[num][i][0];
			double pt12 = Traj.Tgen.stamps[num][i][1];
			double pt2 = Traj.Tgen.stamps[num][i][2];
			double pt23 = Traj.Tgen.stamps[num][i][3];
			double pt3 = Traj.Tgen.stamps[num][i][4];
			if(pt1 > pt12 || pt12 > pt2 || pt2 > pt23 || pt23 > pt3)
			{
				if(Traj.Tgen.p1[i+1][JOINTS_MAX] > 1.1 * prec[i])
					Traj.Tgen.p1[i+1][JOINTS_MAX] -= prec[i];
				if(Traj.Tgen.p1[i+2][JOINTS_MAX] > 1.1 * prec[i])
					Traj.Tgen.p1[i+2][JOINTS_MAX] -= prec[i];
				TG_SLP_FindOneTimeStamps(num);
			}
			else
			{
				break;
			}
		}
		//***************************************************************
		for(int i=0;i<Traj.Tgen.maxwaypoints-3;i++)
		{
			for(int j=0;j<1000;j++)
			{
				double pt23a = Traj.Tgen.stamps[num][i][3];
				double pt23b = Traj.Tgen.stamps[num][i+1][1];
				if(pt23a > pt23b)
				{
					if(Traj.Tgen.p1[i+2][JOINTS_MAX] > 1.1 * prec[i+1])
						Traj.Tgen.p1[i+2][JOINTS_MAX] -= prec[i+1];
					TG_SLP_FindOneTimeStamps(num);
				}
				else
				{
					break;
				}
			}
		}
		
		//*************************************************************** 
		//sprawdzenie czasów poszczególnych waypointów
		for(int i=0;i<Traj.Tgen.maxwaypoints-3;i++)
		{
			double pt1 = Traj.Tgen.stamps[num][i][0];
			double pt12 = Traj.Tgen.stamps[num][i][1];
			double pt2 = Traj.Tgen.stamps[num][i][2];
			double pt23 = Traj.Tgen.stamps[num][i][3];
			double pt3 = Traj.Tgen.stamps[num][i][4];
			if(pt1 > pt12 || pt12 > pt2 || pt2 > pt23 || pt23 > pt3)
			{
				return false;
			}
		}
		for(int i=0;i<Traj.Tgen.maxwaypoints-3;i++)
		{
			double pt23a = Traj.Tgen.stamps[num][i][3];
			double pt23b = Traj.Tgen.stamps[num][i+1][1];
			if(pt23a > pt23b)
			{
				return false;
			}
		}
		
	}
	return true;
}
static bool TG_SLP_FindPath(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		for(int i=0;i<Traj.Tgen.maxwaypoints-2;i++)
		{
			Traj.Tgen.path[num][i][0] = Traj.Tgen.stamps[num][i][0];
			Traj.Tgen.path[num][i][1] = Traj.Tgen.trace[num][i][1];
		}
		Traj.Tgen.path[num][Traj.Tgen.maxwaypoints-2][0] = Traj.Tgen.stamps[num][Traj.Tgen.maxwaypoints-3][2];
		Traj.Tgen.path[num][Traj.Tgen.maxwaypoints-2][1] = Traj.Tgen.trace[num][Traj.Tgen.maxwaypoints-2][1];
			
		Traj.Tgen.path[num][Traj.Tgen.maxwaypoints-1][0] = Traj.Tgen.stamps[num][Traj.Tgen.maxwaypoints-3][4];
		Traj.Tgen.path[num][Traj.Tgen.maxwaypoints-1][1] = Traj.Tgen.trace[num][Traj.Tgen.maxwaypoints-1][1];
	}
	double maxTimes[TG_SEQWAYPOINTSSMAX];
	for(int i=1;i<Traj.Tgen.maxwaypoints;i++)
	{
		double max = Traj.Tgen.path[0][i][0] - Traj.Tgen.path[0][i-1][0];
		for(int num=1;num<JOINTS_MAX;num++)
		{
			if((Traj.Tgen.path[num][i][0] - Traj.Tgen.path[num][i-1][0]) > max)
				max = Traj.Tgen.path[num][i][0] - Traj.Tgen.path[num][i-1][0];
		}
		maxTimes[i] = max;
	}
	for(int i=1;i<Traj.Tgen.maxwaypoints;i++)
	{
		for(int num=0;num<JOINTS_MAX;num++)
		{
			Traj.Tgen.path[num][i][0] = Traj.Tgen.path[num][i-1][0] + maxTimes[i];
		}
	}
	return true;
}
static bool TG_SLP_FindTraj(int num)
{
	uint32_t idx = 0;
	double step = Traj.Tgen.stepTime;
	double stamps[JOINTS_MAX][TG_SEQWAYPOINTSSMAX][9];
	double pt1, py1, pt2, py2, pt3, py3, pt12, pt23, dvt, a1, a2, acc, t, tlstart, tlstop, tpstart, tpstop;
	
	for(int i=0;i<Traj.Tgen.maxwaypoints-2;i++)
	{
		pt1 = Traj.Tgen.path[num][i][0];
		py1 = Traj.Tgen.path[num][i][1];
		pt2 = Traj.Tgen.path[num][i+1][0];
		py2 = Traj.Tgen.path[num][i+1][1];
		pt3 = Traj.Tgen.path[num][i+2][0];
		py3 = Traj.Tgen.path[num][i+2][1];
		a1 = (py1 - py2) / (pt1 - pt2);
		a2 = (py2 - py3) / (pt2 - pt3);
		
		if(a2 > a1)
			acc = fabs(accmax);
		else
			acc = -fabs(accmax);
			
		dvt = (a2 - a1) / acc;
		pt12 = TG_SLP_GetPt12(pt1, py1, pt2, py2, pt3, py3, acc);
		pt23 = pt12 + dvt;
		
		stamps[num][i][0] = pt1;
		stamps[num][i][1] = pt12;
		stamps[num][i][2] = pt2;
		stamps[num][i][3] = pt23;
		stamps[num][i][4] = pt3;
		stamps[num][i][5] = py1;
		stamps[num][i][6] = py2;
		stamps[num][i][7] = py3;
		stamps[num][i][8] = acc;
	}
	
	//Wpisywanie wartosci do trajektorii
	
	// Pierwszy odcinek liniowy
	tlstart = stamps[num][0][0];
	tlstop = stamps[num][0][1];
	pt1 = stamps[num][0][0];
	py1 = stamps[num][0][5];
	pt2 = stamps[num][0][2];
	py2 = stamps[num][0][6];
	pt3 = stamps[num][0][4];
	py3 = stamps[num][0][7];
	acc = stamps[num][0][8];
	for(int j=((tlstart+step)/step); j<(tlstop/step); j++)
	{
		t = step*(double)j;
		Traj.points[idx].pos[num] = TG_SLP_q1(pt1, py1, pt2, py2, pt3, py3, acc, t) / pC->Joints[num].limitPosMax * MAXINT16;
		Traj.points[idx].vel[num] = TG_SLP_dq1(pt1, py1, pt2, py2, pt3, py3, acc, t) / pC->Joints[num].limitVelMax * MAXINT16;
		Traj.points[idx].acc[num] = TG_SLP_ddq1(pt1, py1, pt2, py2, pt3, py3, acc, t) / pC->Joints[num].limitAccMax * MAXINT16;
		idx++;
		if(idx >= TRAJ_POINTSMAX)
			return false;
	}

	// czesc srodkowa
	for(int i=0;i<Traj.Tgen.maxwaypoints-3;i++)
	{
		tpstart = stamps[num][i][1];
		tpstop = stamps[num][i][3];
		tlstart = tpstop;
		tlstop = stamps[num][i+1][1];
		pt1 = stamps[num][i][0];
		py1 = stamps[num][i][5];
		pt2 = stamps[num][i][2];
		py2 = stamps[num][i][6];
		pt3 = stamps[num][i][4];
		py3 = stamps[num][i][7];
		acc = stamps[num][i][8];
		for(int j=((tpstart+step)/step); j<(tpstop/step); j++)
		{
			t = step*(double)j;
			Traj.points[idx].pos[num] = TG_SLP_q2(pt1, py1, pt2, py2, pt3, py3, acc, t) / pC->Joints[num].limitPosMax * MAXINT16;
			Traj.points[idx].vel[num] = TG_SLP_dq2(pt1, py1, pt2, py2, pt3, py3, acc, t) / pC->Joints[num].limitVelMax * MAXINT16;
			Traj.points[idx].acc[num] = TG_SLP_ddq2(pt1, py1, pt2, py2, pt3, py3, acc, t) / pC->Joints[num].limitAccMax * MAXINT16;
			idx++;
			if(idx >= TRAJ_POINTSMAX)
			return false;
		}
		for(int j=((tlstart+step)/step); j<(tlstop/step); j++)
		{
			t = step*(double)j;
			Traj.points[idx].pos[num] = TG_SLP_q3(pt1, py1, pt2, py2, pt3, py3, acc, t) / pC->Joints[num].limitPosMax * MAXINT16;
			Traj.points[idx].vel[num] = TG_SLP_dq3(pt1, py1, pt2, py2, pt3, py3, acc, t) / pC->Joints[num].limitVelMax * MAXINT16;
			Traj.points[idx].acc[num] = TG_SLP_ddq3(pt1, py1, pt2, py2, pt3, py3, acc, t) / pC->Joints[num].limitAccMax * MAXINT16;
			idx++;
			if(idx >= TRAJ_POINTSMAX)
			return false;
		}
	}
	
	
	// Ostatni odcinek paraboli i linii
	int i = Traj.Tgen.maxwaypoints-3;
	tpstart = stamps[num][i][1];
	tpstop = stamps[num][i][3];
	tlstart = tpstop;
	tlstop = stamps[num][i][4];
	pt1 = stamps[num][i][0];
	py1 = stamps[num][i][5];
	pt2 = stamps[num][i][2];
	py2 = stamps[num][i][6];
	pt3 = stamps[num][i][4];
	py3 = stamps[num][i][7];
	acc = stamps[num][i][8];
	for(int j=((tpstart+step)/step); j<(tpstop/step); j++)
	{
		t = step*(double)j;
		Traj.points[idx].pos[num] = TG_SLP_q2(pt1, py1, pt2, py2, pt3, py3, acc, t) / pC->Joints[num].limitPosMax * MAXINT16;
		Traj.points[idx].vel[num] = TG_SLP_dq2(pt1, py1, pt2, py2, pt3, py3, acc, t) / pC->Joints[num].limitVelMax * MAXINT16;
		Traj.points[idx].acc[num] = TG_SLP_ddq2(pt1, py1, pt2, py2, pt3, py3, acc, t) / pC->Joints[num].limitAccMax * MAXINT16;
		idx++;
		if(idx >= TRAJ_POINTSMAX)
			return false;
	}
	for(int j=((tlstart+step)/step); j<(tlstop/step); j++)
	{
		t = step*(double)j;
		Traj.points[idx].pos[num] = TG_SLP_q3(pt1, py1, pt2, py2, pt3, py3, acc, t) / pC->Joints[num].limitPosMax * MAXINT16;
		Traj.points[idx].vel[num] = TG_SLP_dq3(pt1, py1, pt2, py2, pt3, py3, acc, t) / pC->Joints[num].limitVelMax * MAXINT16;
		Traj.points[idx].acc[num] = TG_SLP_ddq3(pt1, py1, pt2, py2, pt3, py3, acc, t) / pC->Joints[num].limitAccMax * MAXINT16;
		idx++;
		if(idx >= TRAJ_POINTSMAX)
			return false;
	}
	return true;
}
void TG_TrajGen(void)
{
	if(pC->Jtc.currentFsm != JTC_FSM_HoldPos && pC->Jtc.currentFsm != JTC_FSM_Operate)
		return;
	if(Traj.Tgen.reqTrajPrepare == false)
		return;

	TG_SetDefaultVariables();
	
	if(TG_GetSeqFromMbs() == false)
	{
		Control_TrajClear();
		return;
	}
	Control_TrajClear();
	pC->tick = 0;
	
	if(TG_SLP_FindAllTimeStamps() == false)
	{
		Traj.Tgen.trajPrepStatus = TPS_UnknownError;
		Control_TrajClear();
		return;
	}
	
	if(TG_SLP_FindPath() == false)
	{
		Traj.Tgen.trajPrepStatus = TPS_UnknownError;
		Control_TrajClear();
		return;
	}
	
	//Sprawdzenie dlugosci calej trajektorii
	double totalTime = Traj.Tgen.path[0][Traj.Tgen.maxwaypoints-1][0];
	double trajLen = totalTime / Traj.Tgen.stepTime;
	if(trajLen > TRAJ_POINTSMAX)
	{
		Traj.Tgen.trajPrepStatus = TPS_TrajToLong;
		Control_TrajClear();
		return;
	}
	
	//Generowanie punktów trajektorii
	Traj.Tgen.maxpoints = trajLen;
	for(int num=0;num<JOINTS_MAX;num++)
		if(TG_SLP_FindTraj(num) == false)
		{
			Traj.Tgen.trajPrepStatus = TPS_UnknownError;
			Control_TrajClear();
			return;
		}
	
	Traj.Tgen.status = TGS_Ready;
	Traj.stepTime = 1000.0  * Traj.Tgen.stepTime; //W Tgen stepTime jest w sekundach (def 0.01), a w Traj stepTime jest w ilosci ms na punkt (def 10)
	Traj.numRecPoints = Traj.Tgen.maxpoints;
	Traj.comStatus = TCS_WasRead;
	Traj.targetTES = TES_Stop;
	Traj.Tgen.reqTrajPrepare = false;
	Traj.Tgen.trajPrepStatus = TPS_Ok;
}
