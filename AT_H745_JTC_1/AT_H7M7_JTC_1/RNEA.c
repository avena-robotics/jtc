#include "RNEA.h"
extern sControl* pC;

const int DOF = 6;
volatile sVector6 mxDim[DOF+1];
volatile sVector6 mxcDim[DOF];
volatile sVector6 m;
volatile sVector3 g0;
volatile sVector6 MIvec[DOF];
volatile sMatrix3 MI[DOF];
volatile sMatrix4 Mxtemp[DOF+1];
volatile sMatrix4 Mx[DOF+1];
volatile sMatrix4 Mxc[DOF+1];
volatile sMatrix3 Rx[DOF+1];
volatile sMatrix3 Rxc[DOF+1];
volatile sMatrix3 R[DOF+1];
volatile sMatrix3 Rc[DOF+1];
volatile sMatrix3 Rxt[DOF+1]; // transpose of Rx
volatile sMatrix3 Rxct[DOF+1];  // transpose of Rxc
volatile sMatrix3 Rt[DOF+1];  // transpose of R
volatile sMatrix3 Rct[DOF+1];  // transpose of Rc
volatile sVector3 g[DOF+1];
volatile sVector3 r[DOF];
volatile sVector3 rc[DOF];
volatile sVector3 rcinv[DOF];

static sVector3 Vec3Zeros(void)
{
	sVector3 t;
	for(int i=0;i<3;i++)
		t.v[i] = 0.0;
	return t;
}
static sMatrix3 Mat3Ones(void)
{
	sMatrix3 t;
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			if(i==j)
				t.v[i][j] = 1.0;
			else
				t.v[i][j] = 0.0;
	return t;
}
static sMatrix4 Mat4Ones(void)
{
	sMatrix4 t;
	for(int i=0;i<4;i++)
		for(int j=0;j<4;j++)
			if(i==j)
				t.v[i][j] = 1.0;
			else
				t.v[i][j] = 0.0;
	return t;
}
static sMatrix4 HT(double x, double y, double z)
{
	sMatrix4 t;
	for(int i=0;i<4;i++)
		for(int j=0;j<4;j++)
			if(i==j)
				t.v[i][j] = 1.0;
			else
				t.v[i][j] = 0.0;
		
	t.v[0][3] = x;
	t.v[1][3] = y;
	t.v[2][3] = z;
	
	return t;
}
static sMatrix4 HRX(double alpha)
{
	sMatrix4 t;
	for(int i=0;i<4;i++)
		for(int j=0;j<4;j++)
			if(i==j)
				t.v[i][j] = 1.0;
			else
				t.v[i][j] = 0.0;
		
	t.v[1][1] = cos(alpha);
	t.v[1][2] = -sin(alpha);
	t.v[2][1] = sin(alpha);
	t.v[2][2] = cos(alpha);
	
	return t;
}
static sMatrix4 HRY(double betha)
{
	sMatrix4 t;
	for(int i=0;i<4;i++)
		for(int j=0;j<4;j++)
			if(i==j)
				t.v[i][j] = 1.0;
			else
				t.v[i][j] = 0.0;
		
	t.v[0][0] = cos(betha);
	t.v[0][2] = sin(betha);
	t.v[2][0] = -sin(betha);
	t.v[2][2] = cos(betha);
	
	return t;
}
static sMatrix4 HRZ(double gamma)
{
	sMatrix4 t;
	for(int i=0;i<4;i++)
		for(int j=0;j<4;j++)
			if(i==j)
				t.v[i][j] = 1.0;
			else
				t.v[i][j] = 0.0;
		
	t.v[0][0] = cos(gamma);
	t.v[0][1] = -sin(gamma);
	t.v[1][0] = sin(gamma);
	t.v[1][1] = cos(gamma);
	
	return t;
}
static sMatrix3 Mat3Transpose(sMatrix3 m)
{
	sMatrix3 t;
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			t.v[j][i] = m.v[i][j];
	return t;
}
static sMatrix3 Mat3xMat3(sMatrix3 m1, sMatrix3 m2)
{
	sMatrix3 t;
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			t.v[i][j]=0;
			for(int k=0;k<3;k++)
				t.v[i][j] += m1.v[i][k] * m2.v[k][j];
		}
	}
	return t;
}
static sMatrix4 Mat4xMat4(sMatrix4 m1, sMatrix4 m2)
{
	sMatrix4 t;
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			t.v[i][j]=0;
			for(int k=0;k<4;k++)
				t.v[i][j] += m1.v[i][k] * m2.v[k][j];
		}
	}
	return t;
}
static sVector3 Mat3xVec3(sMatrix3 m, sVector3 v)
{
	sVector3 t;
	for(int i=0;i<3;i++)
	{
		t.v[i]=0;
		for(int j=0;j<3;j++)
			t.v[i] += m.v[i][j] * v.v[j];
	}
	return t;
}
static sMatrix3 Mat4ToMat3(sMatrix4 m)
{
	sMatrix3 t;
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			t.v[i][j] = m.v[i][j];
	return t;
}
static sVector3 Mat4ToVec3(sMatrix4 m)
{
	sVector3 t;
	for(int i=0;i<3;i++)
			t.v[i] = m.v[i][3];
	return t;
}
static sVector3 Mat3ToVec3(sMatrix3 m)
{
	sVector3 t;
	for(int i=0;i<3;i++)
			t.v[i] = m.v[i][2];
	return t;
}
static sVector3 Vec3xScalar(sVector3 m, double s)
{
	sVector3 t;
	for(int i=0;i<3;i++)
		t.v[i] = m.v[i] * s;
	return t;
}
static sVector3 Vec3Cross(sVector3 m1, sVector3 m2)
{
	sVector3 t;
	t.v[0] = m1.v[1]*m2.v[2]-m1.v[2]*m2.v[1]; 
	t.v[1] = m1.v[2]*m2.v[0]-m1.v[0]*m2.v[2]; 
	t.v[2] = m1.v[0]*m2.v[1]-m1.v[1]*m2.v[0];
	return t;
}
static sVector3 Vec3AddVec3(sVector3 m1, sVector3 m2)
{
	sVector3 t;
	for(int i=0;i<3;i++)
		t.v[i] = m1.v[i] + m2.v[i];
	return t;
}
static sVector3 Vec3SubVec3(sVector3 m1, sVector3 m2)
{
	sVector3 t;
	for(int i=0;i<3;i++)
		t.v[i] = m1.v[i] - m2.v[i];
	return t;
}
static sVector3 Vec3SetValues(double a0, double a1, double a2)
{
	sVector3 t;
	t.v[0] = a0; t.v[1] = a1; t.v[2] = a2;
	return t;
}
void RNEA_Conf(void)
{
	for(int i=0;i<DOF+1;i++)
	{
		Mx[i] = Mat4Ones();
		Mxc[i] = Mat4Ones();
		R[i] = Mat3Ones();
		Rc[i] = Mat3Ones();
	}
	
	g0 = Vec3SetValues(0, 0, 9.81); //global gravity vector
	
	for(int i=0;i<DOF+1;i++)
		for(int j=0;j<6;j++)
			mxDim[i].v[j] = pC->Arm.Joints[i].origin[j];
	
	
	for(int i=0;i<DOF;i++)
	{
		for(int j=0;j<6;j++)
		{
			mxcDim[i].v[j] = pC->Arm.Links[i+1].origin[j]; // Skip Link 0 inertial parameters
			MIvec[i].v[j] = pC->Arm.Links[i+1].innertia[j]; // Skip Link 0 inertial parameters
		}
		m.v[i] = pC->Arm.Links[i+1].mass; // Skip Link 0 mass parameters
	}
	
	//link i innertial matrix
	for(int i=0;i<DOF;i++) 
	{
		MI[i].v[0][0] = MIvec[i].v[0];
		MI[i].v[0][1] = MIvec[i].v[1];
		MI[i].v[0][2] = MIvec[i].v[2];
		MI[i].v[1][0] = MIvec[i].v[1];
		MI[i].v[1][1] = MIvec[i].v[3];
		MI[i].v[1][2] = MIvec[i].v[4];
		MI[i].v[2][0] = MIvec[i].v[2];
		MI[i].v[2][1] = MIvec[i].v[4];
		MI[i].v[2][2] = MIvec[i].v[5];
	}
	
	for(int i=0;i<DOF+1;i++) //partial homogeneous transformation matrix for joints coordinate systems without angular position
	{
		Mxtemp[i] = HT(mxDim[i].v[0], mxDim[i].v[1], mxDim[i].v[2]);
		Mxtemp[i] = Mat4xMat4(Mxtemp[i], HRZ(mxDim[i].v[5]));
		Mxtemp[i] = Mat4xMat4(Mxtemp[i], HRY(mxDim[i].v[4]));
		Mxtemp[i] = Mat4xMat4(Mxtemp[i], HRX(mxDim[i].v[3]));
	}
	
	for(int i=0;i<DOF;i++) //partial homogeneous transformation matrix for links center of mass coordinate systems
	{
		Mxc[i] = HT(mxcDim[i].v[0], mxcDim[i].v[1], mxcDim[i].v[2]);
		Mxc[i] = Mat4xMat4(Mxc[i], HRZ(mxcDim[i].v[5]));
		Mxc[i] = Mat4xMat4(Mxc[i], HRY(mxcDim[i].v[4]));
		Mxc[i] = Mat4xMat4(Mxc[i], HRX(mxcDim[i].v[3]));
		Rxc[i] = Mat4ToMat3(Mxc[i]); //partial rotation matrix for links center of mass coordinate systems
		r[i] = Mat4ToVec3(Mxtemp[i+1]); // vector from joint i to joint i+1 defined in joint i coordinate systems
		rc[i] = Mat4ToVec3(Mxc[i]); // vector from joint i to link i center of mass defined in joint i coordinate systems
		rcinv[i] = Vec3SubVec3(rc[i], r[i]); // vector from joint i+1 to link i center of mass defined in joint i coordinate systems
		Rxct[i] = Mat3Transpose(Rxc[i]);
	}
}
void RNEA_CalcTorques(void)
{
	sVector6 q;
	sVector6 qq;
	sVector6 qqq;
	sVector3 z[DOF+1];
	sVector3 w[DOF+1];
	sVector3 a[DOF+1];
	sVector3 ae[DOF+1];
	sVector3 ac[DOF+1];
	sVector3 f[DOF+1];
	sVector3 tau[DOF+1];
	
	for(int i=0;i<DOF;i++)
	{
		q.v[i] = pC->Joints[i].idSetPos;
		qq.v[i] = pC->Joints[i].idSetVel;
		qqq.v[i] = pC->Joints[i].idSetAcc;
	}

	for(int i=0;i<DOF;i++) 
	{
		Mx[i] = Mat4xMat4(Mxtemp[i], HRZ(q.v[i])); //partial homogeneous transformation matrix for joints coordinate systems with angular position
	}
	Mx[DOF] = Mxtemp[DOF];

	for(int i=0;i<DOF+1;i++)
	{
		Rx[i] = Mat4ToMat3(Mx[i]);  //partial rotation matrix for joints coordinate systems
	}
	
	for(int i=0;i<DOF+1;i++)
		R[i] = Mat3Ones();

	for(int i=0;i<DOF+1;i++)
		for(int j=0;j<=i;j++)
			R[i] = Mat3xMat3(R[i], Rx[j]); //rotation matrix for joints coordinate systems
	
	for(int i=0;i<DOF+1;i++)
	{
		Rc[i] = Mat3xMat3(R[i], Rxc[i]); //rotation matrix for links center of mass coordinate systems
		Rxt[i] = Mat3Transpose(Rx[i]);
		Rt[i] = Mat3Transpose(R[i]);
		Rct[i] = Mat3Transpose(Rc[i]);
		g[i] = Mat3xVec3(Mat3Transpose(R[i]), g0); // gravity vector defined in joint i coordinate systems
		z[i] = Mat3ToVec3(R[i]); //vector "z" is last column of rotation matrix.
	}
	// ************************************************************************************************************************************
	for(int i=0;i<DOF;i++)
	{
		w[i] = Vec3xScalar(Mat3xVec3(Rt[i], z[i]), qq.v[i]);
		if(i > 0)
			w[i] = Vec3AddVec3(Mat3xVec3(Rxt[i], w[i-1]), w[i]);
			
		a[i] = Vec3AddVec3(Mat3xVec3(Rt[i], Vec3xScalar(z[i], qqq.v[i])),  Vec3Cross(w[i], Vec3xScalar(Mat3xVec3(Rt[i], z[i]), qq.v[i])));
		if(i > 0)
			a[i] = Vec3AddVec3(Mat3xVec3(Rxt[i], a[i-1]), a[i]);
			
		ae[i] = Vec3AddVec3(Vec3Cross(a[i], r[i]), Vec3Cross(w[i], Vec3Cross(w[i], r[i])));
		if(i > 0)
			ae[i] = Vec3AddVec3(Mat3xVec3(Rxt[i], ae[i-1]), ae[i]);
			
		ac[i] = Vec3AddVec3(Vec3Cross(a[i], rc[i]), Vec3Cross(w[i], Vec3Cross(w[i], rc[i])));
		if(i > 0)
			ac[i] = Vec3AddVec3(Mat3xVec3(Rxt[i], ae[i-1]), ac[i]);
	}
	
	f[DOF] = Vec3Zeros();
	tau[DOF] = Vec3Zeros();
	for(int i=DOF-1;i>=0;i--)
	{
		f[i] = Vec3SubVec3(Vec3AddVec3(Mat3xVec3(Rx[i+1], f[i+1]), Vec3xScalar(ac[i], m.v[i])), Vec3xScalar(g[i], m.v[i]));
	}
	for(int i=DOF-1;i>=0;i--)
	{
		tau[i] = Vec3SubVec3(Mat3xVec3(Rx[i+1], tau[i+1]), Vec3Cross(f[i], rc[i]));
		tau[i] = Vec3AddVec3(tau[i], Vec3Cross(Mat3xVec3(Rx[i+1], f[i+1]), rcinv[i]));
		tau[i] = Vec3AddVec3(tau[i], Mat3xVec3(Rxc[i], Mat3xVec3(MI[i], Mat3xVec3(Mat3Transpose(Rxc[i]), a[i]))));
		tau[i] = Vec3AddVec3(tau[i], Mat3xVec3(Rxc[i], Vec3Cross(Mat3xVec3(Mat3Transpose(Rxc[i]), w[i]), Mat3xVec3(MI[i], Mat3xVec3(Mat3Transpose(Rxc[i]), w[i])))));
	}
	
	for(int i=0;i<DOF;i++)
		pC->Joints[i].idTorque = tau[i].v[2];
}
