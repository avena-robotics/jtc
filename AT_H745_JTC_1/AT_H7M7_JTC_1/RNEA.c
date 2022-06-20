#include "RNEA.h"
extern sControl* pC;
// general function for matrix and vectors
sVector3 Vec3Zeros(void)
{
	sVector3 t;
	for(int i=0;i<3;i++)
		t.v[i] = 0.0;
	return t;
}
sVector4 Vec4Zeros(void)
{
	sVector4 t;
	for(int i=0;i<4;i++)
		t.v[i] = 0.0;
	return t;
}
sVector6 Vec6Zeros(void)
{
	sVector6 t;
	for(int i=0;i<6;i++)
		t.v[i] = 0.0;
	return t;
}
sMatrix3 Mat3Ones(void)
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
sMatrix4 Mat4Ones(void)
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
sMatrix4 HT(double x, double y, double z)
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
sMatrix4 HRX(double alpha)
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
sMatrix4 HRY(double betha)
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
sMatrix4 HRZ(double gamma)
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
sMatrix3 Mat3xMat3(sMatrix3 m1, sMatrix3 m2)
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
sMatrix4 Mat4xMat4(sMatrix4 m1, sMatrix4 m2)
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
sMatrix3 Mat4ToMat3(sMatrix4 m)
{
	sMatrix3 t;
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			t.v[i][j] = m.v[i][j];
	return t;
}
sMatrix4 Mat3ToMat4(sMatrix3 m3, sMatrix4 m4)
{
	sMatrix4 t = m4;
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			t.v[i][j] = m3.v[i][j];
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
sVector6 Vec6SetValues(double a0, double a1, double a2, double a3, double a4, double a5)
{
	sVector6 t;
	t.v[0] = a0; t.v[1] = a1; t.v[2] = a2; t.v[3] = a3; t.v[4] = a4; t.v[5] = a5;
	return t;
}
static sMatrix3 InvMat3(sMatrix3 m)
{
	sMatrix3 out;
	double m00 = m.v[0][0], m01 = m.v[0][1], m02 = m.v[0][2];
	double m10 = m.v[1][0], m11 = m.v[1][1], m12 = m.v[1][2];
	double m20 = m.v[2][0], m21 = m.v[2][1], m22 = m.v[2][2];
	out.v[0][0] = 	(m12*m21-m11*m22)/(m02*m11*m20-m01*m12*m20-m02*m10*m21+m00*m12*m21+m01*m10*m22-m00*m11*m22);
	out.v[0][1] = 	(m02*m21-m01*m22)/(-(m02*m11*m20)+m01*m12*m20+m02*m10*m21-m00*m12*m21-m01*m10*m22+m00*m11*m22);
	out.v[0][2] = 	(m02*m11-m01*m12)/(m02*m11*m20-m01*m12*m20-m02*m10*m21+m00*m12*m21+m01*m10*m22-m00*m11*m22);
	out.v[1][0] = 	(m12*m20-m10*m22)/(-(m02*m11*m20)+m01*m12*m20+m02*m10*m21-m00*m12*m21-m01*m10*m22+m00*m11*m22);
	out.v[1][1] = 	(m02*m20-m00*m22)/(m02*m11*m20-m01*m12*m20-m02*m10*m21+m00*m12*m21+m01*m10*m22-m00*m11*m22);
	out.v[1][2] = 	(m02*m10-m00*m12)/(-(m02*m11*m20)+m01*m12*m20+m02*m10*m21-m00*m12*m21-m01*m10*m22+m00*m11*m22);
	out.v[2][0] = 	(m11*m20-m10*m21)/(m02*m11*m20-m01*m12*m20-m02*m10*m21+m00*m12*m21+m01*m10*m22-m00*m11*m22);
	out.v[2][1] = 	(m01*m20-m00*m21)/(-(m02*m11*m20)+m01*m12*m20+m02*m10*m21-m00*m12*m21-m01*m10*m22+m00*m11*m22);
	out.v[2][2] = 	(m01*m10-m00*m11)/(m02*m11*m20-m01*m12*m20-m02*m10*m21+m00*m12*m21+m01*m10*m22-m00*m11*m22);
	return out;
}
sMatrix4 InvMat4(sMatrix4 m)
{
	sMatrix4 out = Mat4Ones();
	double m00 = m.v[0][0], m01 = m.v[0][1], m02 = m.v[0][2], m03 = m.v[0][3];
	double m10 = m.v[1][0], m11 = m.v[1][1], m12 = m.v[1][2], m13 = m.v[1][3];
	double m20 = m.v[2][0], m21 = m.v[2][1], m22 = m.v[2][2], m23 = m.v[2][3];
	out.v[0][0] = 	(m12*m21-m11*m22)/(m02*m11*m20-m01*m12*m20-m02*m10*m21+m00*m12*m21+m01*m10*m22-m00*m11*m22);
	out.v[0][1] = 	(m02*m21-m01*m22)/(-(m02*m11*m20)+m01*m12*m20+m02*m10*m21-m00*m12*m21-m01*m10*m22+m00*m11*m22);
	out.v[0][2] = 	(m02*m11-m01*m12)/(m02*m11*m20-m01*m12*m20-m02*m10*m21+m00*m12*m21+m01*m10*m22-m00*m11*m22);
	out.v[0][3] = 	(m03*m12*m21-m02*m13*m21-m03*m11*m22+m01*m13*m22+m02*m11*m23-m01*m12*m23)/(-(m02*m11*m20)+m01*m12*m20+m02*m10*m21-m00*m12*m21-m01*m10*m22+m00*m11*m22);
	out.v[1][0] = 	(m12*m20-m10*m22)/(-(m02*m11*m20)+m01*m12*m20+m02*m10*m21-m00*m12*m21-m01*m10*m22+m00*m11*m22);
	out.v[1][1] = 	(m02*m20-m00*m22)/(m02*m11*m20-m01*m12*m20-m02*m10*m21+m00*m12*m21+m01*m10*m22-m00*m11*m22);
	out.v[1][2] = 	(m02*m10-m00*m12)/(-(m02*m11*m20)+m01*m12*m20+m02*m10*m21-m00*m12*m21-m01*m10*m22+m00*m11*m22);
	out.v[1][3] = 	(m03*m12*m20-m02*m13*m20-m03*m10*m22+m00*m13*m22+m02*m10*m23-m00*m12*m23)/(m02*m11*m20-m01*m12*m20-m02*m10*m21+m00*m12*m21+m01*m10*m22-m00*m11*m22);
	out.v[2][0] = 	(m11*m20-m10*m21)/(m02*m11*m20-m01*m12*m20-m02*m10*m21+m00*m12*m21+m01*m10*m22-m00*m11*m22);
	out.v[2][1] = 	(m01*m20-m00*m21)/(-(m02*m11*m20)+m01*m12*m20+m02*m10*m21-m00*m12*m21-m01*m10*m22+m00*m11*m22);
	out.v[2][2] = 	(m01*m10-m00*m11)/(m02*m11*m20-m01*m12*m20-m02*m10*m21+m00*m12*m21+m01*m10*m22-m00*m11*m22);
	out.v[2][3] = 	(m03*m11*m20-m01*m13*m20-m03*m10*m21+m00*m13*m21+m01*m10*m23-m00*m11*m23)/(-(m02*m11*m20)+m01*m12*m20+m02*m10*m21-m00*m12*m21-m01*m10*m22+m00*m11*m22);
	return out;
}

// ------------------ Inverse dynamic by RNEA ------------------
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


// ------------------ Inverse and forward kinematic  ------------------
double qq1, d1, a2, a3, d4, d5, d6;
sVector6 offset;
sVector6 dir;
static bool Kin_SolIsReal(sVector6 v)
{
	for(int j=0;j<JOINTS_MAX;j++)
	{
		if(isnan(v.v[j]))
		{
			return false;
		}
	}
	return true;
}
sMatrix3 Kin_QuatToRotmat(sVector4 v)
{
	double qw = v.v[0], qx = v.v[1], qy = v.v[2], qz = v.v[3];
	sMatrix3 mat = Mat3Ones();
	
	mat.v[0][0] = 2.0*qw*qw - 1.0 + 2.0*qx*qx;
	mat.v[0][1] = 2.0*qx*qy - 2.0*qw*qz;
	mat.v[0][2] = 2.0*qx*qz + 2.0*qw*qy;
	mat.v[1][0] = 2.0*qx*qy + 2*qw*qz;
	mat.v[1][1] = 2.0*qw*qw - 1.0 + 2*qy*qy;
	mat.v[1][2] = 2.0*qy*qz - 2.0*qw*qx;
	mat.v[2][0] = 2.0*qx*qz - 2*qw*qy;
	mat.v[2][1] = 2.0*qy*qz + 2.0*qw*qx;
	mat.v[2][2] = 2.0*qw*qw - 1.0 + 2*qz*qz;
	
	return mat;
}
sVector4 Kin_RotmatToQuat(sMatrix3 m)
{
	sVector4 v;
	double qw, qx, qy, qz;
	double m00 = m.v[0][0], m01 = m.v[0][1], m02 = m.v[0][2];
	double m10 = m.v[1][0], m11 = m.v[1][1], m12 = m.v[1][2];
	double m20 = m.v[2][0], m21 = m.v[2][1], m22 = m.v[2][2];
	
	double tr = m00 + m11 + m22;
	if (tr > 0) 
	{ 
	  double S = sqrt(tr+1.0) * 2; // S=4*qw 
	  qw = 0.25 * S;
	  qx = (m21 - m12) / S;
	  qy = (m02 - m20) / S; 
	  qz = (m10 - m01) / S; 
	} 
	else if ((m00 > m11)&(m00 > m22)) 
	{ 
	  double S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
	  qw = (m21 - m12) / S;
	  qx = 0.25 * S;
	  qy = (m01 + m10) / S; 
	  qz = (m02 + m20) / S; 
	} 
	else if (m11 > m22) 
	{ 
	  double S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
	  qw = (m02 - m20) / S;
	  qx = (m01 + m10) / S; 
	  qy = 0.25 * S;
	  qz = (m12 + m21) / S; 
	} 
	else 
	{ 
	  double S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
	  qw = (m10 - m01) / S;
	  qx = (m02 + m20) / S;
	  qy = (m12 + m21) / S;
	  qz = 0.25 * S;
	}
	
	v.v[0] = qw;
	v.v[1] = qx;
	v.v[2] = qy;
	v.v[3] = qz;
	
	return v;
}
static sMatrix4 Kin_FKRaw(sVector6 qv)
{
	sVector6 q = qv;
	for(int i=0;i<JOINTS_MAX;i++)
	{
		q.v[i] = dir.v[i] * qv.v[i] + offset.v[i];
		if(q.v[i] > M_PI) 
			q.v[i] =  q.v[i] - 2.0*M_PI;
		else if(q.v[i] < -M_PI)
			q.v[i] = 2.0*M_PI + q.v[i];
	}

	sMatrix4 mat = Mat4Ones();
	
	mat.v[0][0] =	cos(q.v[5])*sin(q.v[0])*sin(q.v[4])+cos(q.v[0])*(cos(q.v[1]+q.v[2]+q.v[3])*cos(q.v[4])*cos(q.v[5])-sin(q.v[1]+q.v[2]+q.v[3])*sin(q.v[5]));
	mat.v[0][1] =	-(sin(q.v[0])*sin(q.v[4])*sin(q.v[5]))-cos(q.v[0])*(cos(q.v[5])*sin(q.v[1]+q.v[2]+q.v[3])+cos(q.v[1]+q.v[2]+q.v[3])*cos(q.v[4])*sin(q.v[5]));
	mat.v[0][2] =	cos(q.v[4])*sin(q.v[0])-cos(q.v[0])*cos(q.v[1]+q.v[2]+q.v[3])*sin(q.v[4]);
	mat.v[0][3] =	(d4+d6*cos(q.v[4]))*sin(q.v[0])+cos(q.v[0])*(a2*cos(q.v[1])+a3*cos(q.v[1]+q.v[2])+d5*sin(q.v[1]+q.v[2]+q.v[3])-d6*cos(q.v[1]+q.v[2]+q.v[3])*sin(q.v[4]));
		
	mat.v[1][0] =	cos(q.v[1]+q.v[2]+q.v[3])*cos(q.v[4])*cos(q.v[5])*sin(q.v[0])-cos(q.v[0])*cos(q.v[5])*sin(q.v[4])-sin(q.v[0])*sin(q.v[1]+q.v[2]+q.v[3])*sin(q.v[5]);
	mat.v[1][1] =	-(cos(q.v[5])*sin(q.v[0])*sin(q.v[1]+q.v[2]+q.v[3]))+(-(cos(q.v[1]+q.v[2]+q.v[3])*cos(q.v[4])*sin(q.v[0]))+cos(q.v[0])*sin(q.v[4]))*sin(q.v[5]);
	mat.v[1][2] =	-(cos(q.v[0])*cos(q.v[4]))-cos(q.v[1]+q.v[2]+q.v[3])*sin(q.v[0])*sin(q.v[4]);
	mat.v[1][3] =	-(cos(q.v[0])*(d4+d6*cos(q.v[4])))+sin(q.v[0])*(a2*cos(q.v[1])+a3*cos(q.v[1]+q.v[2])+d5*sin(q.v[1]+q.v[2]+q.v[3])-d6*cos(q.v[1]+q.v[2]+q.v[3])*sin(q.v[4]));
		
	mat.v[2][0] =	cos(q.v[4])*cos(q.v[5])*sin(q.v[1]+q.v[2]+q.v[3])+cos(q.v[1]+q.v[2]+q.v[3])*sin(q.v[5]);
	mat.v[2][1] =	cos(q.v[1]+q.v[2]+q.v[3])*cos(q.v[5])-cos(q.v[4])*sin(q.v[1]+q.v[2]+q.v[3])*sin(q.v[5]);
	mat.v[2][2] =	-(sin(q.v[1]+q.v[2]+q.v[3])*sin(q.v[4]));
	mat.v[2][3] =	d1-d5*cos(q.v[1]+q.v[2]+q.v[3])+a2*sin(q.v[1])+a3*sin(q.v[1]+q.v[2])-d6*sin(q.v[1]+q.v[2]+q.v[3])*sin(q.v[4]);
	
	return mat;
}

static sIKSol Kin_IKRaw(sMatrix4 mzad)
{
	sIKSol sol;
	sVector6 Qval[IK_SOLNUM];
	for(int i=0;i<IK_SOLNUM;i++)
		Qval[i] = Vec6Zeros();
		
	double x[IK_SOLNUM], y[IK_SOLNUM], theta[IK_SOLNUM];
	
	sMatrix4 M5 = Mat4xMat4(mzad, HT(0,0,-d6));
	double R = sqrt(M5.v[0][3] * M5.v[0][3] + M5.v[1][3] * M5.v[1][3]);
	double alpha11 = -atan2(M5.v[0][3], M5.v[1][3]);
	double alpha12 = alpha11 + M_PI;
	double alpha = atan2(d4, sqrt(R*R - d4*d4));
	
	if(alpha12 > M_PI) 
		alpha12 =  alpha12 - 2.0*M_PI;
	else if(alpha12 < -M_PI)
		alpha12 = 2.0*M_PI + alpha12;
	
	//*********************************** Obliczanie joint 0 i joint 4 ***********************************
	double Q0a = alpha11 + alpha + M_PI_2;
	double Q0b = alpha11 - alpha + M_PI_2;
	double Q0c = alpha12 + alpha + M_PI_2;
	double Q0d = alpha12 - alpha + M_PI_2;
	double Q4a = acos(((mzad.v[0][3] * sin(Q0a) - mzad.v[1][3] * cos(Q0a)) - d4) / d6);
	double Q4b = acos(((mzad.v[0][3] * sin(Q0b) - mzad.v[1][3] * cos(Q0b)) - d4) / d6);
	double Q4c = acos(((mzad.v[0][3] * sin(Q0c) - mzad.v[1][3] * cos(Q0c)) - d4) / d6);
	double Q4d = acos(((mzad.v[0][3] * sin(Q0d) - mzad.v[1][3] * cos(Q0d)) - d4) / d6);
	
	int idx = 0;
	for(int i=0;i<IK_SOLNUM/4;i++)
	{
		Qval[idx].v[0] = Q0a;
		Qval[idx].v[4] = Q4a;
		idx++;
	}
	for(int i=0;i<IK_SOLNUM/4;i++)
	{
		Qval[idx].v[0] = Q0b;
		Qval[idx].v[4] = Q4b;
		idx++;
	}
	for(int i=0;i<IK_SOLNUM/4;i++)
	{
		Qval[idx].v[0] = Q0c;
		Qval[idx].v[4] = Q4c;
		idx++;
	}
	for(int i=0;i<IK_SOLNUM/4;i++)
	{
		Qval[idx].v[0] = Q0d;
		Qval[idx].v[4] = Q4d;
		idx++;
	}
	for(int i=0;i<IK_SOLNUM;i++)
		if((i % (IK_SOLNUM/4)) >= (IK_SOLNUM/8))
			Qval[i].v[4] *= -1.0;
	
	//*********************************** Obliczanie joint 5 ***********************************
	
	for(int i=0;i<IK_SOLNUM;i++)
		if(Kin_SolIsReal(Qval[i]) == true)
		{
			Qval[i].v[5] = atan2((-mzad.v[0][1] * sin(Qval[i].v[0]) + mzad.v[1][1] * cos(Qval[i].v[0])) / sin(Qval[i].v[4]), -((-mzad.v[0][0] * sin(Qval[i].v[0]) + mzad.v[1][0] * cos(Qval[i].v[0])) / sin(Qval[i].v[4])));
		}
	
	//*********************************** Obliczanie joint 2, 3 i 4 ***********************************
	sMatrix4 Mx1, Mx5, Mx6, M4, M5a, mtemp;
	for(int i=0;i<IK_SOLNUM;i++)
	{
		if(Kin_SolIsReal(Qval[i]) == true)
		{
			Mx1 = Mat4xMat4(Mat4xMat4(HRZ(Qval[i].v[0]), HT(0, 0, d1)), HRX(M_PI_2));
			Mx5 = Mat4xMat4(Mat4xMat4(HRZ(Qval[i].v[4]), HT(0, 0, d5)), HRX(-M_PI_2));
			Mx6 = Mat4xMat4(HRZ(Qval[i].v[5]), HT(0, 0, d6));
			mtemp = Mat4xMat4(InvMat4(Mx1), mzad);
			M4 = Mat4xMat4(mtemp, InvMat4(Mat4xMat4(Mx5, Mx6)));
			M5a = Mat4xMat4(mtemp, InvMat4(Mx6));
			x[i] = M4.v[0][3];
			y[i] = M4.v[1][3];
			theta[i] = atan2(M5a.v[1][3] - M4.v[1][3], M5a.v[0][3] - M4.v[0][3]);
			
			if((i % 2) == 0)
				Qval[i].v[2] = acos((x[i]*x[i] + y[i]*y[i] - a2*a2 - a3*a3) / (2*a2*a3));
			else
				Qval[i].v[2] = -acos((x[i]*x[i] + y[i]*y[i] - a2*a2 - a3*a3) / (2*a2*a3));
			Qval[i].v[1] = atan2(y[i], x[i]) - atan2((a3*sin(Qval[i].v[2])), (a2 + a3 * cos(Qval[i].v[2])));
			Qval[i].v[3] = theta[i] - Qval[i].v[1] - Qval[i].v[2] + M_PI_2;
		}
	}
	
	//*********************************** Ograniczenie wartosci do +-pi ***********************************
	for(int i=0;i<IK_SOLNUM;i++)
		if(Kin_SolIsReal(Qval[i]) == true)
		{
			for(int j=0;j<JOINTS_MAX;j++)
				if(Qval[i].v[j] > M_PI) 
					Qval[i].v[j] =  Qval[i].v[j] - 2.0*M_PI;
				else if(Qval[i].v[j] < -M_PI)
					Qval[i].v[j] = 2.0*M_PI + Qval[i].v[j];
		}
	
	//*********************************** Zwrócenie wartosci (bez liczb zespolonych ) ***********************************
	idx = 0;
	for(int i=0;i<IK_SOLNUM;i++)
	{
		if(Kin_SolIsReal(Qval[i]) == true)
			sol.v[idx++] = Qval[i];
	}
	sol.isRealSolNum = idx;
	
	//*********************************** Dopasowanie do ukladów wspólrzednych robota Avena ***********************************
	for(int i=0;i<idx;i++)
	{
		for(int j=0;j<JOINTS_MAX;j++)
		{
			sol.v[i].v[j] = dir.v[j] * (sol.v[i].v[j] - offset.v[j]);
			if(sol.v[i].v[j] > M_PI) 
				sol.v[i].v[j] =  sol.v[i].v[j] - 2.0*M_PI;
			else if(sol.v[i].v[j] < -M_PI)
				sol.v[i].v[j] = 2.0*M_PI + sol.v[i].v[j];
		}
	}
	return sol;
}
void Kin_Conf(void)
{
//	d1 = pC->Arm.Joints[0].origin[2] + pC->Arm.Joints[1].origin[2]; // j1z + j2z;
//	a2 = -pC->Arm.Joints[2].origin[0]; // -j3x; 
//	a3 = pC->Arm.Joints[3].origin[0]; // j4x; 
//	d4 = pC->Arm.Joints[1].origin[0] + pC->Arm.Joints[2].origin[2] - pC->Arm.Joints[3].origin[2] + pC->Arm.Joints[4].origin[2]; // j2x + j3z - j4z + j5z; 
//	d5 = pC->Arm.Joints[4].origin[0] + pC->Arm.Joints[5].origin[2]; // j5x + j6z; 
//	d6 = pC->Arm.Joints[5].origin[0] + pC->Arm.Joints[6].origin[2]; // j6x + j7z;
	
	d1 = 0.1209;
	a2 = 0.686;
	a3 = 0.671;
	d4 = 0.1165;
	d5 = 0.1155;
	d6 = 0.0675;
	
	offset = Vec6SetValues(M_PI_2, M_PI_2, 0, M_PI_2, 0, M_PI_2);
	dir = Vec6SetValues(1.0, 1.0, -1.0, 1.0, 1.0, 1.0);
}
sRobPos Kin_FindNearestSolution(sRobPos targetPos)
{
	double sum[IK_SOLNUMREAL];
	int solnum = 0;
	for(int i=0;i<targetPos.sol.isRealSolNum;i++)
	{
		sum[i] = 0.0;
		for(int num=0;num<JOINTS_MAX;num++)
		{
			sum[i] += fabs(targetPos.sol.v[i].v[num] - pC->Jtc.robPos.pos.v[num]);
		}
	}
	double min = sum[0];
	for(int i=0;i<targetPos.sol.isRealSolNum;i++)
	{
		if(sum[i] < min)
		{
			min = sum[i];
			solnum = i;
		}
	}
	targetPos.conf.v[3] = solnum;
	targetPos.qSol = pC->Jtc.robJog.targetPos.sol.v[solnum];
	return targetPos;
}
static void Kin_CheckNoRealSolutionError(sRobPos pointIn)
{
	if(pointIn.sol.isRealSolNum == 0)
		pC->Jtc.internalKinNoRealSoution = true;
	else
		pC->Jtc.internalKinNoRealSoution = false;
}
sRobPos Kin_FKCalc(sRobPos pointIn)
{
	sRobPos pointOut = pointIn;
	pointOut.mat = Kin_FKRaw(pointIn.pos); //pozycja we wsp. konf. na macierz pozycji i rotacji 4x4
	pointOut.mat = Mat4xMat4(pointOut.mat, pC->Jtc.robTools[pC->Jtc.robToolNum].mat); //uwzglednienie aktualnie wybranego narzedzia
	pointOut.quat = Kin_RotmatToQuat(Mat4ToMat3(pointOut.mat)); //macierz pozycji i rotacji na quaterniony
	return pointOut;
}
sRobPos Kin_IKCalcFromQuat(sRobPos pointIn)
{
	sRobPos pointOut = pointIn;
	pointOut.mat = Mat3ToMat4(Kin_QuatToRotmat(pointIn.quat), pointOut.mat);
	sMatrix4 matTemp = Mat4xMat4(pointOut.mat, pC->Jtc.robTools[pC->Jtc.robToolNum].matInv); //uwzglednienie aktualnie wybranego narzedzia
	pointOut.sol = Kin_IKRaw(matTemp);
	Kin_CheckNoRealSolutionError(pointOut);
	pointOut.qSol = pointOut.sol.v[pointOut.conf.v[3]];
	return pointOut;
}
sRobPos Kin_IKCalcFromRotMat(sRobPos pointIn)
{
	sRobPos pointOut = pointIn;
	sMatrix4 matTemp = Mat4xMat4(pointOut.mat, pC->Jtc.robTools[pC->Jtc.robToolNum].matInv); //uwzglednienie aktualnie wybranego narzedzia
	pointOut.sol = Kin_IKRaw(matTemp);
	Kin_CheckNoRealSolutionError(pointOut);
	pointOut.qSol = pointOut.sol.v[pointOut.conf.v[3]];
	return pointOut;
}
void Kin_RobPosAct(void)
{
	for(int i=0;i<JOINTS_MAX;i++)
		pC->Jtc.robPos.pos.v[i] = pC->Joints[i].currentPos;
	
	pC->Jtc.robPos = Kin_FKCalc(pC->Jtc.robPos);
}
