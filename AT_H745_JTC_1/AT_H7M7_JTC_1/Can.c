#include "Can.h"
extern  sControl* pC;
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
static void Can_SendFrameMove(void)
{
	uint16_t num = Can_TxF_Move; // tx buffer number
	if(pC->Can.TxMsgs[num].status != Can_TxS_Sending)
	{
		uint16_t idx = 0;
		for(int i=0;i<JOINTS_MAX;i++)
		{
			pC->Can.TxMsgs[num].bytes[idx++] = (int16_t)(pC->Joints[i].setTorque / pC->Joints[i].maxTorqueCom * MAXINT16) >> 8;
			pC->Can.TxMsgs[num].bytes[idx++] = (int16_t)(pC->Joints[i].setTorque / pC->Joints[i].maxTorqueCom * MAXINT16) >> 0;
		}
		pC->Can.TxMsgs[num].bytes[idx++] = pC->Gripper.targetPumpState;
		
		for(int i=0;i<CAN_TXDATA_LEN/4;i++)
		{
			pC->Can.TxMsgs[num].data[i] = 0x00;
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+0] << 0);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+1] << 8);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+2] << 16);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+3] << 24);
		}
		
		idx = (CAN_TXDATA_LEN / 4 + 2) * num;
		*(pC->Can.txBufAddr + idx + 0) = pC->Can.TxMsgs[num].r0;
		*(pC->Can.txBufAddr + idx + 1) = pC->Can.TxMsgs[num].r1;
		*(pC->Can.txBufAddr + idx + 2) = pC->Can.TxMsgs[num].data[0];
		*(pC->Can.txBufAddr + idx + 3) = pC->Can.TxMsgs[num].data[1];
		*(pC->Can.txBufAddr + idx + 4) = pC->Can.TxMsgs[num].data[2];
		*(pC->Can.txBufAddr + idx + 5) = pC->Can.TxMsgs[num].data[3];
		
		pC->Can.TxMsgs[num].status = Can_TxS_Sending;
		
		FDCAN1->TXBAR |= (1 << num); // start sending
	}
}
static void Can_SendFrameChangeFsm(void)
{
	uint16_t num = Can_TxF_ChangeFsm; // tx buffer number
	if(pC->Can.TxMsgs[num].status != Can_TxS_Sending)
	{
		uint16_t idx = 0;
		for(int i=0;i<JOINTS_MAX;i++)
		{
			pC->Can.TxMsgs[num].bytes[idx++] = pC->Joints[i].targetFsm;
		}
		pC->Can.TxMsgs[num].bytes[idx++] = pC->Gripper.targetFsm;
		
		for(int i=0;i<CAN_TXDATA_LEN/4;i++)
		{
			pC->Can.TxMsgs[num].data[i] = 0x00;
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+0] << 0);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+1] << 8);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+2] << 16);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+3] << 24);
		}
		
		idx = (CAN_TXDATA_LEN / 4 + 2) * num;
		*(pC->Can.txBufAddr + idx + 0) = pC->Can.TxMsgs[num].r0;
		*(pC->Can.txBufAddr + idx + 1) = pC->Can.TxMsgs[num].r1;
		*(pC->Can.txBufAddr + idx + 2) = pC->Can.TxMsgs[num].data[0];
		*(pC->Can.txBufAddr + idx + 3) = pC->Can.TxMsgs[num].data[1];
		*(pC->Can.txBufAddr + idx + 4) = pC->Can.TxMsgs[num].data[2];
		*(pC->Can.txBufAddr + idx + 5) = pC->Can.TxMsgs[num].data[3];
		
		pC->Can.TxMsgs[num].status = Can_TxS_Sending;
		
		FDCAN1->TXBAR |= (1 << num); // start sending
	}
}
static void Can_SendFrameChangeMode(void)
{
	uint16_t num = Can_TxF_ChangeMode; // tx buffer number
	if(pC->Can.TxMsgs[num].status != Can_TxS_Sending)
	{
		uint16_t idx = 0;
		for(int i=0;i<JOINTS_MAX;i++)
		{
			pC->Can.TxMsgs[num].bytes[idx++] = pC->Joints[i].targetMode;
			pC->Can.TxMsgs[num].bytes[idx++] = pC->Joints[i].confFun;
		}
		pC->Can.TxMsgs[num].bytes[idx++] = 0x00;
		pC->Can.TxMsgs[num].bytes[idx++] = pC->Gripper.confFun;
		
		for(int i=0;i<CAN_TXDATA_LEN/4;i++)
		{
			pC->Can.TxMsgs[num].data[i] = 0x00;
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+0] << 0);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+1] << 8);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+2] << 16);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+3] << 24);
		}
		
		idx = (CAN_TXDATA_LEN / 4 + 2) * num;
		*(pC->Can.txBufAddr + idx + 0) = pC->Can.TxMsgs[num].r0;
		*(pC->Can.txBufAddr + idx + 1) = pC->Can.TxMsgs[num].r1;
		*(pC->Can.txBufAddr + idx + 2) = pC->Can.TxMsgs[num].data[0];
		*(pC->Can.txBufAddr + idx + 3) = pC->Can.TxMsgs[num].data[1];
		*(pC->Can.txBufAddr + idx + 4) = pC->Can.TxMsgs[num].data[2];
		*(pC->Can.txBufAddr + idx + 5) = pC->Can.TxMsgs[num].data[3];
		
		pC->Can.TxMsgs[num].status = Can_TxS_Sending;
		
		FDCAN1->TXBAR |= (1 << num); // start sending
	}
}
static void Can_SendFrameResetAllDevices(void)
{
	uint16_t num = Can_TxF_ResetAllDevices; // tx buffer number
	if(pC->Can.TxMsgs[num].status != Can_TxS_Sending)
	{
		uint16_t idx = 0;
	
		for(int i=0;i<CAN_TXDATA_LEN/4;i++)
		{
			pC->Can.TxMsgs[num].data[i] = 0x00;
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+0] << 0);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+1] << 8);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+2] << 16);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+3] << 24);
		}
		
		idx = (CAN_TXDATA_LEN / 4 + 2) * num;
		*(pC->Can.txBufAddr + idx + 0) = pC->Can.TxMsgs[num].r0;
		*(pC->Can.txBufAddr + idx + 1) = pC->Can.TxMsgs[num].r1;
		*(pC->Can.txBufAddr + idx + 2) = pC->Can.TxMsgs[num].data[0];
		*(pC->Can.txBufAddr + idx + 3) = pC->Can.TxMsgs[num].data[1];
		*(pC->Can.txBufAddr + idx + 4) = pC->Can.TxMsgs[num].data[2];
		*(pC->Can.txBufAddr + idx + 5) = pC->Can.TxMsgs[num].data[3];
		
		pC->Can.TxMsgs[num].status = Can_TxS_Sending;
		
		FDCAN1->TXBAR |= (1 << num); // start sending
	}
}
static void Can_SendFrameResetJoint0(void)
{
	uint16_t num = Can_TxF_ResetJoint0; // tx buffer number
	if(pC->Can.TxMsgs[num].status != Can_TxS_Sending)
	{
		uint16_t idx = 0;
	
		for(int i=0;i<CAN_TXDATA_LEN/4;i++)
		{
			pC->Can.TxMsgs[num].data[i] = 0x00;
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+0] << 0);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+1] << 8);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+2] << 16);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+3] << 24);
		}
		
		idx = (CAN_TXDATA_LEN / 4 + 2) * num;
		*(pC->Can.txBufAddr + idx + 0) = pC->Can.TxMsgs[num].r0;
		*(pC->Can.txBufAddr + idx + 1) = pC->Can.TxMsgs[num].r1;
		*(pC->Can.txBufAddr + idx + 2) = pC->Can.TxMsgs[num].data[0];
		*(pC->Can.txBufAddr + idx + 3) = pC->Can.TxMsgs[num].data[1];
		*(pC->Can.txBufAddr + idx + 4) = pC->Can.TxMsgs[num].data[2];
		*(pC->Can.txBufAddr + idx + 5) = pC->Can.TxMsgs[num].data[3];
		
		pC->Can.TxMsgs[num].status = Can_TxS_Sending;
		
		FDCAN1->TXBAR |= (1 << num); // start sending
	}
}
static void Can_SendFrameResetJoint1(void)
{
	uint16_t num = Can_TxF_ResetJoint1; // tx buffer number
	if(pC->Can.TxMsgs[num].status != Can_TxS_Sending)
	{
		uint16_t idx = 0;
	
		for(int i=0;i<CAN_TXDATA_LEN/4;i++)
		{
			pC->Can.TxMsgs[num].data[i] = 0x00;
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+0] << 0);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+1] << 8);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+2] << 16);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+3] << 24);
		}
		
		idx = (CAN_TXDATA_LEN / 4 + 2) * num;
		*(pC->Can.txBufAddr + idx + 0) = pC->Can.TxMsgs[num].r0;
		*(pC->Can.txBufAddr + idx + 1) = pC->Can.TxMsgs[num].r1;
		*(pC->Can.txBufAddr + idx + 2) = pC->Can.TxMsgs[num].data[0];
		*(pC->Can.txBufAddr + idx + 3) = pC->Can.TxMsgs[num].data[1];
		*(pC->Can.txBufAddr + idx + 4) = pC->Can.TxMsgs[num].data[2];
		*(pC->Can.txBufAddr + idx + 5) = pC->Can.TxMsgs[num].data[3];
		
		pC->Can.TxMsgs[num].status = Can_TxS_Sending;
		
		FDCAN1->TXBAR |= (1 << num); // start sending
	}
}
static void Can_SendFrameResetJoint2(void)
{
	uint16_t num = Can_TxF_ResetJoint2; // tx buffer number
	if(pC->Can.TxMsgs[num].status != Can_TxS_Sending)
	{
		uint16_t idx = 0;
	
		for(int i=0;i<CAN_TXDATA_LEN/4;i++)
		{
			pC->Can.TxMsgs[num].data[i] = 0x00;
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+0] << 0);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+1] << 8);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+2] << 16);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+3] << 24);
		}
		
		idx = (CAN_TXDATA_LEN / 4 + 2) * num;
		*(pC->Can.txBufAddr + idx + 0) = pC->Can.TxMsgs[num].r0;
		*(pC->Can.txBufAddr + idx + 1) = pC->Can.TxMsgs[num].r1;
		*(pC->Can.txBufAddr + idx + 2) = pC->Can.TxMsgs[num].data[0];
		*(pC->Can.txBufAddr + idx + 3) = pC->Can.TxMsgs[num].data[1];
		*(pC->Can.txBufAddr + idx + 4) = pC->Can.TxMsgs[num].data[2];
		*(pC->Can.txBufAddr + idx + 5) = pC->Can.TxMsgs[num].data[3];
		
		pC->Can.TxMsgs[num].status = Can_TxS_Sending;
		
		FDCAN1->TXBAR |= (1 << num); // start sending
	}
}
static void Can_SendFrameResetJoint3(void)
{
	uint16_t num = Can_TxF_ResetJoint3; // tx buffer number
	if(pC->Can.TxMsgs[num].status != Can_TxS_Sending)
	{
		uint16_t idx = 0;
	
		for(int i=0;i<CAN_TXDATA_LEN/4;i++)
		{
			pC->Can.TxMsgs[num].data[i] = 0x00;
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+0] << 0);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+1] << 8);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+2] << 16);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+3] << 24);
		}
		
		idx = (CAN_TXDATA_LEN / 4 + 2) * num;
		*(pC->Can.txBufAddr + idx + 0) = pC->Can.TxMsgs[num].r0;
		*(pC->Can.txBufAddr + idx + 1) = pC->Can.TxMsgs[num].r1;
		*(pC->Can.txBufAddr + idx + 2) = pC->Can.TxMsgs[num].data[0];
		*(pC->Can.txBufAddr + idx + 3) = pC->Can.TxMsgs[num].data[1];
		*(pC->Can.txBufAddr + idx + 4) = pC->Can.TxMsgs[num].data[2];
		*(pC->Can.txBufAddr + idx + 5) = pC->Can.TxMsgs[num].data[3];
		
		pC->Can.TxMsgs[num].status = Can_TxS_Sending;
		
		FDCAN1->TXBAR |= (1 << num); // start sending
	}
}
static void Can_SendFrameResetJoint4(void)
{
	uint16_t num = Can_TxF_ResetJoint4; // tx buffer number
	if(pC->Can.TxMsgs[num].status != Can_TxS_Sending)
	{
		uint16_t idx = 0;
	
		for(int i=0;i<CAN_TXDATA_LEN/4;i++)
		{
			pC->Can.TxMsgs[num].data[i] = 0x00;
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+0] << 0);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+1] << 8);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+2] << 16);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+3] << 24);
		}
		
		idx = (CAN_TXDATA_LEN / 4 + 2) * num;
		*(pC->Can.txBufAddr + idx + 0) = pC->Can.TxMsgs[num].r0;
		*(pC->Can.txBufAddr + idx + 1) = pC->Can.TxMsgs[num].r1;
		*(pC->Can.txBufAddr + idx + 2) = pC->Can.TxMsgs[num].data[0];
		*(pC->Can.txBufAddr + idx + 3) = pC->Can.TxMsgs[num].data[1];
		*(pC->Can.txBufAddr + idx + 4) = pC->Can.TxMsgs[num].data[2];
		*(pC->Can.txBufAddr + idx + 5) = pC->Can.TxMsgs[num].data[3];
		
		pC->Can.TxMsgs[num].status = Can_TxS_Sending;
		
		FDCAN1->TXBAR |= (1 << num); // start sending
	}
}
static void Can_SendFrameResetJoint5(void)
{
	uint16_t num = Can_TxF_ResetJoint5; // tx buffer number
	if(pC->Can.TxMsgs[num].status != Can_TxS_Sending)
	{
		uint16_t idx = 0;
	
		for(int i=0;i<CAN_TXDATA_LEN/4;i++)
		{
			pC->Can.TxMsgs[num].data[i] = 0x00;
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+0] << 0);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+1] << 8);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+2] << 16);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+3] << 24);
		}
		
		idx = (CAN_TXDATA_LEN / 4 + 2) * num;
		*(pC->Can.txBufAddr + idx + 0) = pC->Can.TxMsgs[num].r0;
		*(pC->Can.txBufAddr + idx + 1) = pC->Can.TxMsgs[num].r1;
		*(pC->Can.txBufAddr + idx + 2) = pC->Can.TxMsgs[num].data[0];
		*(pC->Can.txBufAddr + idx + 3) = pC->Can.TxMsgs[num].data[1];
		*(pC->Can.txBufAddr + idx + 4) = pC->Can.TxMsgs[num].data[2];
		*(pC->Can.txBufAddr + idx + 5) = pC->Can.TxMsgs[num].data[3];
		
		pC->Can.TxMsgs[num].status = Can_TxS_Sending;
		
		FDCAN1->TXBAR |= (1 << num); // start sending
	}
}
static void Can_SendFrameResetGripper(void)
{
	uint16_t num = Can_TxF_ResetGripper; // tx buffer number
	if(pC->Can.TxMsgs[num].status != Can_TxS_Sending)
	{
		uint16_t idx = 0;
	
		for(int i=0;i<CAN_TXDATA_LEN/4;i++)
		{
			pC->Can.TxMsgs[num].data[i] = 0x00;
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+0] << 0);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+1] << 8);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+2] << 16);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+3] << 24);
		}
		
		idx = (CAN_TXDATA_LEN / 4 + 2) * num;
		*(pC->Can.txBufAddr + idx + 0) = pC->Can.TxMsgs[num].r0;
		*(pC->Can.txBufAddr + idx + 1) = pC->Can.TxMsgs[num].r1;
		*(pC->Can.txBufAddr + idx + 2) = pC->Can.TxMsgs[num].data[0];
		*(pC->Can.txBufAddr + idx + 3) = pC->Can.TxMsgs[num].data[1];
		*(pC->Can.txBufAddr + idx + 4) = pC->Can.TxMsgs[num].data[2];
		*(pC->Can.txBufAddr + idx + 5) = pC->Can.TxMsgs[num].data[3];
		
		pC->Can.TxMsgs[num].status = Can_TxS_Sending;
		
		FDCAN1->TXBAR |= (1 << num); // start sending
	}
}
static void Can_SendFrameReadFriction(void)
{
	uint16_t num = Can_TxF_ReadFriction; // tx buffer number
	if(pC->Can.TxMsgs[num].status != Can_TxS_Sending)
	{
		uint16_t idx = 0;
		pC->Can.TxMsgs[num].bytes[idx++] = 0x60; //odczyt od rejestru 0x60
		pC->Can.TxMsgs[num].bytes[idx++] = 0x0C; // odczyt 12 rejestrów
		
		for(int i=0;i<CAN_TXDATA_LEN/4;i++)
		{
			pC->Can.TxMsgs[num].data[i] = 0x00;
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+0] << 0);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+1] << 8);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+2] << 16);
			pC->Can.TxMsgs[num].data[i] += ((uint32_t)pC->Can.TxMsgs[num].bytes[4*i+3] << 24);
		}
		
		idx = (CAN_TXDATA_LEN / 4 + 2) * num;
		*(pC->Can.txBufAddr + idx + 0) = pC->Can.TxMsgs[num].r0;
		*(pC->Can.txBufAddr + idx + 1) = pC->Can.TxMsgs[num].r1;
		*(pC->Can.txBufAddr + idx + 2) = pC->Can.TxMsgs[num].data[0];
		*(pC->Can.txBufAddr + idx + 3) = pC->Can.TxMsgs[num].data[1];
		*(pC->Can.txBufAddr + idx + 4) = pC->Can.TxMsgs[num].data[2];
		*(pC->Can.txBufAddr + idx + 5) = pC->Can.TxMsgs[num].data[3];
		
		pC->Can.TxMsgs[num].status = Can_TxS_Sending;
		
		FDCAN1->TXBAR |= (1 << num); // start sending
	}
}
static void Can_StructConf(void)
{
	pC->Can.filterAddrOffset = 0x0000;
	pC->Can.txBufAddrOffset = pC->Can.filterAddrOffset + 4 * CAN_FILTERS_MAX;
	pC->Can.rxBufAddrOffset = pC->Can.txBufAddrOffset + (CAN_TXDATA_LEN + 8) * CAN_TXBUF_MAX;
	pC->Can.rxFifo0AddrOffset = pC->Can.rxBufAddrOffset + (CAN_RXDATA_LEN + 8) * CAN_RXBUF_MAX;
	
	pC->Can.filterAddr = (uint32_t *)(CAN_MSGRAM_STARTADDR + pC->Can.filterAddrOffset);
	pC->Can.txBufAddr = (uint32_t *)(CAN_MSGRAM_STARTADDR + pC->Can.txBufAddrOffset);
	pC->Can.rxBufAddr = (uint32_t *)(CAN_MSGRAM_STARTADDR + pC->Can.rxBufAddrOffset);
	pC->Can.rxFifo0Addr = (uint32_t *)(CAN_MSGRAM_STARTADDR + pC->Can.rxFifo0AddrOffset);
}
static void Can_FiltersConf(void)
{
	uint8_t idx = 0;
	// Responses for boroadcast ID = 0x0000
	pC->Can.Filters[idx++].sfid1 = 0x0200;
	pC->Can.Filters[idx++].sfid1 = 0x0201;
	pC->Can.Filters[idx++].sfid1 = 0x0202;
	pC->Can.Filters[idx++].sfid1 = 0x0203;
	pC->Can.Filters[idx++].sfid1 = 0x0204;
	pC->Can.Filters[idx++].sfid1 = 0x0205;
	pC->Can.Filters[idx++].sfid1 = 0x0206;
	// Responses for boroadcast ID = 0x0010
	pC->Can.Filters[idx++].sfid1 = 0x0210;
	pC->Can.Filters[idx++].sfid1 = 0x0211;
	pC->Can.Filters[idx++].sfid1 = 0x0212;
	pC->Can.Filters[idx++].sfid1 = 0x0213;
	pC->Can.Filters[idx++].sfid1 = 0x0214;
	pC->Can.Filters[idx++].sfid1 = 0x0215;
	pC->Can.Filters[idx++].sfid1 = 0x0216;
	// Responses for boroadcast ID = 0x00F0
	pC->Can.Filters[idx++].sfid1 = 0x02F0;
	pC->Can.Filters[idx++].sfid1 = 0x02F1;
	pC->Can.Filters[idx++].sfid1 = 0x02F2;
	pC->Can.Filters[idx++].sfid1 = 0x02F3;
	pC->Can.Filters[idx++].sfid1 = 0x02F4;
	pC->Can.Filters[idx++].sfid1 = 0x02F5;
	pC->Can.Filters[idx++].sfid1 = 0x02F6;
	// Responses for boroadcast ID = 0x0020
	pC->Can.Filters[idx++].sfid1 = 0x0220;
	pC->Can.Filters[idx++].sfid1 = 0x0221;
	pC->Can.Filters[idx++].sfid1 = 0x0222;
	pC->Can.Filters[idx++].sfid1 = 0x0223;
	pC->Can.Filters[idx++].sfid1 = 0x0224;
	pC->Can.Filters[idx++].sfid1 = 0x0225;
	pC->Can.Filters[idx++].sfid1 = 0x0226;
	
	for(int i=0;i<CAN_FILTERS_MAX;i++)
	{
		pC->Can.Filters[i].sft = 0x01; // Dual ID filter for SFID1 or SFID2
		pC->Can.Filters[i].sfec = 0x07; // Store into Rx buffer or as debug message, configuration of FDCAN_SFT[1:0] ignored
		pC->Can.Filters[i].sfid2 = i; //buffer number
		pC->Can.Filters[i].r0 = (pC->Can.Filters[i].sft << 30) | (pC->Can.Filters[i].sfec << 27) | (pC->Can.Filters[i].sfid1 << 16) | (pC->Can.Filters[i].sfid2 << 0);
		*(pC->Can.filterAddr + i) = pC->Can.Filters[i].r0;
	}
}
static void Can_TxBufferConf(void)
{
	uint8_t num;
	
	// TxBuffer 0
	num = Can_TxF_Move;
	pC->Can.TxMsgs[num].reqSend = false;
	pC->Can.TxMsgs[num].funSendFrame = Can_SendFrameMove;
	pC->Can.TxMsgs[num].esi = 0x00;		//passive error
	pC->Can.TxMsgs[num].xtd = 0x00;		//standardowe identyfikatory 11bit
	pC->Can.TxMsgs[num].rtr = 0x00;		//ramka z danymi
	pC->Can.TxMsgs[num].id = 0x0000;	//identyfikator 11 bitowy
	pC->Can.TxMsgs[num].mm = 0x00;		//marker do event, nie uzywany
	pC->Can.TxMsgs[num].efc = 0x00;		//bez geneorwanie eventow
	pC->Can.TxMsgs[num].fdf = 0x01;		//ramka w formacie CANFD
	pC->Can.TxMsgs[num].brs = 0x01;		//zmienna predkosc - BRS = On
	pC->Can.TxMsgs[num].dlc = 0x0A;		//16 bajtow w ramce
	pC->Can.TxMsgs[num].r0 = (pC->Can.TxMsgs[num].esi << 31) | (pC->Can.TxMsgs[num].xtd << 30) | (pC->Can.TxMsgs[num].rtr << 29) | (pC->Can.TxMsgs[num].id << 18);
	pC->Can.TxMsgs[num].r1 = (pC->Can.TxMsgs[num].mm << 24) | (pC->Can.TxMsgs[num].efc << 23) | (pC->Can.TxMsgs[num].fdf << 21) | (pC->Can.TxMsgs[num].brs << 20) | (pC->Can.TxMsgs[num].dlc << 16);

	// TxBuffer 1
	num = Can_TxF_ChangeFsm;
	pC->Can.TxMsgs[num].reqSend = false;
	pC->Can.TxMsgs[num].funSendFrame = Can_SendFrameChangeFsm;
	pC->Can.TxMsgs[num].esi = 0x00;		//passive error
	pC->Can.TxMsgs[num].xtd = 0x00;		//standardowe identyfikatory 11bit
	pC->Can.TxMsgs[num].rtr = 0x00;		//ramka z danymi
	pC->Can.TxMsgs[num].id = 0x0010;	//identyfikator 11 bitowy
	pC->Can.TxMsgs[num].mm = 0x00;		//marker do event, nie uzywany
	pC->Can.TxMsgs[num].efc = 0x00;		//bez geneorwanie eventow
	pC->Can.TxMsgs[num].fdf = 0x01;		//ramka w formacie CANFD
	pC->Can.TxMsgs[num].brs = 0x01;		//zmienna predkosc - BRS = On
	pC->Can.TxMsgs[num].dlc = 0x07;		//7 bajtow w ramce
	pC->Can.TxMsgs[num].r0 = (pC->Can.TxMsgs[num].esi << 31) | (pC->Can.TxMsgs[num].xtd << 30) | (pC->Can.TxMsgs[num].rtr << 29) | (pC->Can.TxMsgs[num].id << 18);
	pC->Can.TxMsgs[num].r1 = (pC->Can.TxMsgs[num].mm << 24) | (pC->Can.TxMsgs[num].efc << 23) | (pC->Can.TxMsgs[num].fdf << 21) | (pC->Can.TxMsgs[num].brs << 20) | (pC->Can.TxMsgs[num].dlc << 16);

	// TxBuffer 2
	num = Can_TxF_ChangeMode;
	pC->Can.TxMsgs[num].reqSend = false;
	pC->Can.TxMsgs[num].funSendFrame = Can_SendFrameChangeMode;
	pC->Can.TxMsgs[num].esi = 0x00;		//passive error
	pC->Can.TxMsgs[num].xtd = 0x00;		//standardowe identyfikatory 11bit
	pC->Can.TxMsgs[num].rtr = 0x00;		//ramka z danymi
	pC->Can.TxMsgs[num].id = 0x00F0;	//identyfikator 11 bitowy
	pC->Can.TxMsgs[num].mm = 0x00;		//marker do event, nie uzywany
	pC->Can.TxMsgs[num].efc = 0x00;		//bez geneorwanie eventow
	pC->Can.TxMsgs[num].fdf = 0x01;		//ramka w formacie CANFD
	pC->Can.TxMsgs[num].brs = 0x01;		//zmienna predkosc - BRS = On
	pC->Can.TxMsgs[num].dlc = 0x0A;		//16 bajtow w ramce
	pC->Can.TxMsgs[num].r0 = (pC->Can.TxMsgs[num].esi << 31) | (pC->Can.TxMsgs[num].xtd << 30) | (pC->Can.TxMsgs[num].rtr << 29) | (pC->Can.TxMsgs[num].id << 18);
	pC->Can.TxMsgs[num].r1 = (pC->Can.TxMsgs[num].mm << 24) | (pC->Can.TxMsgs[num].efc << 23) | (pC->Can.TxMsgs[num].fdf << 21) | (pC->Can.TxMsgs[num].brs << 20) | (pC->Can.TxMsgs[num].dlc << 16);

	// TxBuffer 3
	num = Can_TxF_ReadFriction;
	pC->Can.TxMsgs[num].reqSend = false;
	pC->Can.TxMsgs[num].funSendFrame = Can_SendFrameReadFriction;
	pC->Can.TxMsgs[num].esi = 0x00;		//passive error
	pC->Can.TxMsgs[num].xtd = 0x00;		//standardowe identyfikatory 11bit
	pC->Can.TxMsgs[num].rtr = 0x00;		//ramka z danymi
	pC->Can.TxMsgs[num].id = 0x0020;	//identyfikator 11 bitowy
	pC->Can.TxMsgs[num].mm = 0x00;		//marker do event, nie uzywany
	pC->Can.TxMsgs[num].efc = 0x00;		//bez geneorwanie eventow
	pC->Can.TxMsgs[num].fdf = 0x01;		//ramka w formacie CANFD
	pC->Can.TxMsgs[num].brs = 0x01;		//zmienna predkosc - BRS = On
	pC->Can.TxMsgs[num].dlc = 0x02;		//2 bajty w ramce
	pC->Can.TxMsgs[num].r0 = (pC->Can.TxMsgs[num].esi << 31) | (pC->Can.TxMsgs[num].xtd << 30) | (pC->Can.TxMsgs[num].rtr << 29) | (pC->Can.TxMsgs[num].id << 18);
	pC->Can.TxMsgs[num].r1 = (pC->Can.TxMsgs[num].mm << 24) | (pC->Can.TxMsgs[num].efc << 23) | (pC->Can.TxMsgs[num].fdf << 21) | (pC->Can.TxMsgs[num].brs << 20) | (pC->Can.TxMsgs[num].dlc << 16);


	// TxBuffer 4
	num = Can_TxF_ResetAllDevices;
	pC->Can.TxMsgs[num].reqSend = false;
	pC->Can.TxMsgs[num].funSendFrame = Can_SendFrameResetAllDevices;
	pC->Can.TxMsgs[num].esi = 0x00;		//passive error
	pC->Can.TxMsgs[num].xtd = 0x00;		//standardowe identyfikatory 11bit
	pC->Can.TxMsgs[num].rtr = 0x00;		//ramka z danymi
	pC->Can.TxMsgs[num].id = 0x00A0;	//identyfikator 11 bitowy
	pC->Can.TxMsgs[num].mm = 0x00;		//marker do event, nie uzywany
	pC->Can.TxMsgs[num].efc = 0x00;		//bez geneorwanie eventow
	pC->Can.TxMsgs[num].fdf = 0x01;		//ramka w formacie CANFD
	pC->Can.TxMsgs[num].brs = 0x01;		//zmienna predkosc - BRS = On
	pC->Can.TxMsgs[num].dlc = 0x00;		//0 bajtow w ramce
	pC->Can.TxMsgs[num].r0 = (pC->Can.TxMsgs[num].esi << 31) | (pC->Can.TxMsgs[num].xtd << 30) | (pC->Can.TxMsgs[num].rtr << 29) | (pC->Can.TxMsgs[num].id << 18);
	pC->Can.TxMsgs[num].r1 = (pC->Can.TxMsgs[num].mm << 24) | (pC->Can.TxMsgs[num].efc << 23) | (pC->Can.TxMsgs[num].fdf << 21) | (pC->Can.TxMsgs[num].brs << 20) | (pC->Can.TxMsgs[num].dlc << 16);

	// TxBuffer 5
	num = Can_TxF_ResetJoint0;
	pC->Can.TxMsgs[num].reqSend = false;
	pC->Can.TxMsgs[num].funSendFrame = Can_SendFrameResetJoint0;
	pC->Can.TxMsgs[num].esi = 0x00;		//passive error
	pC->Can.TxMsgs[num].xtd = 0x00;		//standardowe identyfikatory 11bit
	pC->Can.TxMsgs[num].rtr = 0x00;		//ramka z danymi
	pC->Can.TxMsgs[num].id = 0x01A0;	//identyfikator 11 bitowy
	pC->Can.TxMsgs[num].mm = 0x00;		//marker do event, nie uzywany
	pC->Can.TxMsgs[num].efc = 0x00;		//bez geneorwanie eventow
	pC->Can.TxMsgs[num].fdf = 0x01;		//ramka w formacie CANFD
	pC->Can.TxMsgs[num].brs = 0x01;		//zmienna predkosc - BRS = On
	pC->Can.TxMsgs[num].dlc = 0x00;		//0 bajtow w ramce
	pC->Can.TxMsgs[num].r0 = (pC->Can.TxMsgs[num].esi << 31) | (pC->Can.TxMsgs[num].xtd << 30) | (pC->Can.TxMsgs[num].rtr << 29) | (pC->Can.TxMsgs[num].id << 18);
	pC->Can.TxMsgs[num].r1 = (pC->Can.TxMsgs[num].mm << 24) | (pC->Can.TxMsgs[num].efc << 23) | (pC->Can.TxMsgs[num].fdf << 21) | (pC->Can.TxMsgs[num].brs << 20) | (pC->Can.TxMsgs[num].dlc << 16);

	// TxBuffer 6
	num = Can_TxF_ResetJoint1;
	pC->Can.TxMsgs[num].reqSend = false;
	pC->Can.TxMsgs[num].funSendFrame = Can_SendFrameResetJoint1;
	pC->Can.TxMsgs[num].esi = 0x00;		//passive error
	pC->Can.TxMsgs[num].xtd = 0x00;		//standardowe identyfikatory 11bit
	pC->Can.TxMsgs[num].rtr = 0x00;		//ramka z danymi
	pC->Can.TxMsgs[num].id = 0x01A1;	//identyfikator 11 bitowy
	pC->Can.TxMsgs[num].mm = 0x00;		//marker do event, nie uzywany
	pC->Can.TxMsgs[num].efc = 0x00;		//bez geneorwanie eventow
	pC->Can.TxMsgs[num].fdf = 0x01;		//ramka w formacie CANFD
	pC->Can.TxMsgs[num].brs = 0x01;		//zmienna predkosc - BRS = On
	pC->Can.TxMsgs[num].dlc = 0x00;		//0 bajtow w ramce
	pC->Can.TxMsgs[num].r0 = (pC->Can.TxMsgs[num].esi << 31) | (pC->Can.TxMsgs[num].xtd << 30) | (pC->Can.TxMsgs[num].rtr << 29) | (pC->Can.TxMsgs[num].id << 18);
	pC->Can.TxMsgs[num].r1 = (pC->Can.TxMsgs[num].mm << 24) | (pC->Can.TxMsgs[num].efc << 23) | (pC->Can.TxMsgs[num].fdf << 21) | (pC->Can.TxMsgs[num].brs << 20) | (pC->Can.TxMsgs[num].dlc << 16);

	// TxBuffer 7
	num = Can_TxF_ResetJoint2;
	pC->Can.TxMsgs[num].reqSend = false;
	pC->Can.TxMsgs[num].funSendFrame = Can_SendFrameResetJoint2;
	pC->Can.TxMsgs[num].esi = 0x00;		//passive error
	pC->Can.TxMsgs[num].xtd = 0x00;		//standardowe identyfikatory 11bit
	pC->Can.TxMsgs[num].rtr = 0x00;		//ramka z danymi
	pC->Can.TxMsgs[num].id = 0x01A2;	//identyfikator 11 bitowy
	pC->Can.TxMsgs[num].mm = 0x00;		//marker do event, nie uzywany
	pC->Can.TxMsgs[num].efc = 0x00;		//bez geneorwanie eventow
	pC->Can.TxMsgs[num].fdf = 0x01;		//ramka w formacie CANFD
	pC->Can.TxMsgs[num].brs = 0x01;		//zmienna predkosc - BRS = On
	pC->Can.TxMsgs[num].dlc = 0x00;		//0 bajtow w ramce
	pC->Can.TxMsgs[num].r0 = (pC->Can.TxMsgs[num].esi << 31) | (pC->Can.TxMsgs[num].xtd << 30) | (pC->Can.TxMsgs[num].rtr << 29) | (pC->Can.TxMsgs[num].id << 18);
	pC->Can.TxMsgs[num].r1 = (pC->Can.TxMsgs[num].mm << 24) | (pC->Can.TxMsgs[num].efc << 23) | (pC->Can.TxMsgs[num].fdf << 21) | (pC->Can.TxMsgs[num].brs << 20) | (pC->Can.TxMsgs[num].dlc << 16);

	// TxBuffer 8
	num = Can_TxF_ResetJoint3;
	pC->Can.TxMsgs[num].reqSend = false;
	pC->Can.TxMsgs[num].funSendFrame = Can_SendFrameResetJoint3;
	pC->Can.TxMsgs[num].esi = 0x00;		//passive error
	pC->Can.TxMsgs[num].xtd = 0x00;		//standardowe identyfikatory 11bit
	pC->Can.TxMsgs[num].rtr = 0x00;		//ramka z danymi
	pC->Can.TxMsgs[num].id = 0x01A3;	//identyfikator 11 bitowy
	pC->Can.TxMsgs[num].mm = 0x00;		//marker do event, nie uzywany
	pC->Can.TxMsgs[num].efc = 0x00;		//bez geneorwanie eventow
	pC->Can.TxMsgs[num].fdf = 0x01;		//ramka w formacie CANFD
	pC->Can.TxMsgs[num].brs = 0x01;		//zmienna predkosc - BRS = On
	pC->Can.TxMsgs[num].dlc = 0x00;		//0 bajtow w ramce
	pC->Can.TxMsgs[num].r0 = (pC->Can.TxMsgs[num].esi << 31) | (pC->Can.TxMsgs[num].xtd << 30) | (pC->Can.TxMsgs[num].rtr << 29) | (pC->Can.TxMsgs[num].id << 18);
	pC->Can.TxMsgs[num].r1 = (pC->Can.TxMsgs[num].mm << 24) | (pC->Can.TxMsgs[num].efc << 23) | (pC->Can.TxMsgs[num].fdf << 21) | (pC->Can.TxMsgs[num].brs << 20) | (pC->Can.TxMsgs[num].dlc << 16);

	// TxBuffer 9
	num = Can_TxF_ResetJoint4;
	pC->Can.TxMsgs[num].reqSend = false;
	pC->Can.TxMsgs[num].funSendFrame = Can_SendFrameResetJoint4;
	pC->Can.TxMsgs[num].esi = 0x00;		//passive error
	pC->Can.TxMsgs[num].xtd = 0x00;		//standardowe identyfikatory 11bit
	pC->Can.TxMsgs[num].rtr = 0x00;		//ramka z danymi
	pC->Can.TxMsgs[num].id = 0x01A4;	//identyfikator 11 bitowy
	pC->Can.TxMsgs[num].mm = 0x00;		//marker do event, nie uzywany
	pC->Can.TxMsgs[num].efc = 0x00;		//bez geneorwanie eventow
	pC->Can.TxMsgs[num].fdf = 0x01;		//ramka w formacie CANFD
	pC->Can.TxMsgs[num].brs = 0x01;		//zmienna predkosc - BRS = On
	pC->Can.TxMsgs[num].dlc = 0x00;		//0 bajtow w ramce
	pC->Can.TxMsgs[num].r0 = (pC->Can.TxMsgs[num].esi << 31) | (pC->Can.TxMsgs[num].xtd << 30) | (pC->Can.TxMsgs[num].rtr << 29) | (pC->Can.TxMsgs[num].id << 18);
	pC->Can.TxMsgs[num].r1 = (pC->Can.TxMsgs[num].mm << 24) | (pC->Can.TxMsgs[num].efc << 23) | (pC->Can.TxMsgs[num].fdf << 21) | (pC->Can.TxMsgs[num].brs << 20) | (pC->Can.TxMsgs[num].dlc << 16);

	// TxBuffer 10
	num = Can_TxF_ResetJoint5;
	pC->Can.TxMsgs[num].reqSend = false;
	pC->Can.TxMsgs[num].funSendFrame = Can_SendFrameResetJoint5;
	pC->Can.TxMsgs[num].esi = 0x00;		//passive error
	pC->Can.TxMsgs[num].xtd = 0x00;		//standardowe identyfikatory 11bit
	pC->Can.TxMsgs[num].rtr = 0x00;		//ramka z danymi
	pC->Can.TxMsgs[num].id = 0x01A5;	//identyfikator 11 bitowy
	pC->Can.TxMsgs[num].mm = 0x00;		//marker do event, nie uzywany
	pC->Can.TxMsgs[num].efc = 0x00;		//bez geneorwanie eventow
	pC->Can.TxMsgs[num].fdf = 0x01;		//ramka w formacie CANFD
	pC->Can.TxMsgs[num].brs = 0x01;		//zmienna predkosc - BRS = On
	pC->Can.TxMsgs[num].dlc = 0x00;		//0 bajtow w ramce
	pC->Can.TxMsgs[num].r0 = (pC->Can.TxMsgs[num].esi << 31) | (pC->Can.TxMsgs[num].xtd << 30) | (pC->Can.TxMsgs[num].rtr << 29) | (pC->Can.TxMsgs[num].id << 18);
	pC->Can.TxMsgs[num].r1 = (pC->Can.TxMsgs[num].mm << 24) | (pC->Can.TxMsgs[num].efc << 23) | (pC->Can.TxMsgs[num].fdf << 21) | (pC->Can.TxMsgs[num].brs << 20) | (pC->Can.TxMsgs[num].dlc << 16);

	// TxBuffer 11
	num = Can_TxF_ResetGripper;
	pC->Can.TxMsgs[num].reqSend = false;
	pC->Can.TxMsgs[num].funSendFrame = Can_SendFrameResetGripper;
	pC->Can.TxMsgs[num].esi = 0x00;		//passive error
	pC->Can.TxMsgs[num].xtd = 0x00;		//standardowe identyfikatory 11bit
	pC->Can.TxMsgs[num].rtr = 0x00;		//ramka z danymi
	pC->Can.TxMsgs[num].id = 0x01A6;	//identyfikator 11 bitowy
	pC->Can.TxMsgs[num].mm = 0x00;		//marker do event, nie uzywany
	pC->Can.TxMsgs[num].efc = 0x00;		//bez geneorwanie eventow
	pC->Can.TxMsgs[num].fdf = 0x01;		//ramka w formacie CANFD
	pC->Can.TxMsgs[num].brs = 0x01;		//zmienna predkosc - BRS = On
	pC->Can.TxMsgs[num].dlc = 0x00;		//0 bajtow w ramce
	pC->Can.TxMsgs[num].r0 = (pC->Can.TxMsgs[num].esi << 31) | (pC->Can.TxMsgs[num].xtd << 30) | (pC->Can.TxMsgs[num].rtr << 29) | (pC->Can.TxMsgs[num].id << 18);
	pC->Can.TxMsgs[num].r1 = (pC->Can.TxMsgs[num].mm << 24) | (pC->Can.TxMsgs[num].efc << 23) | (pC->Can.TxMsgs[num].fdf << 21) | (pC->Can.TxMsgs[num].brs << 20) | (pC->Can.TxMsgs[num].dlc << 16);
	
}
static void Can_FdcanConf(void)
{
	// pin configuration
	GPIOD->MODER &= ~GPIO_MODER_MODE0 & ~GPIO_MODER_MODE1;
	GPIOD->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1;
	GPIOD->AFR[0] |= 0x00000099;
	
	// FDCAN setup begins
	FDCAN1->CCCR |= FDCAN_CCCR_INIT | FDCAN_CCCR_CCE;
	// Frame in CANFD format with variable speed (BRSE bit)
	FDCAN1->CCCR |= FDCAN_CCCR_BRSE | FDCAN_CCCR_FDOE | FDCAN_CCCR_TXP;
	
	// transmission baudrate in nominal mode
	FDCAN1->NBTP = (0x01 << FDCAN_NBTP_NSJW_Pos) | (0x04 << FDCAN_NBTP_NBRP_Pos) | (0x0B << FDCAN_NBTP_NTSEG1_Pos) | (0x03 << FDCAN_NBTP_NTSEG2_Pos);
	// transmission baudrate in data mode
	FDCAN1->DBTP = (0x01 << FDCAN_DBTP_DSJW_Pos) | (0x00 << FDCAN_DBTP_DBRP_Pos) | (0x0B << FDCAN_DBTP_DTSEG1_Pos) | (0x03 << FDCAN_DBTP_DTSEG2_Pos);
	
	// all remote and non-filtered frames are discarded
	FDCAN1->GFC = (0x03 << FDCAN_GFC_ANFS_Pos) | (0x03 << FDCAN_GFC_ANFE_Pos) | FDCAN_GFC_RRFS | FDCAN_GFC_RRFE;
	// CAN_FILTERS_MAX of standard filters and the address of the filters
	FDCAN1->SIDFC = (CAN_FILTERS_MAX << FDCAN_SIDFC_LSS_Pos) | (pC->Can.filterAddrOffset << 0); 
	
	// CAN_TXBUF_MAX send buffers and address of the first send buffer
	FDCAN1->TXBC = (CAN_TXBUF_MAX << FDCAN_TXBC_NDTB_Pos) | (pC->Can.txBufAddrOffset << 0);
	// transmit buffers with a size of CAN_TXDATA_LEN bytes
	FDCAN1->TXESC = (CAN_TXBUFSIZE_CODE << FDCAN_TXESC_TBDS_Pos);
	
	// CAN_RXDATA_LEN byte receiving buffers, RXFIFO 1 and RXFIFO 0 elements with a size of CAN_RXDATA_LEN bytes
	FDCAN1->RXESC = (CAN_RXBUFSIZE_CODE << FDCAN_RXESC_RBDS_Pos) | (CAN_RXBUFSIZE_CODE << FDCAN_RXESC_F1DS_Pos) | (CAN_RXBUFSIZE_CODE << FDCAN_RXESC_F0DS_Pos);
	// first receive buffer address offset
	FDCAN1->RXBC = (pC->Can.rxBufAddrOffset << 0); 
	// CAN_RXBUFF_MAX receive buffers, CAN_RXFIFO0_MAX fifo0 and address of first fifo0
	FDCAN1->RXF0C = (CAN_RXFIFO0_MAX << FDCAN_RXF0C_F0S_Pos) | (pC->Can.rxFifo0AddrOffset << 0); 

	// abort from receive to buffer, these interrupts are directed to EINT0
	FDCAN1->IE = FDCAN_IE_TCE | FDCAN_IE_DRXE;
	// Enable interrupts from transfer complete individually for each send buffer
	for(int i=0;i<CAN_TXBUF_MAX;i++)
		FDCAN1->TXBTIE |= (1 << i);
	// The error handling interrupt, these interrupts are directed to EINT1
	FDCAN1->IE |= FDCAN_IE_ARAE | FDCAN_IE_PEDE | FDCAN_IE_PEAE | FDCAN_IE_WDIE | FDCAN_IE_BOE | FDCAN_IE_EWE | FDCAN_IE_EPE | FDCAN_IE_ELOE;
	FDCAN1->ILS = FDCAN_ILS_ARAE | FDCAN_ILS_PEDE | FDCAN_ILS_PEAE | FDCAN_ILS_WDIE | FDCAN_ILS_BOE | FDCAN_ILS_EWE | FDCAN_ILS_EPE | FDCAN_ILS_ELOE;
	// turning on the interrupt line
	FDCAN1->ILE = FDCAN_ILE_EINT0 | FDCAN_ILE_EINT1;
	NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
}
static void Can_Start(void)
{
	// koniec konfiguracji i uruchomienie FDCAN
	FDCAN1->CCCR &= ~FDCAN_CCCR_INIT; 
}
void Can_Conf(void)
{
	Can_StructConf();
	Can_FiltersConf();
	Can_TxBufferConf();
	Can_FdcanConf();
	Can_Start();
}
static void Can_SendFrameOccurred(void)
{
	for(int num=0;num<CAN_TXBUF_MAX;num++)
	{
		if(((FDCAN1->TXBTO >> num) & 0x01) != RESET && pC->Can.TxMsgs[num].status == Can_TxS_Sending)
		{
			pC->Can.TxMsgs[num].status = Can_TxS_Sent;
			pC->Can.TxMsgs[num].timeoutCnt = 0;
			pC->Can.TxMsgs[num].frameTotalCnt++;
		}
	}
}
static void Can_SendFrameToDevices(void)
{
	for(int i=0;i<CAN_TXBUF_MAX;i++)
	{
		pC->Can.frameToSend++;
		pC->Can.frameToSend = pC->Can.frameToSend % CAN_TXBUF_MAX;
		if(pC->Can.TxMsgs[pC->Can.frameToSend].reqSend == true)
		{
			pC->Can.TxMsgs[pC->Can.frameToSend].funSendFrame();
			pC->Can.TxMsgs[pC->Can.frameToSend].reqSend = false;
			break;
		}
		else
		{
			continue;
		}
	}
}
uint32_t n[4]={0,0,0,0};
static void Can_ReadFrameMoveResponse(uint8_t num)
{
	n[0]++;
	uint16_t idx = ((CAN_RXDATA_LEN/4)+2) * num;
	uint16_t devNum = num % CAN_DEVICESMAX;
	pC->Can.RxMsgs[num].r0 = *(pC->Can.rxBufAddr + idx + 0);
	pC->Can.RxMsgs[num].r1 = *(pC->Can.rxBufAddr + idx + 1);
	pC->Can.RxMsgs[num].data[0] = *(pC->Can.rxBufAddr + idx + 2);
	pC->Can.RxMsgs[num].data[1] = *(pC->Can.rxBufAddr + idx + 3);
	pC->Can.RxMsgs[num].data[2] = *(pC->Can.rxBufAddr + idx + 4);
	pC->Can.RxMsgs[num].data[3] = *(pC->Can.rxBufAddr + idx + 5);
	pC->Can.RxMsgs[num].data[4] = *(pC->Can.rxBufAddr + idx + 6);
	pC->Can.RxMsgs[num].data[5] = *(pC->Can.rxBufAddr + idx + 7);
	
	pC->Can.RxMsgs[num].esi = (pC->Can.RxMsgs[num].r0 >> 31) & 0x01;
	pC->Can.RxMsgs[num].xtd = (pC->Can.RxMsgs[num].r0 >> 30) & 0x01;
	pC->Can.RxMsgs[num].xtd = (pC->Can.RxMsgs[num].r0 >> 29) & 0x01;
	pC->Can.RxMsgs[num].id = (pC->Can.RxMsgs[num].r0 >> 18) & 0x07FF;
	
	pC->Can.RxMsgs[num].anmf = (pC->Can.RxMsgs[num].r1 >> 31) & 0x01;
	pC->Can.RxMsgs[num].fidx = (pC->Can.RxMsgs[num].r1 >> 24) & 0x3F;
	pC->Can.RxMsgs[num].fdf = (pC->Can.RxMsgs[num].r1 >> 21) & 0x01;
	pC->Can.RxMsgs[num].brs = (pC->Can.RxMsgs[num].r1 >> 20) & 0x01;
	pC->Can.RxMsgs[num].dlc = (pC->Can.RxMsgs[num].r1 >> 16) & 0x0F;
	pC->Can.RxMsgs[num].rxts = (pC->Can.RxMsgs[num].r1 >> 16) & 0xFFFF;
	
	for(int i=0;i<(CAN_RXDATA_LEN/4);i++)
	{
		pC->Can.RxMsgs[num].bytes[4*i+0] = pC->Can.RxMsgs[num].data[i] >> 0;
		pC->Can.RxMsgs[num].bytes[4*i+1] = pC->Can.RxMsgs[num].data[i] >> 8;
		pC->Can.RxMsgs[num].bytes[4*i+2] = pC->Can.RxMsgs[num].data[i] >> 16;
		pC->Can.RxMsgs[num].bytes[4*i+3] = pC->Can.RxMsgs[num].data[i] >> 24;
	}
	
	if(devNum <= Can_DN_Joint5)
	{
		// pozycja jest w rad
		pC->Joints[devNum].currentPos = (double)((int32_t)(((uint32_t)pC->Can.RxMsgs[num].bytes[0] << 24) + ((uint32_t)pC->Can.RxMsgs[num].bytes[1] << 16) + ((uint32_t)pC->Can.RxMsgs[num].bytes[2] << 8) + ((uint32_t)pC->Can.RxMsgs[num].bytes[3] << 0))) * pC->Joints[num].maxPosCom / MAXINT32;
		// predkosc jest w rad/s
		pC->Joints[devNum].currentVel = (double)((int16_t)(((uint16_t)pC->Can.RxMsgs[num].bytes[4] << 8) + ((uint16_t)pC->Can.RxMsgs[num].bytes[5] << 0))) * pC->Joints[num].maxVelCom / MAXINT16;
		// moment obrotowy jest w Nm
		pC->Joints[devNum].currentTorque = (double)((int16_t)(((uint16_t)pC->Can.RxMsgs[num].bytes[6] << 8) + ((uint16_t)pC->Can.RxMsgs[num].bytes[7] << 0))) * pC->Joints[num].maxTorqueCom / MAXINT16;
		pC->Joints[devNum].currentBearingTemp = (double)pC->Can.RxMsgs[num].bytes[8];
		pC->Joints[devNum].currentFsm = (eJoint_FSM)pC->Can.RxMsgs[num].bytes[9];
		pC->Joints[devNum].mcCurrentError = pC->Can.RxMsgs[num].bytes[10];
		pC->Joints[devNum].mcOccuredError = pC->Can.RxMsgs[num].bytes[11];
		pC->Joints[devNum].currentError = pC->Can.RxMsgs[num].bytes[12];
		pC->Joints[devNum].currentWarning = pC->Can.RxMsgs[num].bytes[13];
		pC->Joints[devNum].ma70CurrentValue = ((uint16_t)pC->Can.RxMsgs[num].bytes[14] << 8) + ((uint16_t)pC->Can.RxMsgs[num].bytes[15] << 0);
		pC->Joints[devNum].currentBearingTemp = (double)pC->Can.RxMsgs[num].bytes[16];
		
		pC->Joints[devNum].flagFirstPosRead = true;
		pC->Joints[devNum].cWPosNotAccurate = pC->Joints[devNum].currentWarning & 0x01;
	}
	else if(devNum == Can_DN_Gripper)
	{
		// aktualny stan pompy
		pC->Gripper.currentPumpState = pC->Can.RxMsgs[num].bytes[0];
		// osiagnieto poziom cisnienia 1
		pC->Gripper.pressure1 = pC->Can.RxMsgs[num].bytes[1];
		// osiagnieto poziom cisnienia 1
		pC->Gripper.pressure2 = pC->Can.RxMsgs[num].bytes[2];
		
		pC->Gripper.currentFsm = (eJoint_FSM)pC->Can.RxMsgs[num].bytes[9];
		pC->Gripper.currentError = pC->Can.RxMsgs[num].bytes[12];
		pC->Gripper.currentWarning = pC->Can.RxMsgs[num].bytes[13];
		
		pC->Gripper.flagFirstPosRead = true;
	}
	
	pC->Can.RxMsgs[num].timeoutCnt = 0;
	pC->Can.RxMsgs[num].frameTotalCnt++;
	pC->Can.RxMsgs[num].status = Can_RxS_Idle;
}
static void Can_ReadFrameChangeFsmResponse(uint8_t num)
{
	n[1]++;
	uint16_t idx = ((CAN_RXDATA_LEN/4)+2) * num;
	uint16_t devNum = num % CAN_DEVICESMAX;
	pC->Can.RxMsgs[num].r0 = *(pC->Can.rxBufAddr + idx + 0);
	pC->Can.RxMsgs[num].r1 = *(pC->Can.rxBufAddr + idx + 1);
	pC->Can.RxMsgs[num].data[0] = *(pC->Can.rxBufAddr + idx + 2);
	pC->Can.RxMsgs[num].data[1] = *(pC->Can.rxBufAddr + idx + 3);
	pC->Can.RxMsgs[num].data[2] = *(pC->Can.rxBufAddr + idx + 4);
	pC->Can.RxMsgs[num].data[3] = *(pC->Can.rxBufAddr + idx + 5);
	pC->Can.RxMsgs[num].data[4] = *(pC->Can.rxBufAddr + idx + 6);
	pC->Can.RxMsgs[num].data[5] = *(pC->Can.rxBufAddr + idx + 7);
	
	pC->Can.RxMsgs[num].esi = (pC->Can.RxMsgs[num].r0 >> 31) & 0x01;
	pC->Can.RxMsgs[num].xtd = (pC->Can.RxMsgs[num].r0 >> 30) & 0x01;
	pC->Can.RxMsgs[num].xtd = (pC->Can.RxMsgs[num].r0 >> 29) & 0x01;
	pC->Can.RxMsgs[num].id = (pC->Can.RxMsgs[num].r0 >> 18) & 0x07FF;
	
	pC->Can.RxMsgs[num].anmf = (pC->Can.RxMsgs[num].r1 >> 31) & 0x01;
	pC->Can.RxMsgs[num].fidx = (pC->Can.RxMsgs[num].r1 >> 24) & 0x3F;
	pC->Can.RxMsgs[num].fdf = (pC->Can.RxMsgs[num].r1 >> 21) & 0x01;
	pC->Can.RxMsgs[num].brs = (pC->Can.RxMsgs[num].r1 >> 20) & 0x01;
	pC->Can.RxMsgs[num].dlc = (pC->Can.RxMsgs[num].r1 >> 16) & 0x0F;
	pC->Can.RxMsgs[num].rxts = (pC->Can.RxMsgs[num].r1 >> 16) & 0xFFFF;
	
	for(int i=0;i<(CAN_RXDATA_LEN/4);i++)
	{
		pC->Can.RxMsgs[num].bytes[4*i+0] = pC->Can.RxMsgs[num].data[i] >> 0;
		pC->Can.RxMsgs[num].bytes[4*i+1] = pC->Can.RxMsgs[num].data[i] >> 8;
		pC->Can.RxMsgs[num].bytes[4*i+2] = pC->Can.RxMsgs[num].data[i] >> 16;
		pC->Can.RxMsgs[num].bytes[4*i+3] = pC->Can.RxMsgs[num].data[i] >> 24;
	}
	
	if(devNum <= Can_DN_Joint5)
	{
		pC->Joints[devNum].currentFsm = (eJoint_FSM)pC->Can.RxMsgs[num].bytes[0];
	}
	else if(devNum == Can_DN_Gripper)
	{
		pC->Gripper.currentFsm = (eJoint_FSM)pC->Can.RxMsgs[num].bytes[0];
	}
	
	pC->Can.RxMsgs[num].timeoutCnt = 0;
	pC->Can.RxMsgs[num].frameTotalCnt++;
	pC->Can.RxMsgs[num].status = Can_RxS_Idle;
}
static void Can_ReadFrameChangeConfResponse(uint8_t num)
{
	n[2]++;
	uint16_t idx = ((CAN_RXDATA_LEN/4)+2) * num;
	uint16_t devNum = num % CAN_DEVICESMAX;
	pC->Can.RxMsgs[num].r0 = *(pC->Can.rxBufAddr + idx + 0);
	pC->Can.RxMsgs[num].r1 = *(pC->Can.rxBufAddr + idx + 1);
	pC->Can.RxMsgs[num].data[0] = *(pC->Can.rxBufAddr + idx + 2);
	pC->Can.RxMsgs[num].data[1] = *(pC->Can.rxBufAddr + idx + 3);
	pC->Can.RxMsgs[num].data[2] = *(pC->Can.rxBufAddr + idx + 4);
	pC->Can.RxMsgs[num].data[3] = *(pC->Can.rxBufAddr + idx + 5);
	pC->Can.RxMsgs[num].data[4] = *(pC->Can.rxBufAddr + idx + 6);
	pC->Can.RxMsgs[num].data[5] = *(pC->Can.rxBufAddr + idx + 7);
	
	pC->Can.RxMsgs[num].esi = (pC->Can.RxMsgs[num].r0 >> 31) & 0x01;
	pC->Can.RxMsgs[num].xtd = (pC->Can.RxMsgs[num].r0 >> 30) & 0x01;
	pC->Can.RxMsgs[num].xtd = (pC->Can.RxMsgs[num].r0 >> 29) & 0x01;
	pC->Can.RxMsgs[num].id = (pC->Can.RxMsgs[num].r0 >> 18) & 0x07FF;
	
	pC->Can.RxMsgs[num].anmf = (pC->Can.RxMsgs[num].r1 >> 31) & 0x01;
	pC->Can.RxMsgs[num].fidx = (pC->Can.RxMsgs[num].r1 >> 24) & 0x3F;
	pC->Can.RxMsgs[num].fdf = (pC->Can.RxMsgs[num].r1 >> 21) & 0x01;
	pC->Can.RxMsgs[num].brs = (pC->Can.RxMsgs[num].r1 >> 20) & 0x01;
	pC->Can.RxMsgs[num].dlc = (pC->Can.RxMsgs[num].r1 >> 16) & 0x0F;
	pC->Can.RxMsgs[num].rxts = (pC->Can.RxMsgs[num].r1 >> 16) & 0xFFFF;
	
	for(int i=0;i<(CAN_RXDATA_LEN/4);i++)
	{
		pC->Can.RxMsgs[num].bytes[4*i+0] = pC->Can.RxMsgs[num].data[i] >> 0;
		pC->Can.RxMsgs[num].bytes[4*i+1] = pC->Can.RxMsgs[num].data[i] >> 8;
		pC->Can.RxMsgs[num].bytes[4*i+2] = pC->Can.RxMsgs[num].data[i] >> 16;
		pC->Can.RxMsgs[num].bytes[4*i+3] = pC->Can.RxMsgs[num].data[i] >> 24;
	}
	
	if(devNum <= Can_DN_Joint5)
	{
		if(pC->Can.RxMsgs[num].bytes[0] == 1)
		{
			pC->Joints[devNum].flagConfirmChangeConf = true;
			pC->Joints[devNum].currentMode = pC->Joints[devNum].targetMode;
		}
	}
	else if(devNum == Can_DN_Gripper)
	{
		if(pC->Can.RxMsgs[num].bytes[0] == 1)
			pC->Gripper.flagConfirmChangeConf = true;
	}
	
	pC->Can.RxMsgs[num].timeoutCnt = 0;
	pC->Can.RxMsgs[num].frameTotalCnt++;
	pC->Can.RxMsgs[num].status = Can_RxS_Idle;
}

static void Can_ReadFrameReadFrictionResponse(uint8_t num)
{
	n[3]++;
	uint16_t idx = ((CAN_RXDATA_LEN/4)+2) * num;
	uint16_t devNum = num % CAN_DEVICESMAX;
	pC->Can.RxMsgs[num].r0 = *(pC->Can.rxBufAddr + idx + 0);
	pC->Can.RxMsgs[num].r1 = *(pC->Can.rxBufAddr + idx + 1);
	pC->Can.RxMsgs[num].data[0] = *(pC->Can.rxBufAddr + idx + 2);
	pC->Can.RxMsgs[num].data[1] = *(pC->Can.rxBufAddr + idx + 3);
	pC->Can.RxMsgs[num].data[2] = *(pC->Can.rxBufAddr + idx + 4);
	pC->Can.RxMsgs[num].data[3] = *(pC->Can.rxBufAddr + idx + 5);
	pC->Can.RxMsgs[num].data[4] = *(pC->Can.rxBufAddr + idx + 6);
	pC->Can.RxMsgs[num].data[5] = *(pC->Can.rxBufAddr + idx + 7);
	
	pC->Can.RxMsgs[num].esi = (pC->Can.RxMsgs[num].r0 >> 31) & 0x01;
	pC->Can.RxMsgs[num].xtd = (pC->Can.RxMsgs[num].r0 >> 30) & 0x01;
	pC->Can.RxMsgs[num].xtd = (pC->Can.RxMsgs[num].r0 >> 29) & 0x01;
	pC->Can.RxMsgs[num].id = (pC->Can.RxMsgs[num].r0 >> 18) & 0x07FF;
	
	pC->Can.RxMsgs[num].anmf = (pC->Can.RxMsgs[num].r1 >> 31) & 0x01;
	pC->Can.RxMsgs[num].fidx = (pC->Can.RxMsgs[num].r1 >> 24) & 0x3F;
	pC->Can.RxMsgs[num].fdf = (pC->Can.RxMsgs[num].r1 >> 21) & 0x01;
	pC->Can.RxMsgs[num].brs = (pC->Can.RxMsgs[num].r1 >> 20) & 0x01;
	pC->Can.RxMsgs[num].dlc = (pC->Can.RxMsgs[num].r1 >> 16) & 0x0F;
	pC->Can.RxMsgs[num].rxts = (pC->Can.RxMsgs[num].r1 >> 16) & 0xFFFF;
	
	for(int i=0;i<(CAN_RXDATA_LEN/4);i++)
	{
		pC->Can.RxMsgs[num].bytes[4*i+0] = pC->Can.RxMsgs[num].data[i] >> 0;
		pC->Can.RxMsgs[num].bytes[4*i+1] = pC->Can.RxMsgs[num].data[i] >> 8;
		pC->Can.RxMsgs[num].bytes[4*i+2] = pC->Can.RxMsgs[num].data[i] >> 16;
		pC->Can.RxMsgs[num].bytes[4*i+3] = pC->Can.RxMsgs[num].data[i] >> 24;
	}
	
	union conv32 x;
	uint32_t idx2=0;
	if(devNum <= Can_DN_Joint5)
	{
		x.u32 = (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 24;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 16;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 8;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 0;
		pC->Joints[devNum].fricCoeffFromCan[0] = x.f32;
		
		x.u32 = (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 24;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 16;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 8;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 0;
		pC->Joints[devNum].fricCoeffFromCan[1] = x.f32;
		
		x.u32 = (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 24;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 16;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 8;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 0;
		pC->Joints[devNum].fricCoeffFromCan[2] = x.f32;
		
		x.u32 = (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 24;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 16;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 8;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 0;
		pC->Joints[devNum].fricCoeffFromCan[3] = x.f32;
		
		x.u32 = (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 24;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 16;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 8;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 0;
		pC->Joints[devNum].fricCoeffFromCan[4] = x.f32;
		
		x.u32 = (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 24;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 16;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 8;
		x.u32 += (uint32_t)pC->Can.RxMsgs[num].bytes[idx2++] << 0;
		pC->Joints[devNum].fricCoeffFromCan[5] = x.f32;
		
		pC->Jtc.fricType = JTC_FT_Polynomial;
		Joints_SetFrictionPolynomialFromCan(devNum);
		pC->Joints[devNum].flagFrictionReadFromCan = true;
	}
	
	pC->Can.RxMsgs[num].timeoutCnt = 0;
	pC->Can.RxMsgs[num].frameTotalCnt++;
	pC->Can.RxMsgs[num].status = Can_RxS_Idle;
}
static void Can_ReadFramesFromBuffer(void)
{
	for(int num=0;num<CAN_DEVICESMAX;num++)
	{
		if(((FDCAN1->NDAT1 >> num) & 0x01) != RESET)
		{
			Can_ReadFrameMoveResponse(num);
			FDCAN1->NDAT1 = (1 << num);
		}
	}
	for(int num=CAN_DEVICESMAX;num<2*CAN_DEVICESMAX;num++)
	{
		if(((FDCAN1->NDAT1 >> num) & 0x01) != RESET)
		{
			Can_ReadFrameChangeFsmResponse(num);
			FDCAN1->NDAT1 = (1 << num);
		}
	}
	for(int num=2*CAN_DEVICESMAX;num<3*CAN_DEVICESMAX;num++)
	{
		if(((FDCAN1->NDAT1 >> num) & 0x01) != RESET)
		{
			Can_ReadFrameChangeConfResponse(num);
			FDCAN1->NDAT1 = (1 << num);
		}
	}
	for(int num=3*CAN_DEVICESMAX;num<4*CAN_DEVICESMAX;num++)
	{
		if(((FDCAN1->NDAT1 >> num) & 0x01) != RESET)
		{
			Can_ReadFrameReadFrictionResponse(num);
			FDCAN1->NDAT1 = (1 << num);
		}
	}
}
static void Can_TimeoutCntInc(void)
{
	for(int num=0;num<CAN_TXBUF_MAX;num++)
		pC->Can.TxMsgs[num].timeoutCnt++;
		
	for(int num=0;num<CAN_RXBUF_MAX;num++)
		pC->Can.RxMsgs[num].timeoutCnt++;
}
static void Can_CheckTxStatus(void)
{
	for(int num=0;num<CAN_TXBUF_MAX;num++)
	{
		if(pC->Can.TxMsgs[num].timeoutCnt >= CAN_TIMEOUTMAX)
		{
			pC->Can.TxMsgs[num].timeoutCnt = CAN_TIMEOUTMAX;
			pC->Can.TxMsgs[num].flagTimeout = true;
		}
		else
			pC->Can.TxMsgs[num].flagTimeout = false;
	}
}
static void Can_CheckRxStatus(void)
{
	for(int num=0;num<CAN_RXBUF_MAX;num++)
	{
		if(pC->Can.RxMsgs[num].timeoutCnt >= CAN_TIMEOUTMAX)
		{
			pC->Can.RxMsgs[num].timeoutCnt = CAN_TIMEOUTMAX;
			pC->Can.RxMsgs[num].flagTimeout = true;
		}
		else
			pC->Can.RxMsgs[num].flagTimeout = false;
	}
}
static void Can_CheckCanStatus(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		if(pC->Joints[num].reqIgnore == true)
		{
			pC->Can.RxMsgs[num+0].flagTimeout = false;
			pC->Can.RxMsgs[num+0].timeoutCnt = 0x00;
			pC->Can.RxMsgs[num+7].flagTimeout = false;
			pC->Can.RxMsgs[num+7].timeoutCnt = 0x00;
			pC->Can.RxMsgs[num+14].flagTimeout = false;
			pC->Can.RxMsgs[num+14].timeoutCnt = 0x00;
			pC->Can.RxMsgs[num+21].flagTimeout = false;
			pC->Can.RxMsgs[num+21].timeoutCnt = 0x00;
		}
	}
	if(pC->Gripper.reqIgnore == true)
	{
		pC->Can.RxMsgs[6].flagTimeout = false;
		pC->Can.RxMsgs[6].timeoutCnt = 0x00;
		pC->Can.RxMsgs[13].flagTimeout = false;
		pC->Can.RxMsgs[13].timeoutCnt = 0x00;
		pC->Can.RxMsgs[20].flagTimeout = false;
		pC->Can.RxMsgs[20].timeoutCnt = 0x00;
		pC->Can.RxMsgs[27].flagTimeout = false;
		pC->Can.RxMsgs[27].timeoutCnt = 0x00;
	}
	
	
	pC->Can.statusFlags = 0x00;
	// Bajt 0 dla Tx Timeout
	bool flag = true;
	for(int i=0;i<CAN_TXBUF_MAX;i++)
		if(pC->Can.TxMsgs[i].flagTimeout == false)
			flag = false;
	if(flag == true)
		pC->Can.statusFlags |= (1 << Can_SFP_Tx0Timeout);

	// Bajt 1 dla Rx Timeout
	if(pC->Can.RxMsgs[0].flagTimeout && pC->Can.RxMsgs[7].flagTimeout && pC->Can.RxMsgs[14].flagTimeout && pC->Can.RxMsgs[21].flagTimeout)		pC->Can.statusFlags |= (1 << Can_SFP_Rx0Timeout);
	if(pC->Can.RxMsgs[1].flagTimeout && pC->Can.RxMsgs[8].flagTimeout && pC->Can.RxMsgs[15].flagTimeout && pC->Can.RxMsgs[22].flagTimeout)		pC->Can.statusFlags |= (1 << Can_SFP_Rx1Timeout);
	if(pC->Can.RxMsgs[2].flagTimeout && pC->Can.RxMsgs[9].flagTimeout && pC->Can.RxMsgs[16].flagTimeout && pC->Can.RxMsgs[23].flagTimeout)		pC->Can.statusFlags |= (1 << Can_SFP_Rx2Timeout);
	if(pC->Can.RxMsgs[3].flagTimeout && pC->Can.RxMsgs[10].flagTimeout && pC->Can.RxMsgs[17].flagTimeout && pC->Can.RxMsgs[24].flagTimeout)		pC->Can.statusFlags |= (1 << Can_SFP_Rx3Timeout);
	if(pC->Can.RxMsgs[4].flagTimeout && pC->Can.RxMsgs[11].flagTimeout && pC->Can.RxMsgs[18].flagTimeout && pC->Can.RxMsgs[25].flagTimeout)		pC->Can.statusFlags |= (1 << Can_SFP_Rx4Timeout);
	if(pC->Can.RxMsgs[5].flagTimeout && pC->Can.RxMsgs[12].flagTimeout && pC->Can.RxMsgs[19].flagTimeout && pC->Can.RxMsgs[26].flagTimeout)		pC->Can.statusFlags |= (1 << Can_SFP_Rx5Timeout);
	if(pC->Can.RxMsgs[6].flagTimeout && pC->Can.RxMsgs[13].flagTimeout && pC->Can.RxMsgs[20].flagTimeout && pC->Can.RxMsgs[27].flagTimeout)		pC->Can.statusFlags |= (1 << Can_SFP_Rx6Timeout);
		
	if(pC->Can.statusFlags != 0x0000)
		pC->Can.statusId = Can_SId_Error;
	else
		pC->Can.statusId = Can_SId_NoError;
	
	pC->Can.statusOccurredFlags |= pC->Can.statusFlags;
}
static void Can_CheckStatus(void)
{
	Can_CheckTxStatus();
	Can_CheckRxStatus();
	Can_CheckCanStatus();	
}
void Can_SendDataToJoints(void)
{
	Can_SendFrameToDevices();
	Can_CheckStatus();
	Can_TimeoutCntInc();
}
void FDCAN1_IT0_IRQHandler(void)
{
	if((FDCAN1->IR & FDCAN_IR_TC) != RESET)
	{
		Can_SendFrameOccurred();
		FDCAN1->IR = FDCAN_IR_TC;
	}
	if((FDCAN1->IR & FDCAN_IR_DRX) != RESET)
	{
		Can_ReadFramesFromBuffer();
		FDCAN1->IR = FDCAN_IR_DRX;
	}
}
void FDCAN1_IT1_IRQHandler(void)
{
	if((FDCAN1->IR & FDCAN_IR_ARA) != RESET)
	{
		FDCAN1->IR = FDCAN_IR_ARA;
	}
	if((FDCAN1->IR & FDCAN_IR_PED) != RESET)
	{
		FDCAN1->IR = FDCAN_IR_PED;
	}
	if((FDCAN1->IR & FDCAN_IR_PEA) != RESET)
	{
		FDCAN1->IR = FDCAN_IR_PEA;
	}
	if((FDCAN1->IR & FDCAN_IR_WDI) != RESET)
	{
		FDCAN1->IR = FDCAN_IR_WDI;
	}
	if((FDCAN1->IR & FDCAN_IR_BO) != RESET)
	{
		FDCAN1->IR = FDCAN_IR_BO;
	}
	if((FDCAN1->IR & FDCAN_IR_EW) != RESET)
	{
		FDCAN1->IR = FDCAN_IR_EW;
	}
	if((FDCAN1->IR & FDCAN_IR_EP) != RESET)
	{
		FDCAN1->IR = FDCAN_IR_EP;
	}
	if((FDCAN1->IR & FDCAN_IR_ELO) != RESET)
	{
		uint32_t reg = FDCAN1->ECR;
		FDCAN1->IR = FDCAN_IR_ELO;
	}
}
