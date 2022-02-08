#ifndef	_CONTROL
#define _CONTROL

#include <stm32h7xx.h>
#include <stm32h745xx.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define LED1_ON				GPIOB->ODR |= GPIO_ODR_OD0;
#define LED1_OFF			GPIOB->ODR &= ~GPIO_ODR_OD0;
#define LED1_TOG			GPIOB->ODR ^= GPIO_ODR_OD0;
#define LED2_ON				GPIOE->ODR |= GPIO_ODR_OD1;
#define LED2_OFF			GPIOE->ODR &= ~GPIO_ODR_OD1;
#define LED2_TOG			GPIOE->ODR ^= GPIO_ODR_OD1;
#define LED3_ON				GPIOB->ODR |= GPIO_ODR_OD14;
#define LED3_OFF			GPIOB->ODR &= ~GPIO_ODR_OD14;
#define LED3_TOG			GPIOB->ODR ^= GPIO_ODR_OD14;
#define LEDALL_ON			LED_PORT->ODR |= LED1_PIN | LED2_PIN | LED3_PIN;
#define LEDALL_OFF		LED_PORT->ODR &= ~LED1_PIN & ~LED2_PIN & ~LED3_PIN;
#define LEDALL_TOG		LED_PORT->ODR ^= LED1_PIN | LED2_PIN | LED3_PIN;

typedef enum 
{
	JTC_FSM_Null = 255,
	JTC_FSM_Start = 0,
	JTC_FSM_Init,
	JTC_FSM_HoldPos,
	JTC_FSM_Operate,
	JTC_FSM_Teaching,
	JTC_FSM_Error,
}eJTC_FSM;
typedef enum 
{
	TES_Null = 0,
	TES_Stop,
	TES_Pause,
	TES_Execute,
	TES_Finish,
	TES_TransNullToStop,
}eTrajExecStatus;
typedef enum
{
	TCS_Null = 0,
	TCS_IsRead = 1,
	TCS_WasRead = 2
}eTrajComStatus;
typedef enum 
{
	Host_FT_Header = 155,
	Host_FT_null = 0,
	Host_FT_ClearCurrentErrors,
	Host_FT_ClearOccuredErrors,
	Host_FT_JtcStatus,
	Host_FT_Trajectory,
	Host_FT_FrictionTable,
	Host_FT_FrictionTableUseDefault,
	Host_FT_PidParam,
	Host_FT_PidParamUseDefault,
	Host_FT_ArmModel,
	Host_FT_ArmModelUseDefault,
	Host_FT_TrajSetExecStatus,
  Host_FT_TeachingModeEnable,
  Host_FT_TeachingModeDisable,
	Host_FT_FrictionPolynomial,
	Host_FT_FrictionPolynomialUseDefault,
}eHost_FrameType;
typedef enum
{
	Host_RxFS_Idle = 0,
	Host_RxFS_NoError,
	Host_RxFS_ErrorIncorrectHeader,
	Host_RxFS_ErrorIncorrectFrameType,
	Host_RxFS_ErrorIncorrectCrc,
	Host_RxFS_ErrorDiscontinuousFrame,
}eHost_RxFrameStatus;
typedef enum
{
	Host_RxDS_Idle = 0,
	Host_RxDS_NoError, //bez bledu
	Host_RxDS_TrajTooManyPoints, //zbyt dluga trajektoria
	Host_RxDS_TrajTooManySegs, //zbyt wiele segmentow
	Host_RxDS_TrajIncorrectSegOrder, //segmenty w niepoprawnej kolejnosci
	Host_RxDS_TrajIncorrectStepTime, //niepoprawny krok czasowy (prawdopodobnie stepTime = 0)
	Host_RxDS_GetFrameIncorectData, //niepoprawne dane dla ramki GetFrame
}eHost_RxDataStatus;
typedef enum
{
	Host_RxFC_WasReceived = 0,
	Host_RxFC_IsReceived = 1,
}eHost_RxFrameConsist;
typedef enum
{
	Host_TxFN_JtcStatus = 0,
}eHost_TxFrameNum;
typedef enum
{
	Host_TxFS_Idle = 0,
	Host_TxFS_ReadyToSend,
	Host_TxFS_Sending,
}eHost_TxFrameStatus;
typedef enum
{
	Host_CS_Idle = 0,
	Host_CS_Sending,
}eHost_ComStatus;
typedef enum
{
	Joint_FSM_Start = 0,
	Joint_FSM_Init = 1,
	Joint_FSM_ReadyToOperate = 2,
	Joint_FSM_OperationEnable = 3,
	Joint_FSM_TransStartToInit = 10,
	Joint_FSM_TransInitToReadyToOperate = 11,
	Joint_FSM_TransReadyToOperateToOperationEnable = 12,
	Joint_FSM_TransOperationEnableToReadyToOperate = 13,
	Joint_FSM_TransFaulyReactionActiveToFault = 14,
	Joint_FSM_TransFaultToReadyToOperate= 15,
	Joint_FSM_ReactionActive = 254,
	Joint_FSM_Fault = 255
}eJoint_FSM;
typedef enum
{
	Can_TxS_Idle = 0,
	Can_TxS_Sending = 1,
	Can_TxS_Sent = 2
}eCanTxStatus;
typedef enum
{
	Can_RxS_Idle = 0,
	Can_RxS_WaitingForResponse,
	Can_RxS_Reading,
	Can_RxS_Read,
}eCanRxStatus;
typedef enum
{
	Can_SId_NoError = 0,
	Can_SId_Error = 1,
}eCanStatusId;
typedef enum
{
	Can_SFP_Tx0Timeout	= 0,
	
	Can_SFP_Rx0Timeout	= 8,
	Can_SFP_Rx1Timeout	= 9,
	Can_SFP_Rx2Timeout	= 10,
	Can_SFP_Rx3Timeout	= 11,
	Can_SFP_Rx4Timeout	= 12,
	Can_SFP_Rx5Timeout	= 13,
}eCan_StatusFlagPos;

typedef enum
{
    JTC_FT_Polynomial = 0,
    JTC_FT_Table = 1,
}eJTC_FricType;

#define M_4_PI											12.566370
#define M_2_PI											6.283185
#define M_PI												3.141592
#define M_PI_2											1.570796
#define M_PI_4 											0.785398
#define M_PI_8 											0.392699
#define MAXINT16										32767.0

//#define RS422
#define UARTUSB

#ifdef RS422
#define HOST_COMBAUDRATE 						5000000
#define HOST_COMTIMSEND							15
#endif

#ifdef UARTUSB
#define HOST_COMBAUDRATE 						115200
#define HOST_COMTIMSEND							40
#endif

#define HOST_COMBUFREADSIZE 				60000
#define HOST_COMBUFWRITESIZE 				1000
#define HOST_COMFRAMESSIZE					1
#define HOST_COMTIMEOUTMAX					100
#define TRAJ_POINTSMAX							12000
#define TRAJ_SEGSSMAX								100
#define JOINTS_MAX									6
#define JOINTS_FRICTABVELSIZE				20
#define JOINTS_FRICTABTEMPSIZE			20
#define JOINTS_FRICCOEFFMAX					4
#define JOINTS_PIDBUFMAX						200

#define ARMMODEL_DOF								6

#define CAN_MSGRAM_STARTADDR				0x4000AC00
#define CAN_FILTERS_MAX							0x06
#define CAN_TXBUF_MAX								0x01
#define CAN_TXBUFSIZE_CODE					0x03
#define CAN_TXDATA_DLCCODE					0x0B
#define CAN_TXDATA_LEN							0x14
#define CAN_RXBUF_MAX								0x06
#define CAN_RXFIFO0_MAX							(0x40 - CAN_RXBUF_MAX)
#define CAN_RXBUFSIZE_CODE					0x01
#define CAN_RXDATA_LEN							0x0C
#define CAN_TIMEOUTMAX							5

typedef struct
{
	bool										active;
	eHost_TxFrameStatus			status;
	uint8_t									frame[HOST_COMBUFWRITESIZE];
	uint8_t									len;
	void										(*funPrepareFrame)(void);
}sHost_TxFrame;
typedef struct
{
	bool										isReading;
	eHost_RxFrameConsist		consist;
	eHost_RxFrameStatus			status;
	eHost_RxDataStatus 			dataStatus;
	eHost_FrameType					frameType;
	uint16_t								receivedLength;
	uint16_t								expectedLength;
}sHost_RxFrame;
typedef struct
{
	eHost_ComStatus					status;
	uint8_t									bufread0[HOST_COMBUFREADSIZE];
	uint8_t									bufread1[HOST_COMBUFREADSIZE];
	uint8_t 								bufwrite[HOST_COMBUFWRITESIZE];
	sHost_TxFrame						txFrames[HOST_COMFRAMESSIZE];
	sHost_RxFrame						rxFrame;
	uint8_t									framenum;
	uint32_t								timeout;
	bool										flagTimeout;
	bool										firstRun;
}sHost_Com;
typedef struct
{
	int16_t 					pos[JOINTS_MAX];
	int16_t 					vel[JOINTS_MAX];
	int16_t 					acc[JOINTS_MAX];
}sTrajPoint;
typedef struct
{
	double 						pos[JOINTS_MAX];
	double 						vel[JOINTS_MAX];
	double 						acc[JOINTS_MAX];
}sTrajPointDouble;
typedef struct
{
	eTrajComStatus		comStatus;
	eTrajExecStatus		targetTES;
	eTrajExecStatus		currentTES;
	uint16_t					numTraj;
	uint16_t					numSeg;
	uint16_t					maxSeg;
	uint16_t					maxPoints;
	bool							flagReadSeg[TRAJ_SEGSSMAX];
	uint16_t					numPointsSeg[TRAJ_SEGSSMAX];
	uint16_t					numRecPoints;
	
	uint16_t					stepTime;
	sTrajPoint				points[TRAJ_POINTSMAX];
	
	uint32_t					maxInterPoints;
	uint32_t					numInterPoint;
	sTrajPointDouble	startPoint;
	sTrajPointDouble	interpolatePoint;
	sTrajPointDouble	endPoint;
}sTrajectory;
typedef struct
{
	eJoint_FSM		targetFsm;
	eJoint_FSM		currentFsm;
	uint8_t				mcCurrentError;
	uint8_t				mcOccuredError;
	uint8_t				currentError;
	uint8_t				currentWarning;
	uint16_t			internallErrors;							//Bledy - wszystkie flagi biezacych bled�w wewnetrznych zebrane w jeden rejestr do wyslania do hosta
	uint16_t			internallOccuredErrors;				//Bledy - wszystkie flagi bled�w wewnetrznych zebrane w jeden rejestr do wyslania do hosta
	
	bool					flagSetPosOverlimit;					//Pozycja - wyliczona pozycja jest poza zakresem
	bool					flagSetVelOverlimit;					//Predkosc - wyliczona predkosc jest poza zakresem
	bool					flagSetAccOverlimit;					//Przyspieszenie - wyliczone przyspieszenie jest poza zakresem
	bool					flagSetTorqueOverlimit; 			//Moment - wyliczony moment jest poza zakresem
	bool					flagPosErrorOverlimit;				//Pozycja - uchyb pozycji katowej jest poza zakresem
	bool					flagFricTableValueOverlimit; 	//Moment - Temperatura lub predkosc poza zakresem w tablicy (brak poprawnej wartosci w tablicy tarcia)
	
	bool					flagCanError;									// Flaga - dowolny blad jointa odebrany z CAN (suma logiczna wszystkich flg bledow odebranych przez CAN)
	bool					flagJtcError;									// Flaga - dowolny blad jointa powstaly w JTC (suma logiczna wszystkich flg bledow wewnetrznych)
	
	double 				setPos;					//Pozycja - wartosc zadana
	double				setVel;					//Predkosc - wartosc zadana
	double				setAcc;					//Przyspieszenie - wartosc zadana
	double				setTorque;			//Moment - wartosc zadana
	
	double 				setPosTemp;			//Pozycja - tymczasowa wartosc zadana
	double				setVelTemp;			//Predkosc - tymczasowa wartosc zadana
	double				setAccTemp;			//Przyspieszenie - tymczasowa wartosc zadana
	double				setTorqueTemp;	//Moment - tymczasowa wartosc zadana
	
	double 				currentPos;			//Pozycja - wartosc aktualna
	double 				currentVel;			//Predkosc - wartosc aktualna
	double 				currentAcc;			//Przyspieszenie - wartosc aktualna
	double				currentTorque;	//Moment - wartosc aktualna
	double				currentTemp;		//Teperatura - wartosc aktualna
	
	double				limitPosMin;				//Limit wartosci pozycji dla danego jointa - wartosc minimum
	double				limitPosMax;				//Limit wartosci pozycji dla danego jointa - wartosc maximum
	double				limitVelMin;				//Limit wartosci predkosci dla danego jointa - wartosc minimum
	double				limitVelMax;				//Limit wartosci predkosci dla danego jointa - wartosc maximum
	double				limitAccMin;				//Limit wartosci przyspieszenia dla danego jointa - wartosc minimum
	double				limitAccMax;				//Limit wartosci przyspieszenia dla danego jointa - wartosc maximum
	double				limitTorqueMin;			//Limit wartosci momentu dla danego jointa - wartosc minimum
	double				limitTorqueMax;			//Limit wartosci momentu dla danego jointa - wartosc maximum
	double				limitTempMin;				//Limit wartosci temperatury dla danego jointa - wartosc minimum
	double				limitTempMax;				//Limit wartosci temperatury dla danego jointa - wartosc maximum
	double				limitPosErrorMin;		//Limit wartosci uchybu pozycji dla danego jointa - wartosc minimum
	double				limitPosErrorMax;		//Limit wartosci uchybu pozycji dla danego jointa - wartosc maximum
	
	double				maxPosCom;			//Maksymalna wartosc pozycji do obliczania zakres�w przy przesylaniu danych
	double				maxVelCom;			//Maksymalna wartosc predkosci do obliczania zakres�w przy przesylaniu danych
	double				maxAccCom;			//Maksymalna wartosc przyspieszenia do obliczania zakres�w przy przesylaniu danych
	double				maxTorqueCom;		//Maksymalna wartosc momentu do obliczania zakres�w przy przesylaniu danych
	
	double				fricTorque;																								//Moment - wartosc momentu tarcia odczytana z tablicy kompensacji
	double				fricTableVelMin;																					//Minimalna predkosc dla ktorej okreslono moment tarcia
	double				fricTableVelMax;																					//Maksymalna predkosc dla ktorej okreslono moment tarcia
	double				fricTableTempMin;																					//Minimalna temperatura dla ktorej okreslono moment tarcia
	double				fricTableTempMax;																					//Maksymalna temperatura dla ktorej okreslono moment tarcia
	double				fricTableVelIdx[JOINTS_FRICTABVELSIZE];										//Wartosc predkosci dla indeksu 0 w tablicy kompensacji tarcia
	double				fricTableTempIdx[JOINTS_FRICTABTEMPSIZE];									//Wartosc temperatury dla indeksu 0 w tablicy kompensacji tarcia
	double				fricTable[JOINTS_FRICTABVELSIZE][JOINTS_FRICTABTEMPSIZE]; //Tablica ze wszpolczynnikami kompensacji tarcia
	double				fricCoeff[JOINTS_FRICCOEFFMAX];														//Tablica wsp�lczynnik�w do r�wnania tarcia
	
	
	double				idSetPos;				//Pozycja katowa - wartosc do liczenia dynamiki odwrotnej
	double				idSetVel;				//Predkosc katowa - wartosc do liczenia dynamiki odwrotnej
	double				idSetAcc;				//Przyspieszenie katowe - wartosc do liczenia dynamiki odwrotnej
	double				idTorque;				//Moment - wartosc momentu wyliczona z zadania dynamiki
	
	double				pidKp;																//PID - wsp�lczynnik kp
	double				pidKi;																//PID - wsp�lczynnik ki
	double				pidKd;																//PID - wsp�lczynnik kd
	double				pidDt;																//PID - krok czasowy
	double				pidErrorCurrent;											//PID - aktualny uchyb
	double				pidErrorMeanCurrent;									//PID - aktualna srenia uchyb�w
	double				pidErrorMeanPrev;											//PID - poprzednia srednia uchyb�w
	uint32_t			pidErrorBufIdx;												//PID - index elementu w buforze
	double				pidErrorBuf[JOINTS_PIDBUFMAX];				//PID - bufor zawierajacy historie uchyb�w do filtracji r�zniczkowania
	double				pidErrorDiv;													//PID - pochodna z uchybu
	double				pidErrorInt;													//PID - calka uchybu
	double				pidErrorIntMin;												//PID - saturacja calki uchybu - wartosc minimalna
	double				pidErrorIntMax;												//PID - saturacja calki uchybu - wartosc maksymalna
	double				pidTorque;														//PID - wyjscie z regulatora
}sJoint;
typedef struct
{
	uint8_t				sft;
	uint8_t				sfec;
	uint16_t			sfid1;
	uint16_t			sfid2;
	uint32_t			r0;
}sCanFilter;
typedef struct
{
	eCanTxStatus	status;
	uint32_t			timeoutCnt;
	bool					flagTimeout;
	uint64_t			frameTotalCnt;
	
	uint8_t				esi;
	uint8_t				xtd;
	uint8_t				rtr;
	uint32_t			id;
	uint8_t				mm;
	uint8_t				efc;
	uint8_t				fdf;
	uint8_t				brs;
	uint8_t				dlc;
	uint8_t				bytes[CAN_TXDATA_LEN];
	
	uint32_t			r0;
	uint32_t			r1;
	uint32_t			data[CAN_TXDATA_LEN/4];
}sCanTxMsg;
typedef struct
{
	eCanRxStatus	status;
	uint32_t			timeoutCnt;
	bool					flagTimeout;
	uint64_t			frameTotalCnt;
	
	uint8_t				esi;
	uint8_t				xtd;
	uint8_t				rtr;
	uint32_t			id;
	uint8_t				anmf;
	uint8_t				fidx;
	uint8_t				fdf;
	uint8_t				brs;
	uint8_t				dlc;
	uint16_t			rxts;
	uint8_t				bytes[CAN_RXDATA_LEN];
	
	uint32_t			r0;
	uint32_t			r1;
	uint32_t			data[CAN_RXDATA_LEN/4];
}sCanRxMsg;
typedef struct
{
	uint16_t 			filterAddrOffset;
	uint16_t 			txBufAddrOffset;
	uint16_t 			rxBufAddrOffset;
	uint16_t 			rxFifo0AddrOffset;
	uint32_t* 		filterAddr;
	uint32_t* 		txBufAddr;
	uint32_t* 		rxBufAddr;
	uint32_t* 		rxFifo0Addr;
	
	eCanStatusId	statusId;
	uint32_t			statusFlags;
	uint32_t			statusOccurredFlags;
	
	sCanFilter		Filters[CAN_FILTERS_MAX];
	sCanTxMsg			TxMsgs[CAN_TXBUF_MAX];
	sCanRxMsg			RxMsgs[CAN_RXBUF_MAX];
}sCan;
typedef struct
{
	eJTC_FSM			targetFsm;
	eJTC_FSM			currentFsm;
	bool					errorModeReq;
	bool					initModeReq;
	bool					teachingModeReq;
	bool					holdposModeReq;
	bool					operateModeReq;
	
	eJTC_FricType	fricType;								//Rodzaj uzywanej kompensacji tarcia: 0 - wielomian 3 stopnia, 1 - tablica wsp�lczynnik�w 20x20
	
	uint16_t			errors;									// Wszystkie flagi biezacych bledow
	uint16_t			occuredErrors;					// Wszystkie flagi bledow
	bool					emergencyInput;					 
	bool					emergencyOutput;
	bool					internalError;					// Dowolny wewnetrzny Blad w pracy JTC - powoduje ustawienie emergencyOutput
	bool					externalError;					// Dowolny zewnetrzny Blad w pracy JTC - nie powoduje ustawienia emergencyOutput
	bool					internalJointsError;		// Blad w pracy JTC zwiazany z jointami - powoduje ustawienie emergencyOutput
	bool					internalCanError;				// Blad w pracy JTC zwiazany z CAN - powoduje ustawienie emergencyOutput
	bool					internalComError;				// Blad w pracy JTC zwiazany z COM - powoduje ustawienie emergencyOutput
	bool					externaljointsError;		// Blad w dowolnym joint odebrany przez CAN - nie powoduje ustawienia emergencyOutput

	uint8_t				jtcInitStatus;
	uint8_t				jointsInitStatus;
	bool					flagInitGetFrictionTable;
	bool					flagInitGetPidParam;
	bool					flagInitGetArmModel;
	bool					flagInitJointsTab[JOINTS_MAX];
}sJtc;
typedef struct
{
	double				origin[6];
}sArmModelJoint;
typedef struct
{
	double				origin[6];
	double				mass;
	double				innertia[6];
}sArmModelLink;
typedef struct
{
	sArmModelJoint	Joints[ARMMODEL_DOF+1];
	sArmModelLink		Links[ARMMODEL_DOF+1];
}sArmModel;
typedef struct
{
	volatile 	uint32_t 			tick;
						sJtc					Jtc;
						sJoint				Joints[JOINTS_MAX];
						sCan					Can;
						sArmModel			Arm;
}sControl;

void Control_SystemConf(void);
void Control_Delay(uint32_t ms);
void Control_TrajClear(void);
void Control_SetDefualtArmModel(void);
void Control_ClearCurrentErrors(void);
void Control_ClearOccuredErrors(void);

#include "Com.h"
#include "Can.h"
#include "Joints.h"
#include "RNEA.h"

#endif
