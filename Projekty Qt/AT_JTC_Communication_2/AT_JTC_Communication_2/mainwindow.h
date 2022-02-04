#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QObject>
#include <QDebug>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QString>
#include <QVector>
#include <qmath.h>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QElapsedTimer>
#include <QRegularExpression>
#include <QRegularExpressionMatch>
#include <QMessageBox>
#include <stdio.h>
#include <unistd.h>
#include "trajectory.h"
#include "joint.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE
typedef enum
{
    JTC_FSM_Start = 0,
    JTC_FSM_Init,
    JTC_FSM_Operate,
    JTC_FSM_Error,
}eJTC_FSM;
typedef enum
{
    Joint_FSM_Start = 0,
    Joint_FSM_Init = 1,
    Joint_FSM_ReadyToOperate = 2,
    Joint_FSM_OperatioEnable = 3,
    Joint_FSM_TransStartToInit = 10,
    Joint_FSM_TransInitToReadyToOperate = 11,
    Joint_FSM_TransReadyToOperateToOperationEnable = 12,
    Joint_FSM_TransOperationEnableToReadyToOperate = 13,
    Joint_FSM_TransFaultReactionActiveToFault = 14,
    Joint_FSM_TransFaultToReadyToOperate= 15,
    Joint_FSM_ReactionActive = 254,
    Joint_FSM_Fault = 255
}eJoint_FSM;
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
}eHost_FrameType;
typedef enum
{
    Host_RxFS_Idle = 0,
    Host_RxFS_NoError,
    Host_RxFS_ErrorIncorrectHeader,
    Host_RxFS_ErrorIncorrectFrameType,
    Host_RxFS_ErrorIncorrectCrc,
    Host_RxFS_ErrorTimeout,
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
    Host_FTAS_Null = 0,
    Host_FTAS_ClearCurrentErrors,
    Host_FTAS_ClearOccuredErrors,
    Host_FTAS_Trajectory,
    Host_FTAS_FrictionTable,
    Host_FTAS_FrictionTableUseDefault,
    Host_FTAS_PidParam,
    Host_FTAS_PidParamUseDefault,
    Host_FTAS_ArmModel,
    Host_FTAS_ArmModelUseDefault,
    Host_FTAS_TrajSetExecStatus,
}eHost_FrameToAsynchroSend;
typedef enum
{
    Host_FTSS_GetJtcStatus = 0,
    Host_FTSS_GetCanStatus,
    Host_FTSS_GetCanMissingFrameLevel,
    Host_FTSS_GetJointsStatus,
    Host_FTSS_GetJointsValues,
}eHost_FrameToSynchroSend;

const int COMFRAMETOSYNCHROSENDMAX = 1;
const int COMBUFREADMAX = 10000;
const int COMTIMESEND = 25;
const int COMTIMEREAD = 15;
const int COMTIMEOUT = 1000;
const int JOINTS_MAX = 6;
const double JOINT_POSMAX = M_PI;
const double JOINT_VELMAX = 2*M_PI;
const double JOINT_ACCMAX = 4*M_PI;
const double JOINT_TORQUEMAX = 256.0;
const double MAX_INT16 = 32767.0;
const int TRAJ_MAXPOINTINSEG = 500; //max 1500
const int ARMMODEL_DOF=6;

typedef struct
{
    QString             name;
    QString             type;
    QString             parentName;
    QString             childName;
    double				origin[6];
    double              limitLower;
    double              limitUpper;
    double              limitEffort;
    double              limitVelocity;
}sArmModelJoint;
typedef struct
{
    QString             name;
    double				origin[6];
    double				mass;
    double				innertia[6];
}sArmModelLink;
typedef struct
{
    bool wasRead;
    QString armModelFilePath;
    QString armModelString;
    QByteArray armModelWriteString;

    QList<sArmModelJoint>   Joints;
    QList<sArmModelLink>	Links;
}sArmModel;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:

private slots:
    uint16_t Com_Crc16(uint8_t* packet, uint32_t nBytes);
    uint16_t Com_Crc16v2(QByteArray packet, uint32_t nBytes);
    void SetDefualtArmModel(void);
    void ShowTrajectory();
    void ShowJtcValues(void);
    void ShowJointsValues(void);
    void RedrawButtons();
    void RefreshLabels(void);
    void ShowFrictionTable(uint8_t num);
    void Com_ReadFrameJtcStatus(uint8_t* buf);
    void Com_ReadFrameReceivedFrameResponse(uint8_t* buf);
    void Com_SendTrajectory(void);
    void Com_ReadFrame(uint8_t* buf);
    void Com_PrepareArmModelToSend(void);
    void Com_SendCommandClearCurrentErrors(void);
    void Com_SendCommandClearOccuredErrors(void);
    void Com_SendCommandUseDefaultPidParam(void);
    void Com_SendCommandUseDefaultFrictionTable(void);
    void Com_SendCommandUseDefaultArmModel(void);
    void Com_SendPidParam(void);
    void Com_SendFrictionTable(void);
    void Com_SendArmModel(void);
    void Com_Timeout();
    void Com_Send(void);
    void Com_Read(void);
    void Com_RefreshSerialPort();
    void on_Com_OpenButt_clicked();
    void on_Com_CloseButt_clicked();
    void on_Com_SendTrajectoryToJtc_clicked();
    void on_Com_ReadFrictionTableFloat_clicked();
    void on_Com_SendFrictionTableToJtc_clicked();
    void ArmModelReadStringFromFile(void);
    void ArmModelShowLinks(void);
    void ArmModelShowJoins(void);
    sArmModelLink ArmModelLinkClear();
    sArmModelLink ArmModelParseLink(QString str);
    sArmModelJoint ArmModelJointClear();
    sArmModelJoint ArmModelParseJoint(QString str);
    void ArmModelParse(void);
    void on_Com_ReadArmModel_clicked();
    void on_Com_SendArmModelToJtc_clicked();
    void on_Com_ReadTrajectoryInt16_clicked();
    void on_Com_SendCommanUsDefaultFrictionTableToJtc_clicked();
    void Com_SendCommandTrajSetExecStatus();
    void ShowPidParametersTable(void);
    void on_Com_ReadPidParam_clicked();
    void on_Com_SendCommandUseDefaultPidParamlToJtc_clicked();
    void on_Com_SendCommandUseDefaultArmModelToJtc_clicked();
    void on_Com_ReadTextEdit_2_clicked();
    void on_Com_ReadDebugTextedit_clicked();
    void on_Com_SendPidParamlToJtc_clicked();
    void on_Traj_TrajectoryStop_clicked();
    void on_Traj_TrajectoryPause_clicked();
    void on_Traj_TrajectoryExecute_clicked();
    void on_Com_OnOffSynchroTransmision_clicked();
    void on_JTC_ClearErrors_clicked();
    void on_JTC_ClearOccuredErrors_clicked();
    void Com_ButtonSetEnable(bool state);

private:
    Ui::MainWindow *ui;
    sArmModel Arm;
    uint8_t jtcFsm;
    uint16_t jtcErrors;
    uint16_t jtcOccuredErrors;
    uint8_t jtcInitStatus;
    uint8_t jointsInitStatus;

    uint8_t canStatus;
    uint32_t canStatusFlags;
    uint32_t canOccuredFlags;
    Trajectory traj;
    Joint joints[JOINTS_MAX];
    QTimer *timerSerialSend;
    QTimer *timerSerialRead;
    QTimer *timerSerialTimeout;
    QTimer *timerLabels;
    QSerialPort* serial;
    QStandardItemModel* trajStandardItemModel;
    QStandardItemModel* frictionTableStandardItemModel;
    QStandardItemModel* pidParametersStandardItemModel;
    QStandardItemModel* jtcParametersStandardItemModel;
    QStandardItemModel* jointsParametersStandardItemModel;
    QStandardItemModel* armModelLinksStandardItemModel;
    QStandardItemModel* armModelJointsStandardItemModel;
    bool comAsynchronicSend; //wysylanie ramek asynchronicznych
    bool comSynchroTransmisionEnable;
    uint32_t numFrameToAsynchroSend;
    uint32_t numFrameToSynchroSend;
    QByteArray comWriteString;
    QByteArray fricTableWriteString;
    QByteArray pidParamWriteString;
    QByteArray comReadString;
    bool frictionWasRead;
    bool pidParamWasRead;
};
#endif // MAINWINDOW_H
