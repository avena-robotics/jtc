#ifndef COM_H
#define COM_H

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

typedef enum
{
    Host_FT_Header = 155,
    Host_FT_null = 0,
    Host_FT_JtcStatus,
    Host_FT_CanStatus,
    Host_FT_CanMissingFrameLevel,
    Host_FT_JointsStatus,
    Host_FT_JointsValues,
    Host_FT_ReceivedFrameResponse,
    Host_FT_Trajectory,
    Host_FT_FrictionTable,
    Host_FT_FrictionTableUseDefault,
    Host_FT_PidParam,
    Host_FT_PidParamUseDefault,
    Host_FT_ArmModel,
    Host_FT_ArmModelUseDefault,
}eHost_FrameType;
typedef enum
{
    Host_RxFS_Idle = 0,
    Host_RxFS_IsReceived = 1,
    Host_RxFS_WasReceived = 2,
    Host_RxFS_ErrorIncorrectHeader = 3,
    Host_RxFS_ErrorIncorrectFrameType = 4,
    Host_RxFS_ErrorIncorrectCrc = 5,
    Host_RxFS_ErrorTimeout = 6,
}eHost_RxFrameStatus;
typedef enum
{
    Host_RxDS_NoError = 0, //bez bledu
    Host_RxDS_TrajTooManyPoints, //zbyt dluga trajektoria
    Host_RxDS_TrajTooManySegs, //zbyt wiele segmentow
    Host_RxDS_TrajIncorrectSegOrder, //segmenty w niepoprawnej kolejnosci
    Host_RxDS_TrajIncorrectStepTime, //niepoprawny krok czasowy (prawdopodobnie stepTime = 0)
}eHost_RxDataStatus;
typedef struct
{
    uint32_t    numOfTraj; //numer trajektorii
    uint32_t    lenPointsInTraj; //liczba sampli w trajektorii
    uint32_t    lenSegsInTraj; // liczba segmentów na które dzielona jest trajektoria
    uint32_t    stepTime; //rok czasowy w danej trajektorii

    uint32_t    lenPointsInSeg; //liczba sampli w trajektorii
    uint32_t    lenBytesInSeg; //liczba bajtów wsegmencie
    uint32_t    numOfSeg; //numer segmentu w danej trajektorii

    uint16_t    crc; //crc dla danego segmentu
    QByteArray  strToSend;
    QList<QList<double>>    value;

}sTrajSeg;
typedef struct
{
    bool                    isSend;
    uint32_t                numOfTraj; //numer trajektorii
    uint32_t                lenPointsInTraj; //liczba sampli w trajektorii
    uint32_t                lenSegsInTraj; // liczba segmentów na które dzielona jest trajektoria
    uint32_t                stepTime; //rok czasowy w danej trajektorii
    QList<sTrajSeg>         seg;

    QList<QList<double>>    value;
    int                     numOfSegToSend;
    QString                 trajString;

}sTraj;
class Com : public QObject
{
    Q_OBJECT
public:
    explicit Com(QObject *parent = nullptr);
    uint16_t Com_Crc16(QByteArray packet, uint32_t nBytes);
    bool Com_OpenPort(QString name);
    bool Com_ClosePort(void);

    void Com_ReadTrajInt16(void);
    void Com_ReadTrajFloat(void);
    void Com_ConvertTrajToDoubleFromInt16(QString line);
    void Com_ConvertTrajToDoubleFromDoubleString(QString line);
    void Com_PrepareTrajectorySegments(sTrajSeg* seg, int num);
    void Com_PrepareTrajectorySegmentToSend(sTrajSeg* seg);
    void Com_PrepareTrajectoryToSend(void);
    void Com_ReadFricTableFloat(void);
    void Com_ConvertFricTableToDoubleFromDoubleString(QString line);
    void Com_PrepareFrictionTableToSend(int max);
    void Com_ReadFrameReceivedFrameResponse(uint8_t* buf);
    uint16_t Com_Crc16v2(uint8_t *packet, uint32_t nBytes);

public slots:
    void Com_SendTrajectory(void);
    void Com_SendFrictionTable();
    void Com_ReadFrame(uint8_t* buf);

signals:

private:
    const double JOINT_POSMAX = M_PI;
    const double JOINT_VELMAX = 2*M_PI;
    const double JOINT_ACCMAX = 4*M_PI;
    const double MAX_INT16 = 32767.0;
//    const int TRAJ_MAXPOINTINSEG = 1560;
    const int TRAJ_MAXPOINTINSEG = 156;

public:
    QTimer *timerSerial;
    QSerialPort* serial;
    sTraj trajectory;
    QString frictTableString;
    QList<QList<float>> frictableValue;
    QByteArray frictionTableStrToSend;
    QString str;
    QByteArray readStr;

};

#endif // COM_H
