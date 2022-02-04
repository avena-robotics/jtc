#ifndef TRAJECTORY_H
#define TRAJECTORY_H

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
typedef enum
{
    TES_Null = 0,
    TES_Stop,
    TES_Pause,
    TES_Execute,
    TES_Finish,
}eTrajExecStatus;
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
class Trajectory : public QObject
{
    Q_OBJECT
public:
    uint16_t Com_Crc16(QByteArray packet, uint32_t nBytes);
    explicit Trajectory(QObject *parent = nullptr);
    void ReadTrajectoryInt16(void);
    void ConvertTrajToDoubleFromInt16(QString line);
    void PrepareTrajectorySegments(sTrajSeg* seg, int num);
    void PrepareTrajectorySegmentToSend(sTrajSeg* seg);
    void PrepareTrajectoryToSend(void);
    void TrajectoryClear(void);

    eTrajExecStatus         targetStatus;
    eTrajExecStatus         currentStatus;
    uint32_t                numCurrentPoint;
    bool                    isSend;
    uint32_t                numOfTraj; //numer trajektorii
    uint32_t                lenPointsInTraj; //liczba sampli w trajektorii
    uint32_t                lenSegsInTraj; // liczba segmentów na które dzielona jest trajektoria
    uint32_t                stepTime; //krok czasowy w danej trajektorii
    QList<sTrajSeg>         seg;

    QList<QList<double>>    value;
    uint32_t                numOfSegToSend;
    QString                 trajString;
    bool                    wasRead;
    QString                 trajFilePath;

signals:

};

#endif // TRAJECTORY_H
