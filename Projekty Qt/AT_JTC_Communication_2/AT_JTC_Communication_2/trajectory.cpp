#include "trajectory.h"
#include "mainwindow.h"
uint16_t Trajectory::Com_Crc16(QByteArray packet, uint32_t nBytes)
{
    uint16_t crc = 0;
    for(uint32_t byte = 0; byte < nBytes; byte++)
    {
        crc = crc ^ ((uint16_t)packet[byte] << 8);
        for (uint8_t bit = 0; bit < 8; bit++)
            if(crc & 0x8000) 	crc = (crc << 1) ^ 0x1021;
            else							crc = crc << 1;
    }
    return crc;
}
Trajectory::Trajectory(QObject *parent) : QObject(parent)
{
    TrajectoryClear();
}
void Trajectory::ReadTrajectoryInt16()
{
    QString fileName = trajFilePath;
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;
    TrajectoryClear();
    while (!file.atEnd())
    {
        QString line = file.readLine();
        trajString += line;
        ConvertTrajToDoubleFromInt16(line);
    }
}
void Trajectory::ConvertTrajToDoubleFromInt16(QString line)
{
    QStringList lineList = line.split(",");
    QList<double> doubleLineList;
    uint16_t idx = 0;
    doubleLineList.append(lineList[idx++].toDouble());
    for(int i=0;i<JOINTS_MAX;i++)
        doubleLineList.append(lineList[idx++].toDouble() / MAX_INT16 * JOINT_POSMAX);
    for(int i=0;i<JOINTS_MAX;i++)
        doubleLineList.append(lineList[idx++].toDouble() / MAX_INT16 * JOINT_VELMAX);
    for(int i=0;i<JOINTS_MAX;i++)
        doubleLineList.append(lineList[idx++].toDouble() / MAX_INT16 * JOINT_ACCMAX);

    value.append(doubleLineList);
}
void Trajectory::PrepareTrajectorySegments(sTrajSeg *seg, int num)
{
    seg->numOfTraj = numOfTraj;
    seg->stepTime = stepTime;
    seg->lenPointsInTraj = lenPointsInTraj;
    seg->lenSegsInTraj = lenSegsInTraj;

    seg->numOfSeg = num;
    if((seg->lenPointsInTraj % TRAJ_MAXPOINTINSEG) == 0)
    {
        seg->lenPointsInSeg = TRAJ_MAXPOINTINSEG;
    }
    else
    {
        if(seg->numOfSeg < (seg->lenSegsInTraj - 1))
            seg->lenPointsInSeg = TRAJ_MAXPOINTINSEG;
        else
            seg->lenPointsInSeg = seg->lenPointsInTraj % TRAJ_MAXPOINTINSEG;
    }

    seg->lenBytesInSeg = 36 * seg->lenPointsInSeg + 14;

    int startIdx = seg->numOfSeg * TRAJ_MAXPOINTINSEG;
    int finishIdx = startIdx + seg->lenPointsInSeg;
    for(int i=startIdx;i<finishIdx;i++)
    {
        seg->value.append(value[i]);
    }
}
void Trajectory::PrepareTrajectorySegmentToSend(sTrajSeg *seg)
{
    seg->strToSend.clear();
    seg->strToSend.append(Host_FT_Header);
    seg->strToSend.append(Host_FT_Trajectory);

    seg->strToSend.append(seg->numOfTraj >> 8);
    seg->strToSend.append(seg->numOfTraj >> 0);

    seg->strToSend.append(seg->numOfSeg >> 8);
    seg->strToSend.append(seg->numOfSeg >> 0);

    seg->strToSend.append(seg->lenSegsInTraj >> 8);
    seg->strToSend.append(seg->lenSegsInTraj >> 0);

    seg->strToSend.append(seg->stepTime >> 8);
    seg->strToSend.append(seg->stepTime >> 0);

    for(uint32_t i=0;i<seg->lenPointsInSeg;i++)
    {
        for(uint32_t j=1;j<7;j++)
        {
            int16_t val = seg->value[i][j] / JOINT_POSMAX * MAX_INT16;
            seg->strToSend.append(val >> 8);
            seg->strToSend.append(val >> 0);
        }
        for(uint32_t j=7;j<13;j++)
        {
            int16_t val = seg->value[i][j] / JOINT_VELMAX * MAX_INT16;
            seg->strToSend.append(val >> 8);
            seg->strToSend.append(val >> 0);
        }
        for(uint32_t j=13;j<19;j++)
        {
            int16_t val = seg->value[i][j] / JOINT_ACCMAX * MAX_INT16;
            seg->strToSend.append(val >> 8);
            seg->strToSend.append(val >> 0);
        }
    }
    uint16_t nd = seg->strToSend.length() + 4;
    seg->strToSend.insert(2, (uint8_t)(nd >> 8));
    seg->strToSend.insert(3, (uint8_t)(nd >> 0));
    uint16_t crc = Com_Crc16(seg->strToSend, seg->strToSend.length());
    seg->strToSend.append(crc >> 8);
    seg->strToSend.append(crc >> 0);
}
void Trajectory::PrepareTrajectoryToSend()
{
    numOfTraj = 0;
    stepTime = 10;
    lenPointsInTraj = value.length();
    if((lenPointsInTraj % TRAJ_MAXPOINTINSEG) == 0)
        lenSegsInTraj = (lenPointsInTraj / TRAJ_MAXPOINTINSEG);
    else
        lenSegsInTraj = (lenPointsInTraj / TRAJ_MAXPOINTINSEG) + 1;
    seg.resize(lenSegsInTraj);
    for(uint32_t i=0;i<lenSegsInTraj;i++)
    {
        PrepareTrajectorySegments(&seg[i], i);
        PrepareTrajectorySegmentToSend(&seg[i]);
        qDebug() << "numOfSeg = " << seg[i].numOfSeg;
        qDebug() << "lenPointsInTraj = " << seg[i].lenPointsInTraj;
        qDebug() << "lenSegsInTraj = " << seg[i].lenSegsInTraj;
        qDebug() << "lenPointsInSeg = " << seg[i].lenPointsInSeg;
        qDebug() << "lenBytesInSeg = " << seg[i].lenBytesInSeg;
        qDebug() << "\r\n";
    }
    numOfSegToSend = 0;
    isSend = true;
}
void Trajectory::TrajectoryClear()
{
    wasRead = false;
    targetStatus = TES_Null;
    currentStatus = targetStatus;
    isSend = false;
    numOfTraj = 0;
    lenPointsInTraj = 0;
    lenSegsInTraj = 0;
    stepTime = 0;
    seg.clear();
    value.clear();
    numOfSegToSend = 0;
    numCurrentPoint = 0;
    trajString.clear();
}
