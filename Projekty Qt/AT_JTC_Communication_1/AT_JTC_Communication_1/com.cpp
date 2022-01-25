#include "com.h"

Com::Com(QObject *parent) : QObject(parent)
{
    timerSerial = new QTimer(this);
    serial = new QSerialPort(this);
    connect(timerSerial, SIGNAL(timeout()), this, SLOT(Com_SendTrajectory()));
}
uint16_t Com::Com_Crc16(QByteArray packet, uint32_t nBytes)
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
uint16_t Com::Com_Crc16v2(uint8_t* packet, uint32_t nBytes)
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
union conv32
{
    uint32_t u32; // here_write_bits
    float    f32; // here_read_float
};
bool Com::Com_OpenPort(QString name)
{
    serial->setPortName(name);
    serial->setBaudRate(QSerialPort::Baud115200);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    if(!(serial->isOpen()))
    {
        serial->open(QIODevice::ReadWrite);
        if(serial->isOpen())
        {
//            timerSerial->start(100);
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}
bool Com::Com_ClosePort()
{
    serial->close();
    timerSerial->stop();
}
void Com::Com_SendTrajectory()
{
    serial->write(trajectory.seg[trajectory.numOfSegToSend].strToSend, trajectory.seg[trajectory.numOfSegToSend].lenBytesInSeg);
    qDebug() << "Wysłano segment number " << trajectory.numOfSegToSend << " z " << trajectory.lenSegsInTraj << " segmentów";
}
void Com::Com_SendFrictionTable()
{
    serial->write(frictionTableStrToSend, frictionTableStrToSend.length());
}
void Com::Com_ReadFrameReceivedFrameResponse(uint8_t* buf)
{
    uint16_t nd = ((uint16_t)buf[2] << 8) + ((uint16_t)buf[3] << 0);
    uint16_t crc1 = Com_Crc16v2(buf, nd-2);
    uint16_t crc2 = ((uint16_t)buf[nd-2] << 8) + ((uint16_t)buf[nd-1] << 0);
    if(crc1 == crc2)
    {
        if(buf[4] == 7)
        {
            if(buf[5] == Host_RxFS_WasReceived && buf[6] == Host_RxDS_NoError)
            {

                trajectory.numOfSegToSend++;
                if(trajectory.numOfSegToSend < trajectory.lenSegsInTraj)
                {
                    Com_SendTrajectory();
                }
            }
            else if(buf[5] == Host_RxFS_WasReceived && buf[6] != Host_RxDS_NoError)
            {
                if(trajectory.numOfSegToSend < trajectory.lenSegsInTraj)
                {
                    Com_SendTrajectory();
                }
            }
        }
    }
}
void Com::Com_ReadFrame(uint8_t* buf)
{
    if(buf[0] == Host_FT_Header)
    {
        if(buf[1] == Host_FT_ReceivedFrameResponse)
        {
            Com_ReadFrameReceivedFrameResponse(buf);
        }
    }
}
void Com::Com_ReadTrajInt16()
{
    QString fileName;
//    fileName = QFileDialog::getOpenFileName(nullptr, tr("Open File"), "C:\\Users\\Dawid\\Moj dysk\\Avena Technologie\\Projekty Qt\\AT_JTC_Communication_1\\TrajectoryInt.csv", tr("Image Files (*.txt *.csv)"));
    fileName = "C:\\Users\\Dawid\\Moj dysk\\Avena Technologie\\Projekty Qt\\AT_JTC_Communication_1\\TrajectoryInt.csv";
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    while (!file.atEnd())
    {
        QString line = file.readLine();
        trajectory.trajString += line;
        Com_ConvertTrajToDoubleFromInt16(line);
    }
}
void Com::Com_ReadTrajFloat()
{
    QString fileName;
    fileName = QFileDialog::getOpenFileName(nullptr, tr("Open File"), "C:\\Users\\Dawid\\Moj dysk\\Avena Technologie\\Projekty uC\\AT_JtcTest\\Trajectory", tr("Image Files (*.txt *.csv)"));
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    while (!file.atEnd())
    {
        QString line = file.readLine();
        trajectory.trajString += line;
        Com_ConvertTrajToDoubleFromDoubleString(line);
    }
}
void Com::Com_ConvertTrajToDoubleFromInt16(QString line)
{
    QStringList lineList = line.split(",");
    QList<double> doubleLineList;
    doubleLineList.append(lineList[0].toDouble());
    for(int i=1;i<8;i++)
        doubleLineList.append(lineList[i].toDouble() / MAX_INT16 * JOINT_POSMAX);
    for(int i=8;i<15;i++)
        doubleLineList.append(lineList[i].toDouble() / MAX_INT16 * JOINT_VELMAX);
    for(int i=15;i<22;i++)
        doubleLineList.append(lineList[i].toDouble() / MAX_INT16 * JOINT_ACCMAX);

    trajectory.value.append(doubleLineList);
}
void Com::Com_ConvertTrajToDoubleFromDoubleString(QString line)
{
    QStringList lineList = line.split(",");
    QList<double> doubleLineList;
    doubleLineList.append(lineList[0].toDouble());
    for(int i=1;i<8;i++)
        doubleLineList.append(lineList[i].toDouble());
    for(int i=8;i<15;i++)
        doubleLineList.append(lineList[i].toDouble());
    for(int i=15;i<22;i++)
        doubleLineList.append(lineList[i].toDouble());

    trajectory.value.append(doubleLineList);
}
void Com::Com_PrepareTrajectorySegments(sTrajSeg* seg, int num)
{
    seg->numOfTraj = trajectory.numOfTraj;
    seg->stepTime = trajectory.stepTime;
    seg->lenPointsInTraj = trajectory.lenPointsInTraj;
    seg->lenSegsInTraj = trajectory.lenSegsInTraj;

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

    seg->lenBytesInSeg = 42 * seg->lenPointsInSeg + 14;

    int startIdx = seg->numOfSeg * TRAJ_MAXPOINTINSEG;
    int finishIdx = startIdx + seg->lenPointsInSeg;
    for(int i=startIdx;i<finishIdx;i++)
    {
        seg->value.append(trajectory.value[i]);
    }
}
void Com::Com_PrepareTrajectorySegmentToSend(sTrajSeg* seg)
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
        for(uint32_t j=1;j<8;j++)
        {
            int16_t val = seg->value[i][j] / JOINT_POSMAX * MAX_INT16;
            seg->strToSend.append(val >> 8);
            seg->strToSend.append(val >> 0);
        }
        for(uint32_t j=8;j<15;j++)
        {
            int16_t val = seg->value[i][j] / JOINT_VELMAX * MAX_INT16;
            seg->strToSend.append(val >> 8);
            seg->strToSend.append(val >> 0);
        }
        for(uint32_t j=15;j<22;j++)
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
void Com::Com_PrepareTrajectoryToSend()
{
    trajectory.numOfTraj = 0;
    trajectory.stepTime = 10;
    trajectory.lenPointsInTraj = trajectory.value.length();
    trajectory.lenSegsInTraj = (trajectory.lenPointsInTraj / TRAJ_MAXPOINTINSEG) + 1;
    trajectory.seg.resize(trajectory.lenSegsInTraj);
    for(int i=0;i<trajectory.lenSegsInTraj;i++)
    {
        Com_PrepareTrajectorySegments(&trajectory.seg[i], i);
        Com_PrepareTrajectorySegmentToSend(&trajectory.seg[i]);
        qDebug() << "numOfSeg = " << trajectory.seg[i].numOfSeg;
        qDebug() << "lenPointsInTraj = " << trajectory.seg[i].lenPointsInTraj;
        qDebug() << "lenSegsInTraj = " << trajectory.seg[i].lenSegsInTraj;
        qDebug() << "lenPointsInSeg = " << trajectory.seg[i].lenPointsInSeg;
        qDebug() << "lenBytesInSeg = " << trajectory.seg[i].lenBytesInSeg;
        qDebug() << "\r\n";
    }
    trajectory.numOfSegToSend = 0;
    trajectory.isSend = true;
}
void Com::Com_ReadFricTableFloat()
{
    QString fileName;
//    fileName = QFileDialog::getOpenFileName(nullptr, tr("Open File"), "C:\\Users\\Dawid\\Moj dysk\\Avena Technologie\\Projekty uC\\AT_JtcTest\\Friction", tr("Image Files (*.txt *.csv)"));
    fileName = "C:\\Users\\Dawid\\Moj dysk\\Avena Technologie\\Projekty Qt\\AT_JTC_Communication_1\\FricTableJoint0.csv";
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    while (!file.atEnd())
    {
        QString line = file.readLine();
        frictTableString += line;
        Com_ConvertFricTableToDoubleFromDoubleString(line);
    }
}
void Com::Com_ConvertFricTableToDoubleFromDoubleString(QString line)
{
    QStringList lineList = line.split(",");
    QList<float> floatLineList;
    for(int i=0;i<lineList.length();i++)
        floatLineList.append(lineList[i].toFloat());

    frictableValue.append(floatLineList);
}
void Com::Com_PrepareFrictionTableToSend(int max)
{
    frictionTableStrToSend.clear();
    frictionTableStrToSend.append(Host_FT_Header);
    frictionTableStrToSend.append(Host_FT_FrictionTable);
    conv32 x;
    for(int num=0;num<max;num++)
    {
        for(int i=0;i<frictableValue.length();i++)
        {
            for(int j=0;j<frictableValue[i].length();j++)
            {
                x.f32 = frictableValue[i][j];
                frictionTableStrToSend.append(x.u32 >> 24);
                frictionTableStrToSend.append(x.u32 >> 16);
                frictionTableStrToSend.append(x.u32 >> 8);
                frictionTableStrToSend.append(x.u32 >> 0);
            }
        }
    }
    uint16_t nd = frictionTableStrToSend.length() + 4;
    frictionTableStrToSend.insert(2, (uint8_t)(nd >> 8));
    frictionTableStrToSend.insert(3, (uint8_t)(nd >> 0));
    uint16_t crc = Com_Crc16(frictionTableStrToSend, frictionTableStrToSend.length());
    frictionTableStrToSend.append(crc >> 8);
    frictionTableStrToSend.append(crc >> 0);
    qDebug() << nd;
}
