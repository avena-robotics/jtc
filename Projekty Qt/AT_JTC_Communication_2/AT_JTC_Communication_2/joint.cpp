#include "joint.h"
#include "mainwindow.h"
Joint::Joint(QObject *parent) : QObject(parent)
{
    pidKp = 0.0;
    pidKi = 0.0;
    pidKd = 0.0;
    pidErrorIntMin = 0.0;
    pidErrorIntMax = 0.0;
    pos = 0.0;
    vel = 0.0;
    torque = 0.0;
    currentFsm = 0;
    mcCurrentError = 0;
    mcOccuredError = 0;
    currentError = 0;
    currentWarning = 0;
}
union conv32
{
    uint32_t u32; // here_write_bits
    float    f32; // here_read_float
};
uint16_t Joint::Com_Crc16(QByteArray packet, uint32_t nBytes)
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
void Joint::ConvertFricTableToDoubleFromDoubleString(QString line)
{
    QStringList lineList = line.split(",");
    QList<float> floatLineList;
    for(int i=0;i<lineList.length();i++)
        floatLineList.append(lineList[i].toFloat());

    fricTableValue.append(floatLineList);
}
void Joint::ReadFrictionTableFloat()
{
//    QString fileName = QFileDialog::getOpenFileName(nullptr, tr("Open File"), "C:\\Users\\Dawid\\Moj dysk\\Avena Technologie\\Projekty uC\\AT_JtcTest\\Friction", tr("Image Files (*.txt *.csv)"));
    QFile file(fricTableFilePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    fricTableString.clear();
    fricTableValue.clear();
    while (!file.atEnd())
    {
        QString line = file.readLine();
        fricTableString += line;
        ConvertFricTableToDoubleFromDoubleString(line);
    }
}
void Joint::PrepareFrictionTableToSend(void)
{
    fricTableStrToSend.clear();
    conv32 x;
    for(int i=0;i<fricTableValue.length();i++)
    {
        for(int j=0;j<fricTableValue[i].length();j++)
        {
            x.f32 = fricTableValue[i][j];
            fricTableStrToSend.append(x.u32 >> 24);
            fricTableStrToSend.append(x.u32 >> 16);
            fricTableStrToSend.append(x.u32 >> 8);
            fricTableStrToSend.append(x.u32 >> 0);
        }
    }
}
void Joint::PreparePidParamToSend()
{
    pidParamStrToSend.clear();
    conv32 x;
    for(int i=0;i<pidParamTableValue.length();i++)
    {
        x.f32 = pidParamTableValue[i];
        pidParamStrToSend.append(x.u32 >> 24);
        pidParamStrToSend.append(x.u32 >> 16);
        pidParamStrToSend.append(x.u32 >> 8);
        pidParamStrToSend.append(x.u32 >> 0);
    }
}
void Joint::ReadPidParametersFloat()
{
//  QString fileName = QFileDialog::getOpenFileName(nullptr, tr("Open File"), "C:\\Users\\Dawid\\Moj dysk\\Avena Technologie\\Projekty uC\\AT_JtcTest\\Friction", tr("Image Files (*.txt *.csv)"));
    QFile file(pidParamFilePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    pidParamString.clear();
    pidParamString = file.readLine();
    ConvertPidParametersToDoubleFromDoubleString(pidParamString);
}
void Joint::ConvertPidParametersToDoubleFromDoubleString(QString line)
{
    pidParamTableValue.clear();
    QStringList lineList = line.split(",");
    for(uint32_t i=0;i<lineList.length();i++)
        pidParamTableValue.append(lineList[i].toFloat());

    pidKp = pidParamTableValue[0];
    pidKi = pidParamTableValue[1];
    pidKd = pidParamTableValue[2];
    pidErrorIntMin = pidParamTableValue[3];
    pidErrorIntMax = pidParamTableValue[4];
}

