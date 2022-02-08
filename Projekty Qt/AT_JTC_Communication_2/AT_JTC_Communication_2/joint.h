#ifndef JOINT_H
#define JOINT_H

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
class Joint : public QObject
{
    Q_OBJECT
public:
    explicit Joint(QObject *parent = nullptr);
    uint16_t Com_Crc16(QByteArray packet, uint32_t nBytes);
    void ConvertFricTableToDoubleFromDoubleString(QString line);
    void ReadFrictionTableFloat(void);
    void PrepareFrictionTableToSend(void);
    void PrepareFrictionPolynomialCoeffsToSend(void);
    void PreparePidParamToSend(void);
    void ReadPidParametersFloat(void);
    void ConvertPidParametersToDoubleFromDoubleString(QString line);
    void ConvertFrictionPolynomialCoeffsToDoubleFromDoubleString(QString line);

    QString fricTableString;
    QString pidParamString;
    QList<QList<float>> fricTableValue;
    QByteArray fricTableStrToSend;
    QByteArray fricPolynomialCooefsStrToSend;
    QByteArray pidParamStrToSend;
    QString fricTableFilePath;
    QString pidParamFilePath;
    QList<float> pidParamTableValue;
    float pidKp;				//PID - wspólczynnik kp
    float pidKi;				//PID - wspólczynnik ki
    float pidKd;				//PID - wspólczynnik kd
    float pidErrorIntMin;		//PID - saturacja calki uchybu - wartosc minimalna
    float pidErrorIntMax;		//PID - saturacja calki uchybu - wartosc maksymalna

    float       pos;
    float       vel;
    float       torque;
    float       temperature;
    uint8_t     currentFsm;
    uint8_t     mcCurrentError;
    uint8_t     mcOccuredError;
    uint8_t     currentError;
    uint8_t     currentWarning;
    uint16_t    internallErrors;
    uint16_t    internallOccuredErrors;
    QList<float> fricPolynomialCoeffs;

signals:

};
#endif // JOINT_H
