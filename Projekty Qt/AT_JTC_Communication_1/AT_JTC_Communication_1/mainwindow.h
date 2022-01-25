#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <QTimer>
#include <QMessageBox>
#include <QStandardItem>
#include <QStandardItemModel>
#include "com.h"



QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE



class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void ShowReadedString(void);

private slots:
    void on_Com_OpenButt_clicked();
    void on_Com_CloseButt_clicked();
    void on_Com_ReadTrajectory_clicked();
    void ShowTrajectory(void);
    void on_Com_ReadTrajectoryFloat_clicked();
    void on_Com_SendTrajectoryToJtc_clicked();
    void on_Com_ReadFrictionTableFloat_clicked();
    void ShowFrictionTable();
    void on_Com_SendFrictionTableToJtc_clicked();

private:
    const int JOINTS_MAX = 7;
    Ui::MainWindow *ui;
    Com Com;
    QStandardItemModel* trajStandardItemModel;
    QStandardItemModel* frictionTableStandardItemModel;
    QTimer *timerSerial;

};
#endif // MAINWINDOW_H
